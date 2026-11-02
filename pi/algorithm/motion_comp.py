"""동적 모션 보상 + 화물 이동 판정. Frame 값만 받는 순수 계산이라 센서 드라이버를 모른다.

단위: 무게중심 변위는 cm, 가속도는 G (정지 오프셋을 뺀 값).
좌표: X = 좌우(앞왼->앞오), Y = 앞뒤(앞왼->뒤왼). 좌우 가속 ay 가 X 를, 앞뒤 가속 ax 가 Y 를 움직인다.

흐름
  rest      정지 구간 평균으로 기준(X0, Y0, 가속도 오프셋)을 잡는다. 안 흔들릴 때까지 창이 밀려간다.
  learning  보상 계수가 아직 한 축도 없다. 판정은 하지 않고 계수 표본만 모은다.
  ready     잔차 R 을 내고 화물 이동을 판정한다. 계수는 슬립 플래그가 꺼져 있을 때만 갱신한다.

화물이 미끄러지면 그 변위가 보상 계수에 섞이지 않도록, 계수 표본은 n_win 샘플 지연시킨 뒤에
그 사이 플래그가 안 켜졌을 때만 넣는다.
"""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np


def calc_comp_coef(dX, dY, ax, ay, a_min=0.1, n_min=5, prev=(0.0, 0.0)):
    """보상 계수 산출 (최소제곱, cm/G).

    dX, dY : 보정 구간의 무게중심 변위 (X - X0, Y - Y0) [cm]
    ax, ay : 정지 오프셋을 뺀 가속도 [G]
    prev   : 유효성 검사를 통과하지 못한 축은 이 값을 유지 (Cx, Cy)
    """
    dX, dY, ax, ay = map(np.asarray, (dX, dY, ax, ay))
    Cx, Cy = prev

    # 좌우 가속이 충분할 때만 Cx 갱신
    if np.sum(np.abs(ay) > a_min) >= n_min:
        Cx = np.sum(dX * ay) / np.sum(ay ** 2)

    # 앞뒤 가속이 충분할 때만 Cy 갱신
    if np.sum(np.abs(ax) > a_min) >= n_min:
        Cy = np.sum(dY * ax) / np.sum(ax ** 2)

    return Cx, Cy


def calc_cargo_disp(X, Y, X0, Y0, ax, ay, Cx, Cy):
    """차량 거동 몫을 뺀 순수 화물 변위 (dX_cargo, dY_cargo) [cm]."""
    return (X - X0) - Cx * ay, (Y - Y0) - Cy * ax


def calc_residual(X, Y, X0, Y0, ax, ay, Cx, Cy):
    """잔차 R [cm].

    X, Y    현재 무게중심 (cm)
    X0, Y0  출발 전 기준 무게중심 (cm)
    ax, ay  현재 앞뒤 / 좌우 가속도 (G), IMU 값 - 정지 구간 평균(오프셋)
    Cx, Cy  좌우 / 앞뒤 보상 계수 (cm/G), calc_comp_coef 의 결과
    """
    dxc, dyc = calc_cargo_disp(X, Y, X0, Y0, ax, ay, Cx, Cy)
    return np.sqrt(dxc ** 2 + dyc ** 2)


@dataclass
class MotionParams:
    # 기준 잡기 (rest). 10Hz 기준 rest_n=20 이 2초
    rest_n: int = 20
    rest_std_max_cm: float = 0.2    # 이보다 무게중심이 흔들리면 아직 정지가 아니다 (깨끗한 정지 런 σ <= 0.09, 손 닿은 런 0.3~1.4)
    rest_a_std_max: float = 0.02    # G, 가속도도 마찬가지 (정지 런 ax 표준편차 < 0.01)
    # 보상 계수
    calib_n: int = 100              # 계수 표본 창 (10Hz 에서 10초)
    a_min: float = 0.1              # G, 유효 가속 기준
    n_min: int = 5                  # 유효 가속 표본 최소 개수
    c_abs_max: float = 30.0         # cm/G, |C| 는 대략 화물 무게중심 높이다 (기울임 런 실측 Cy 10.8~13.5). 넘으면 버린다
    # 가속도 연동 임계값 R_th = r_th0 + k * |a|
    r_th0: float | None = None      # cm, None 이면 정지 노이즈 r_th0_sigma 배 (최소 r_th0_min)
    r_th0_min: float = 0.5          # 깨끗한 정지 런 R 의 95% <= 0.2 cm
    r_th0_sigma: float = 4.0
    k: float | None = None          # cm/G, None 이면 계수 표본의 잔차 95% 분위수로 추정
    k_floor: float = 5.0         # 기울임 런에서 보상 후 잔차 / |a| 는 최대 약 13 cm/G
    k_max: float = 20.0             # 추정이 이보다 크면 표본이 이상한 것 (실로그에서 69 가 나온 적 있다)
    a_win: int = 3                  # |a| 를 최근 몇 샘플 최댓값으로 볼지 (보상 오차는 가속 피크에 약간 늦는다)
    # 판정
    n_win: int = 10                 # 최근 N 샘플 (10Hz 에서 1초)
    alpha_th: float = 0.6           # 켜짐 비율
    alpha_off: float = 0.3          # 꺼짐 비율 (히스테리시스)
    grace_s: float = 0.5            # 플래그가 이 시간 안에 다시 켜지면 T_slip 을 이어 센다


@dataclass
class MotionResult:
    valid: bool                     # 이번 샘플로 갱신했는가 (입력에 None 이 있으면 False)
    state: str                      # "rest" | "learning" | "ready"
    x0: float | None
    y0: float | None
    cx: float                       # cm/G
    cy: float
    cal_x: bool                     # 그 축 계수가 한 번이라도 유효하게 구해졌는가
    cal_y: bool
    dx_cargo: float | None          # cm
    dy_cargo: float | None
    r: float | None                 # cm
    r_th: float | None              # cm
    flag_slip: bool
    t_slip_s: float                 # 플래그가 연속으로 켜진 시간 (grace 안의 끊김은 이어 센다)


class MotionCompensator:
    def __init__(self, params=None):
        self.p = params or MotionParams()
        self.reset()

    def reset(self):
        """기준부터 전부 다시. 화물을 다시 실었을 때 부른다."""
        p = self.p
        self.state = "rest"
        self._rest = deque(maxlen=p.rest_n)
        self.x0 = self.y0 = None
        self.ax0 = self.ay0 = 0.0
        self.cx = self.cy = 0.0
        self.cal_x = self.cal_y = False
        self.r_th0 = p.r_th0
        self.k = p.k if p.k is not None else p.k_floor
        self._pending = deque()
        self._calib = deque(maxlen=p.calib_n)   # (dX, dY, ax, ay)
        self._amag = deque(maxlen=p.a_win)
        self._hits = deque(maxlen=p.n_win)
        self.flag = False
        self._t_on = None
        self._t_off = None

    # ------------------------------------------------------------------
    def update(self, t_s, x, y, ax, ay):
        """샘플 하나. t_s 초, x/y cm, ax/ay G (IMU 원값, 오프셋 제거 전)."""
        if None in (t_s, x, y, ax, ay):
            return self._result(False, None)

        if self.state == "rest":
            self._rest.append((x, y, ax, ay))
            self._try_baseline()
            return self._result(True, None)

        a_x, a_y = ax - self.ax0, ay - self.ay0
        self._amag.append(math.hypot(a_x, a_y))
        a_mag = max(self._amag)

        dxc, dyc = calc_cargo_disp(x, y, self.x0, self.y0, a_x, a_y, self.cx, self.cy)
        r = math.hypot(dxc, dyc)
        r_th = self.r_th0 + self.k * a_mag

        self._collect(x - self.x0, y - self.y0, a_x, a_y)
        self._refit()

        if self.state == "ready":
            self._hits.append(r > r_th)
            self._judge(t_s)

        return self._result(True, (dxc, dyc, r, r_th), t_s)

    # ------------------------------------------------------------------
    def _try_baseline(self):
        p = self.p
        if len(self._rest) < p.rest_n:
            return
        a = np.array(self._rest)
        if a[:, 0].std() > p.rest_std_max_cm or a[:, 1].std() > p.rest_std_max_cm:
            return
        if a[:, 2].std() > p.rest_a_std_max or a[:, 3].std() > p.rest_a_std_max:
            return
        self.x0, self.y0 = float(a[:, 0].mean()), float(a[:, 1].mean())
        self.ax0, self.ay0 = float(a[:, 2].mean()), float(a[:, 3].mean())
        if self.r_th0 is None:
            sigma = math.hypot(a[:, 0].std(), a[:, 1].std())
            self.r_th0 = max(p.r_th0_min, p.r_th0_sigma * sigma)
        self.state = "learning"

    def _collect(self, dX, dY, a_x, a_y):
        """슬립 중이 아닐 때만, n_win 샘플 지연시켜 계수 표본에 넣는다."""
        if self.flag:
            self._pending.clear()
            return
        self._pending.append((dX, dY, a_x, a_y))
        if len(self._pending) > self.p.n_win:
            self._calib.append(self._pending.popleft())

    def _refit(self):
        p = self.p
        if len(self._calib) < p.n_min:
            return
        dX, dY, a_x, a_y = np.array(self._calib).T
        nan = float("nan")
        cx, cy = calc_comp_coef(dX, dY, a_x, a_y, p.a_min, p.n_min, prev=(nan, nan))
        updated = False
        if not math.isnan(cx) and abs(cx) <= p.c_abs_max:
            self.cx, self.cal_x, updated = float(cx), True, True
        if not math.isnan(cy) and abs(cy) <= p.c_abs_max:
            self.cy, self.cal_y, updated = float(cy), True, True
        if not updated:
            return
        if self.state == "learning":
            self.state = "ready"
        if p.k is None:
            self.k = self._estimate_k(dX, dY, a_x, a_y)

    def _estimate_k(self, dX, dY, a_x, a_y):
        """계수 표본에서 (R - r_th0) / |a| 의 95% 분위수. 유효 가속 표본이 없으면 k_floor."""
        p = self.p
        a_mag = np.hypot(a_x, a_y)
        sel = a_mag > p.a_min
        if not sel.any():
            return p.k_floor
        dxc = dX - self.cx * a_y
        dyc = dY - self.cy * a_x
        over = (np.hypot(dxc, dyc) - self.r_th0)[sel] / a_mag[sel]
        return min(p.k_max, max(p.k_floor, float(np.quantile(over, 0.95))))

    def _judge(self, t_s):
        p = self.p
        ratio = sum(self._hits) / p.n_win if len(self._hits) == p.n_win else 0.0
        if not self.flag and ratio >= p.alpha_th:
            self.flag = True
            if self._t_on is None or self._t_off is None or t_s - self._t_off > p.grace_s:
                self._t_on = t_s
            self._t_off = None
        elif self.flag and ratio < p.alpha_off:
            self.flag = False
            self._t_off = t_s

    def _t_slip(self, t_s):
        if self._t_on is None:
            return 0.0
        if self.flag:
            return t_s - self._t_on
        if self._t_off is not None and t_s - self._t_off <= self.p.grace_s:
            return t_s - self._t_on
        self._t_on = self._t_off = None
        return 0.0

    def _result(self, valid, disp, t_s=None):
        dxc, dyc, r, r_th = disp if disp else (None, None, None, None)
        return MotionResult(
            valid=valid, state=self.state, x0=self.x0, y0=self.y0,
            cx=self.cx, cy=self.cy, cal_x=self.cal_x, cal_y=self.cal_y,
            dx_cargo=dxc, dy_cargo=dyc, r=r, r_th=r_th, flag_slip=self.flag,
            t_slip_s=self._t_slip(t_s) if t_s is not None else 0.0,
        )


def sample_from(frame, cog, imu_label):
    """Frame 과 analysis.cog.CoG 에서 update() 인자 (t_s, x_cm, y_cm, ax, ay) 를 뽑는다.
    무게중심이 없거나 단위가 % 이거나(cm 로 못 바꿈) IMU 값이 없으면 해당 값이 None."""
    imu = frame.imu.get(imu_label)
    if cog is None or cog.unit != "mm":
        x = y = None
    else:
        x, y = cog.x / 10.0, cog.y / 10.0
    return (frame.t_s, x, y,
            imu.ax if imu else None, imu.ay if imu else None)
