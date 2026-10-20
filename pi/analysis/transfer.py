"""주행 중 하중 이동: 무게중심 높이 h 를 추정하고, 무게중심 변화가 가감속으로 설명되는지 본다.

가속도 a(G)로 달리면 화물(무게중심 높이 h)에 걸린 관성력 때문에 바닥 셀이 보는 무게중심이 h x a 만큼
옮겨진다. 앞으로 가속(ax+)하면 뒤로(Y+), 왼쪽으로 돌면(ay+) 오른쪽으로(X+). 비탈에 서 있어도 같다.
    X = X0 + kx (ay - ay0)        Y = Y0 + ky (ax - ax0)        IMU 는 COG_LAG 초 전 값
k 는 무게중심 단위 / G 이고, mm 단위면 k 가 곧 h(mm)다 (판이 휘는 만큼 축마다 조금 다르게 나온다).
X0, Y0 = 가감속이 없을 때의 무게중심, ax0, ay0 = 그때의 IMU 값.

h 추정: 가감속이 A_FIT 이상인 프레임마다 (a - a0, 무게중심 - X0) 점을 모아 원점을 지나는 직선의 기울기를
최소제곱으로 낸다 (2026-10-03 급출발 기록에서 k 를 맞춘 방법과 같다). 오래된 점은 FORGET 로 잊고,
시작값(config 의 H_START_MM)은 PRIOR_W 만큼의 점으로 친다. 잔차가 경고 수준(E_WARNING 배) 이상인 점(화물 이동)은 맞춤에 안 쓴다.
기울기 부호도 데이터로 정해지므로 IMU 가 반대로 달려 있어도 맞춰진다.
COG_H_LEARN=0 이면 이 학습을 끄고 h 를 고정값(config 의 H_FIXED_MM)으로 둔다 (주행이 없는 시연용).
화물이 달라졌다고 볼 때는 학습한 h 를 버리고 시작값으로 돌아간다: 화물 이동이 위험으로 확정된 순간, 그리고 화물을
내려서 무게중심이 RESET_S 초 넘게 안 나올 때. 위험이 뜨기 직전 프레임들이 맞춤에 섞여 h 가 튀어 있어도(2026-10-04
화물추락: 220 -> 337mm) 운전자가 멈추고 다시 묶은 뒤까지 이어지지 않는다.

화물 이동 판정: 잔차 = 실측 - 예측. 가감속으로 설명 안 되는 무게중심 변화 = 화물이 미끄러지거나 쓰러지거나
떨어진 것. 세 단계로 거른다.
 1) 허용이 가속에 따라 커진다: 허용 = 셀 간격 x (COG_RESID + COG_RESID_A x |a|) %. 보상을 해도 급가감속 중에는
    h 오차가 가속에 비례해 남기 때문이다. 조용하면 COG_RESID 까지 내려가 정차 · 정속 중의 이동도 잡는다.
    |a| 는 최근 A_WIN 프레임의 최댓값(보상 오차는 가속 피크보다 조금 늦게 나온다).
 2) 프레임마다 e = 잔차 / 허용 (두 축 중 큰 쪽)으로 심각도 s 를 낸다: e < 1 정상, 1 이상 1(주의),
    E_WARNING 이상 2(경고), E_DANGER 이상 3(위험). 최근 MOVE_WIN_S 초 안에서 s 가 L 이상인 프레임이
    MOVE_ALPHA 비율 이상인 가장 높은 L 이 그 순간의 단계다. 한 프레임 튄 값은 여기서 걸러진다.
 3) 단계가 주의 이상으로 이어진 시간 T_slip 으로 의심과 확정을 나눈다. MOVE_CONFIRM_S 넘게 이어지면 최소
    경고, MOVE_DANGER_S 넘게 이어지고 최근 e 의 중앙값이 E_DANGER 이상이면 위험. 급제동 순간의 보상 오차는
    금방 꺼지지만 실제 이동은 계속된다. GRACE_S 안의 끊김은 이어서 센다.
기준(X0, Y0)은 조용할 때(가감속 · 회전 없음) 천천히 따라가고, 화물이 새 자리에서 조용히 SETTLE_S 초
머물면 그 자리를 새 기준으로 받아들인다. 알림은 analyzer 의 Hold 가 몇 초 더 붙잡아 둔다.

Transfer.static_x / static_y 는 하중 이동(k x a)을 뺀 무게중심이다. 무게중심 위치 판정(cog.py zone)에 쓰면
급제동 순간의 쏠림으로 오경보가 나지 않는다.
"""
import collections
import math
import statistics
from dataclasses import dataclass

from analysis.risk import CAUTION, DANGER, OK, WARNING, Check

A_FIT = 0.08        # G. 기준보다 이만큼 넘게 가감속할 때만 h 를 고친다
A_QUIET = 0.04      # G. 기준 IMU 값과 이만큼 안이면 가감속 없음
GZ_QUIET = 5.0      # °/s. 이보다 느리게 돌면 회전 없음
FORGET = 0.995      # h 맞춤 점 하나가 들어올 때마다 옛 점의 무게를 이만큼 줄인다 (약 200점 기억)
PRIOR_W = 0.05      # G^2. 시작값(H_START_MM)을 이만큼의 점으로 친다 (0.2G 프레임 1~2개)
REF_TAU = 2.0       # 초. 조용할 때 기준 무게중심 · 기준 IMU 값이 따라가는 시정수
SETTLE_S = 4.0      # 초. 이동한 화물이 새 자리에서 이만큼 조용히 있으면 새 기준으로
RESET_S = 2.0       # 초. 무게중심이 이만큼 안 나오면(화물 내림) 기준을 버리고 다음 화물에서 새로 잡는다
REBASE_MUTE_S = 3.0 # 초. 기준 재설정(rebase) 직후 이만큼은 화물 이동을 판정하지 않는다. 막 놓은 화물이 자리 잡는 시간.
                    # 이 동안 화물이 쓰러져도 못 잡으니, 재설정은 화물이 가만히 있을 때 누른다
K_MAX = 1.0         # |k| 상한 = 셀 간격 x K_MAX (/G). 이상한 점 몇 개에 h 가 튀지 않게
                    # (2026-10-04 화물추락: 추락 뒤 이동량이 맞춤에 섞여 h_y 가 556mm 까지 튀었다. 정상은 132~220mm)

# --- 화물 이동 판정 ---------------------------------------------------------
E_CAUTION, E_WARNING, E_DANGER = 1.0, 1.4, 1.8   # 허용의 몇 배부터 주의 / 경고 / 위험인가
A_WIN = 3           # 프레임. 허용에 쓰는 |a| 를 최근 이만큼의 최댓값으로 (10Hz 에서 0.3초)
MIN_WIN_N = 3       # 프레임. 최근 구간에 이만큼은 쌓여야 비율을 본다
GRACE_S = 0.5       # 초. 단계가 이만큼 안에 다시 주의 이상이 되면 T_slip 을 이어 센다


@dataclass
class Transfer:
    pred_x: float       # IMU 로 예측한 무게중심 (CoG 와 같은 단위)
    pred_y: float
    kx: float | None    # 하중 이동 계수, 좌우 (CoG 단위 / G). mm 단위면 곧 무게중심 높이 h(mm)
    ky: float | None    # 앞뒤. 시작값도 데이터도 없으면 None
    static_x: float | None = None   # 하중 이동(k x a)을 뺀 무게중심. k 를 모르면 그대로의 무게중심
    static_y: float | None = None
    e: float | None = None          # 잔차 / 허용 (두 축 중 큰 쪽). 1 이면 허용만큼
    t_slip: float = 0.0             # 초. 화물 이동(주의 이상)이 이어진 시간


class _Axis:
    """무게중심 한 축과 그걸 움직이는 가속도 한 축 (X <- ay, Y <- ax)."""

    def __init__(self, span, k0, resid_pct, resid_a):
        self.span = span                        # 셀 간격 (CoG 단위)
        self.resid = resid_pct                  # 정지 때 허용 (셀 간격 %)
        self.resid_a = resid_a                  # 가속 1G 당 늘어나는 허용 (셀 간격 %)
        self.thr = span * resid_pct / 100       # 정지 때 잔차 허용 (CoG 단위)
        self.k0 = k0
        self.sxy = self.sxx = 0.0
        self.c0 = self.a0 = None                # 기준 무게중심, 기준 가속도

    def thr_at(self, a_mag):
        """가속 크기 a_mag(G) 일 때 잔차 허용 (CoG 단위)."""
        return self.span * (self.resid + self.resid_a * a_mag) / 100

    def k(self):
        if self.k0 is not None:
            k = (PRIOR_W * self.k0 + self.sxy) / (PRIOR_W + self.sxx)
        elif self.sxx >= PRIOR_W:
            k = self.sxy / self.sxx
        else:
            return None
        lim = K_MAX * self.span
        return max(-lim, min(lim, k))

    def fit(self, da, dc):
        self.sxy = FORGET * self.sxy + da * dc
        self.sxx = FORGET * self.sxx + da * da

    def forget_fit(self):
        """학습한 h 를 버린다. k() 는 다시 시작값 k0 에서 출발한다."""
        self.sxy = self.sxx = 0.0


class TransferCheck:
    def __init__(self, cfg, bed):
        span_x, span_y = bed or (100.0, 100.0)
        k0 = cfg.cog_h if bed else None         # % 단위면 mm 시작값을 못 쓴다
        self.unit = "mm" if bed else "%"
        self.lag = cfg.cog_lag
        self.x = _Axis(span_x, k0, cfg.cog_resid, cfg.cog_resid_a)
        self.y = _Axis(span_y, k0, cfg.cog_resid, cfg.cog_resid_a)
        self.learn = bool(cfg.cog_h_learn)      # False 면 h 를 학습하지 않고 고정값(H_FIXED_MM)으로 둔다
        self.win_s = cfg.move_win_s             # 최근 구간 길이(초)
        self.alpha = cfg.move_alpha             # 구간 안에서 이 비율 이상이어야 그 단계
        self.confirm_s = cfg.move_confirm_s     # T_slip 이 이만큼이면 최소 경고
        self.danger_s = cfg.move_danger_s       # T_slip 이 이만큼이면 위험 (e 도 E_DANGER 이상일 때)
        self.hist = collections.deque()         # 최근 COG_LAG 초 IMU (t, ax, ay, |gz|)
        self.prev_t = None                      # 마지막으로 무게중심을 본 시각
        self.settle = []                        # 이동한 화물의 새 자리 후보 [(t, X0, Y0)]
        self.mute_pending = False               # rebase() 직후: 다음에 본 프레임부터 REBASE_MUTE_S 초 판정 보류
        self.mute_until = None                  # 이 시각까지 판정 보류
        self._reset_judge()

    def rebase(self):
        """기준 재설정: 지금 무게중심 · 가속도를 정상 기준으로 다시 잡는다 (예: 화물을 올려 놓은 뒤, 시연 직전).
        화물 이동 판정만 지운다. 기준 무게중심 · 가속도, 최근 구간, 이동 지속시간, 새 기준 후보, 위험 표시를 비우고
        학습한 h 도 시작값으로 되돌린다 (학습을 껐으면 영향 없음). 직후 REBASE_MUTE_S 초는 판정을 보류하며
        그동안 화물이 자리 잡는 대로 기준을 따라간다."""
        X, Y = self.x, self.y
        X.c0 = Y.c0 = X.a0 = Y.a0 = None
        X.forget_fit()
        Y.forget_fit()
        self.settle = []
        self.prev_t = None
        self._reset_judge()
        self.mute_pending, self.mute_until = True, None

    def _reset_judge(self):
        self.a_hist = collections.deque(maxlen=A_WIN)   # 최근 |a| (기준 IMU 값을 뺀 크기)
        self.win = collections.deque()                  # 최근 MOVE_WIN_S 초 [(t, s, e)]
        self.t_on = self.t_off = None                   # 이동이 이어진 시작 시각 / 마지막으로 꺼진 시각
        self.was_danger = False                         # 지난 프레임이 위험이었나 (위험이 새로 뜰 때만 h 를 버린다)

    def __call__(self, t, c, imu):
        """-> (Transfer 또는 None, Check)"""
        if imu is None:
            return None, Check("move", "화물 이동", None, "IMU 값 없음")
        self.hist.append((t, imu.ax, imu.ay, abs(imu.gz)))
        while len(self.hist) > 1 and self.hist[1][0] <= t - self.lag:
            self.hist.popleft()
        X, Y = self.x, self.y
        if c is None:
            if self.prev_t is not None and t - self.prev_t > RESET_S:
                X.c0 = Y.c0 = None                      # 화물을 내렸다: 기준도, 이전 화물로 학습한 h 도 버린다
                X.forget_fit()
                Y.forget_fit()
                self._reset_judge()
            return None, Check("move", "화물 이동", None, "무게중심 값 없음")

        _, ax, ay, _ = self.hist[0]             # COG_LAG 초 전 IMU (로드셀이 그만큼 늦다)
        if self.mute_pending:                   # 기준 재설정 직후: 무게중심을 처음 본 프레임부터 센다
            self.mute_pending = False
            self.mute_until = t + REBASE_MUTE_S
        if self.mute_until is not None:
            if t < self.mute_until:             # 보류 중: 화물이 자리 잡는 대로 지금 값을 기준으로 둔다
                X.a0, Y.a0 = ay, ax
                X.c0, Y.c0 = c.x, c.y
                self.prev_t = t
                out = Transfer(c.x, c.y, X.k(), Y.k(), static_x=c.x, static_y=c.y, e=0.0)
                return out, Check("move", "화물 이동", OK, "기준 재설정 중 (%.1f초 남음)" % (self.mute_until - t))
            self.mute_until = None
        if X.a0 is None:
            X.a0, Y.a0 = ay, ax
        day, dax = ay - X.a0, ax - Y.a0
        kx, ky = X.k(), Y.k()
        sx, sy = c.x - (kx or 0) * day, c.y - (ky or 0) * dax     # 하중 이동을 뺀 무게중심
        if X.c0 is None:                        # 처음 (또는 화물을 새로 올림)
            X.c0, Y.c0 = sx, sy
        px, py = X.c0 + (kx or 0) * day, Y.c0 + (ky or 0) * dax
        rx, ry = c.x - px, c.y - py
        self.a_hist.append(math.hypot(day, dax))
        a_mag = max(self.a_hist)
        thr_x, thr_y = X.thr_at(a_mag), Y.thr_at(a_mag)
        e = max(abs(rx) / thr_x, abs(ry) / thr_y)       # 가속을 반영한 허용의 몇 배인가

        # h 맞춤: 가감속이 충분하고 화물 이동으로 안 보일 때만 (COG_H_LEARN=0 이면 안 한다 -> h 는 고정값 그대로)
        if self.learn and e < E_WARNING:
            if abs(day) >= A_FIT:
                X.fit(day, c.x - X.c0)
            if abs(dax) >= A_FIT:
                Y.fit(dax, c.y - Y.c0)

        # 기준 따라가기: 최근 COG_LAG 초 동안 가감속 · 회전이 없을 때만
        quiet = all(abs(a_y - X.a0) < A_QUIET and abs(a_x - Y.a0) < A_QUIET and gz < GZ_QUIET
                    for _, a_x, a_y, gz in self.hist)
        dt = t - self.prev_t if self.prev_t is not None else 0.0
        self.prev_t = t
        if e < E_CAUTION:
            self.settle = []
            if quiet:
                a = 1 - math.exp(-dt / REF_TAU)
                X.a0 += a * day
                Y.a0 += a * dax
                X.c0 += a * (sx - X.c0)
                Y.c0 += a * (sy - Y.c0)
        elif quiet:                             # 이동한 화물이 새 자리에서 가만히 있나
            if self.settle:
                mx = sum(s[1] for s in self.settle) / len(self.settle)
                my = sum(s[2] for s in self.settle) / len(self.settle)
                if abs(sx - mx) > X.thr / 2 or abs(sy - my) > Y.thr / 2:
                    self.settle = []
            self.settle.append((t, sx, sy))
            if t - self.settle[0][0] >= SETTLE_S:
                X.c0 = sum(s[1] for s in self.settle) / len(self.settle)
                Y.c0 = sum(s[2] for s in self.settle) / len(self.settle)
                self.settle = []

        out = Transfer(px, py, kx, ky, static_x=sx, static_y=sy, e=e)
        if (kx is None and abs(day) >= A_QUIET) or (ky is None and abs(dax) >= A_QUIET):
            return out, Check("move", "화물 이동", None, "h 추정 중")

        level = self._judge(t, e)
        if level == DANGER and not self.was_danger:     # 화물이 달라졌다: 위험 직전 프레임이 섞인 h 를 버린다
            X.forget_fit()
            Y.forget_fit()
        self.was_danger = level == DANGER
        out.t_slip = self._t_slip(t)
        u = self.unit
        if level == OK:
            return out, Check("move", "화물 이동", OK, "잔차 X %+.0f Y %+.0f%s" % (rx, ry, u))
        if abs(rx) / thr_x >= abs(ry) / thr_y:
            r, thr, side = rx, thr_x, "오른쪽" if rx > 0 else "왼쪽"
        else:
            r, thr, side = ry, thr_y, "뒤쪽" if ry > 0 else "앞쪽"
        state = "확정" if out.t_slip >= self.confirm_s else "의심"
        return out, Check("move", "화물 이동", level,
                          "가감속으로 설명 안 되는 이동 %s %.0f%s (허용 %.0f%s) %s %.1f초" % (
                              side, abs(r), u, thr, u, state, out.t_slip))

    # ------------------------------------------------------------------
    def _judge(self, t, e):
        """프레임 하나의 e 를 최근 구간에 넣고 지금의 위험 단계(0 정상 ~ 3 위험)를 낸다."""
        s = 3 if e >= E_DANGER else 2 if e >= E_WARNING else 1 if e >= E_CAUTION else 0
        self.win.append((t, s, e))
        while self.win and self.win[0][0] < t - self.win_s:
            self.win.popleft()
        n = len(self.win)
        level = OK
        if n >= MIN_WIN_N:
            for lv in (DANGER, WARNING, CAUTION):
                if sum(1 for _, k, _ in self.win if k >= lv) / n >= self.alpha:
                    level = lv
                    break

        # 이동이 이어진 시간 (끊김은 GRACE_S 까지 이어서 센다)
        if level >= CAUTION:
            if self.t_on is None or (self.t_off is not None and t - self.t_off > GRACE_S):
                self.t_on = t
            self.t_off = None
        elif self.t_on is not None and self.t_off is None:
            self.t_off = t

        slip = self._t_slip(t)
        if level >= CAUTION and slip >= self.confirm_s:
            level = max(level, WARNING)
        if (level >= CAUTION and slip >= self.danger_s
                and statistics.median(e_ for _, _, e_ in self.win) >= E_DANGER):
            level = DANGER
        return level

    def _t_slip(self, t):
        """이동이 이어진 시간(초). 꺼진 지 GRACE_S 가 지나면 0."""
        if self.t_on is None:
            return 0.0
        if self.t_off is not None and t - self.t_off > GRACE_S:
            self.t_on = self.t_off = None
            return 0.0
        return t - self.t_on
