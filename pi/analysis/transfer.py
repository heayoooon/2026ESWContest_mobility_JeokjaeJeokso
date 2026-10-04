"""주행 중 하중 이동: 무게중심 높이 h 를 추정하고, 무게중심 변화가 가감속으로 설명되는지 본다.

가속도 a(G)로 달리면 화물(무게중심 높이 h)에 걸린 관성력 때문에 바닥 셀이 보는 무게중심이 h x a 만큼
옮겨진다. 앞으로 가속(ax+)하면 뒤로(Y+), 왼쪽으로 돌면(ay+) 오른쪽으로(X+). 비탈에 서 있어도 같다.
    X = X0 + kx (ay - ay0)        Y = Y0 + ky (ax - ax0)        IMU 는 COG_LAG 초 전 값
k 는 무게중심 단위 / G 이고, mm 단위면 k 가 곧 h(mm)다 (판이 휘는 만큼 축마다 조금 다르게 나온다).
X0, Y0 = 가감속이 없을 때의 무게중심, ax0, ay0 = 그때의 IMU 값.

h 추정: 가감속이 A_FIT 이상인 프레임마다 (a - a0, 무게중심 - X0) 점을 모아 원점을 지나는 직선의 기울기를
최소제곱으로 낸다 (2026-10-03 급출발 기록에서 k 를 맞춘 방법과 같다). 오래된 점은 FORGET 로 잊고,
COG_H 시작값은 PRIOR_W 만큼의 점으로 친다. 잔차가 위험 수준인 점(화물 이동)은 맞춤에 안 쓴다.
기울기 부호도 데이터로 정해지므로 IMU 가 반대로 달려 있어도 맞춰진다.

변화 적절성: 잔차 = 실측 - 예측. 셀 간격의 COG_RESID % 를 넘으면 주의, 2배를 넘으면 위험.
가감속으로 설명 안 되는 무게중심 변화 = 화물이 미끄러지거나 쓰러지거나 떨어진 것.
기준(X0, Y0)은 조용할 때(가감속 · 회전 없음) 천천히 따라가고, 화물이 새 자리에서 조용히 SETTLE_S 초
머물면 그 자리를 새 기준으로 받아들인다. 알림은 analyzer 의 Hold 가 몇 초 더 붙잡아 둔다.
"""
import collections
import math
from dataclasses import dataclass

from analysis.risk import CAUTION, DANGER, OK, Check

A_FIT = 0.08        # G. 기준보다 이만큼 넘게 가감속할 때만 h 를 고친다
A_QUIET = 0.04      # G. 기준 IMU 값과 이만큼 안이면 가감속 없음
GZ_QUIET = 5.0      # °/s. 이보다 느리게 돌면 회전 없음
FORGET = 0.995      # h 맞춤 점 하나가 들어올 때마다 옛 점의 무게를 이만큼 줄인다 (약 200점 기억)
PRIOR_W = 0.05      # G^2. COG_H 시작값을 이만큼의 점으로 친다 (0.2G 프레임 1~2개)
REF_TAU = 2.0       # 초. 조용할 때 기준 무게중심 · 기준 IMU 값이 따라가는 시정수
SETTLE_S = 3.0      # 초. 이동한 화물이 새 자리에서 이만큼 조용히 있으면 새 기준으로
RESET_S = 2.0       # 초. 무게중심이 이만큼 안 나오면(화물 내림) 기준을 버리고 다음 화물에서 새로 잡는다
K_MAX = 2.0         # |k| 상한 = 셀 간격 x K_MAX (/G). 이상한 점 몇 개에 h 가 튀지 않게


@dataclass
class Transfer:
    pred_x: float       # IMU 로 예측한 무게중심 (CoG 와 같은 단위)
    pred_y: float
    kx: float | None    # 하중 이동 계수, 좌우 (CoG 단위 / G). mm 단위면 곧 무게중심 높이 h(mm)
    ky: float | None    # 앞뒤. 시작값도 데이터도 없으면 None


class _Axis:
    """무게중심 한 축과 그걸 움직이는 가속도 한 축 (X <- ay, Y <- ax)."""

    def __init__(self, span, k0, resid_pct):
        self.span = span                        # 셀 간격 (CoG 단위)
        self.thr = span * resid_pct / 100       # 잔차 허용 (CoG 단위)
        self.k0 = k0
        self.sxy = self.sxx = 0.0
        self.c0 = self.a0 = None                # 기준 무게중심, 기준 가속도

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


class TransferCheck:
    def __init__(self, cfg, bed):
        span_x, span_y = bed or (100.0, 100.0)
        k0 = cfg.cog_h if bed else None         # % 단위면 mm 시작값을 못 쓴다
        self.unit = "mm" if bed else "%"
        self.lag = cfg.cog_lag
        self.x = _Axis(span_x, k0, cfg.cog_resid)
        self.y = _Axis(span_y, k0, cfg.cog_resid)
        self.hist = collections.deque()         # 최근 COG_LAG 초 IMU (t, ax, ay, |gz|)
        self.prev_t = None                      # 마지막으로 무게중심을 본 시각
        self.settle = []                        # 이동한 화물의 새 자리 후보 [(t, X0, Y0)]

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
                X.c0 = Y.c0 = None
            return None, Check("move", "화물 이동", None, "무게중심 값 없음")

        _, ax, ay, _ = self.hist[0]             # COG_LAG 초 전 IMU (로드셀이 그만큼 늦다)
        if X.a0 is None:
            X.a0, Y.a0 = ay, ax
        day, dax = ay - X.a0, ax - Y.a0
        kx, ky = X.k(), Y.k()
        sx, sy = c.x - (kx or 0) * day, c.y - (ky or 0) * dax     # 하중 이동을 뺀 무게중심
        if X.c0 is None:                        # 처음 (또는 화물을 새로 올림)
            X.c0, Y.c0 = sx, sy
        px, py = X.c0 + (kx or 0) * day, Y.c0 + (ky or 0) * dax
        rx, ry = c.x - px, c.y - py
        e = max(abs(rx) / X.thr, abs(ry) / Y.thr)       # 허용의 몇 배인가

        # h 맞춤: 가감속이 충분하고 화물 이동으로 안 보일 때만
        if e < 2:
            if abs(day) >= A_FIT:
                X.fit(day, c.x - X.c0)
            if abs(dax) >= A_FIT:
                Y.fit(dax, c.y - Y.c0)

        # 기준 따라가기: 최근 COG_LAG 초 동안 가감속 · 회전이 없을 때만
        quiet = all(abs(a_y - X.a0) < A_QUIET and abs(a_x - Y.a0) < A_QUIET and gz < GZ_QUIET
                    for _, a_x, a_y, gz in self.hist)
        dt = t - self.prev_t if self.prev_t is not None else 0.0
        self.prev_t = t
        if e < 1:
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

        out = Transfer(px, py, kx, ky)
        if (kx is None and abs(day) >= A_QUIET) or (ky is None and abs(dax) >= A_QUIET):
            return out, Check("move", "화물 이동", None, "h 추정 중")
        u = self.unit
        if e < 1:
            return out, Check("move", "화물 이동", OK, "잔차 X %+.0f Y %+.0f%s" % (rx, ry, u))
        if abs(rx) / X.thr >= abs(ry) / Y.thr:
            r, thr, side = rx, X.thr, "오른쪽" if rx > 0 else "왼쪽"
        else:
            r, thr, side = ry, Y.thr, "뒤쪽" if ry > 0 else "앞쪽"
        return out, Check("move", "화물 이동", DANGER if e >= 2 else CAUTION,
                          "가감속으로 설명 안 되는 이동 %s %.0f%s (허용 %.0f%s)" % (side, abs(r), u, thr, u))
