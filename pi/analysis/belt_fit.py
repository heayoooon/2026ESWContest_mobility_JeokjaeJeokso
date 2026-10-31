"""벨트 누름 계수 k 와 누름 자리 (x, y) 를 정차 중 벨트를 조이거나 풀 때의 셀 변화로 추정한다.

장력 T 인 벨트는 화물을 k T 로 눌러 그 자리 (x, y) 에서 바닥 셀에 더해진다 (total.py, cog.py).
화물이 그대로인 채 장력만 바뀌면 셀 합과 모멘트가 장력 변화에 비례해 바뀐다. 벨트마다 c = k / g (1N 당 g) 로
    Δ셀 합 = Σ c_i ΔT_i        ΔX 모멘트 = Σ c_i x_i ΔT_i        ΔY 모멘트 = Σ c_i y_i ΔT_i
X 모멘트 = (앞오 + 뒤오) x L, Y 모멘트 = (뒤왼 + 뒤오) x W (cog.py 의 식, g·mm).
조임 · 풀림 한 번이 식 한 줄이다. 줄을 모아 최소제곱으로 c_i, c_i x_i, c_i y_i 를 풀고 x, y 는 c 로 나눠 낸다.
시작값(BELT_K, BELT_N_Y / BELT_S_Y)은 PRIOR_N 만큼 조인 한 번으로 쳐서, 조임이 쌓일수록 데이터 쪽으로 간다.

ΔT 는 '조이기 전 멈춘 구간' 과 '조인 뒤 멈춘 구간' 의 평균 차다. 언제 다 조였는지 알려줄 필요 없다.
장력 · 셀 합 · IMU 가 STILL_S 초 넘게 구간 평균에서 안 벗어나면 멈춘 구간으로 보고, 그 구간이 길어지는 동안
매 프레임 다시 풀어 낸다 (길수록 평균이 정확해진다). 다음 변화가 오면 그 구간을 확정하고 다음 조임의 '조이기 전' 값으로 쓴다.
그래서 앞 벨트를 조이고 0.8초 뒤 뒷벨트를 건드려도 그 0.8초로 앞 벨트를 내고 바로 뒷벨트로 넘어간다.
두 벨트가 같이 바뀐 줄도 최소제곱이 같이 푼다 (한쪽을 먼저 알면 그만큼 빼고 푸는 것과 같다).

적재 · 주행 모드 (analyzer.set_mode): 적재 모드에서만 추정한다. 주행으로 바꾸는 순간 freeze() 가 지금 멈춘 구간을
'다음 변화' 를 기다리지 않고 확정하고, 주행하는 동안 그 k, x, y 를 그대로 쓴다. 적재로 돌아오면 thaw() 가 다시 추정을 켜고
주행 전 기준 구간은 버린다 (달리는 동안 화물 · 장력이 바뀌었을 수 있다). 다음 CARGO_S 넘게 가만히 있는 구간이 새 기준이다.

2026-10-25 장력꽉22 · 33 (정차, 벨트를 하나씩 조임): 멈춘 구간 1초 · 0.5초 · 0.3초 모두
앞 k 2.0 ~ 2.1 (200 ~ 219g/N) y -56 ~ -60mm, 뒤 k 1.8 (176 ~ 188g/N) y 242 ~ 256mm 로 같게 나왔다.
주행 기록으로는 안 맞는다. 장력이 1 ~ 3N 밖에 안 흔들리고, 그 흔들림도 화물 관성이 셀과 벨트를 같이 당겨서 생긴다.
그래서 IMU 가 흔들리는 동안은 멈춘 구간으로 안 본다.

k 와 (x, y) 의 뜻 (아직 코드에 반영 안 한 한계)
  - 화물이 한쪽으로 치우치면 벨트 양쪽 다리 각도가 달라진다 (한쪽 α₁ 이 낮아지면 반대쪽 α₂ 는 높아진다).
    누르는 힘은 2 T sinα 가 아니라 T (sinα₁ + sinα₂) = 2 T sin(평균 α) cos(Δα/2) 다.
    k 는 이 값을 데이터로 직접 재므로 비대칭이 이미 들어 있고, 각도 차 20° 여도 cos(10°) 로 1.5% 차이뿐이다.
  - 더 큰 영향은 수평 힘이다. 양쪽 다리의 수평 성분 T (cosα₁ − cosα₂) 가 화물 높이에서 화물을 옆으로 밀어
    바닥 셀에 모멘트를 만든다 (예: 70° / 90°, 30N, 높이 150mm, 14kg 이면 무게중심이 약 11mm 옮겨 보인다).
    그래서 추정한 x, y 는 벨트가 실제 닿는 자리가 아니라 이 모멘트까지 합친 '등가 누름 자리' 다.
    10-25 에서 x 가 화물 가운데(125mm)가 아닌 50 ~ 100mm, 앞 y 가 실측 30mm 가 아닌 -50mm 로 나온 것도 이 때문으로 본다.
  - 주행 중 화물이 옆으로 밀리면 각도가 바뀌어 k, x, y 도 바뀌지만, 주행 중에는 다시 배우지 않는다 (위의 '장력이 준 줄').
  - 장력 하한 (tension.py, EN 12195-1 식 10) 은 여전히 2 sin(TENSION_ANGLE) 를 쓴다. 추정한 k 로 바꿀 수 있지만
    10-25 실측 k 1.8 ~ 2.1 이면 차이가 5% 안이라 아직 안 바꿨다.

버리는 경우
  - 장력이 준 줄. 조일 때만 배운다. 주행 중 화물이 밀려 벨트가 풀린 것과 구분이 안 된다
    (sim 18초: 급제동에 화물이 94mm 밀리며 앞 장력이 8N 줄어 앞 y 를 524mm 로 잘못 배웠다)
  - 조이기 전 · 뒤 멈춘 구간의 IMU 가속이 STILL_A 넘게 다른 줄 (한쪽은 달리거나 도는 중)
  - 셀 합 변화가 지금 k 로 예상한 값에서 GATE 비율 + GATE_G 넘게 벗어난 줄 (조이며 손으로 누름, 같이 짐을 실음)
  - 장력 변화가 STEP_N 보다 작은 줄 (변화가 작아 잡음뿐)
  - 장력은 그대로인데 무게만 달라진 멈춘 구간이 CARGO_S 보다 짧으면 기준으로도 안 쓴다 (조이다 손으로 화물을 누름)
  - 벨트를 RELEASE_N 밑으로 풀면 그 벨트가 들어간 줄 전부. 다시 매면 누르는 자리가 바뀐다
  - 풀어 낸 k 가 K_RANGE 밖이거나 y 가 시작값에서 셀 간격 W 의 Y_RANGE 넘게 멀면 그 벨트는 이전 값을 그대로 쓴다
BELT_FIT=0 이면 추정하지 않고 시작값만 쓴다.
"""
from dataclasses import dataclass

G = 9.80665             # m/s^2 (acquisition/config.py 와 같은 값)

# --- 바꿔도 되는 기준값 ------------------------------------------------------
STILL_N = 0.5           # N. 장력이 멈춘 구간 평균에서 이만큼 안이면 멈춤 (10-25 멈춘 구간 안 흔들림 0.1 ~ 0.6N)
STILL_G = 150           # g. 셀 합이 같은 방식으로 (10-25 멈춘 구간 안 흔들림 15 ~ 356g, 대부분 100g 안)
STILL_A = 0.03          # G. IMU 가속이 구간 첫 값에서 이만큼 안이면 정차
STILL_S = 0.3           # 초. 이만큼 이어져야 멈춘 구간 (10Hz 에서 4프레임). 10-25 기록에서 0.3초도 1초와 같은 값
CARGO_S = 2.0           # 초. 장력 변화 없이 무게만 달라진 멈춘 구간은 이만큼 이어져야 기준으로 (짐 싣기 · 내리기)
STEP_N = 8.0            # N. 멈춘 구간 사이 장력 변화가 이보다 작으면 추정에 안 쓴다. 10-25 조임은 한 번에 11 ~ 38N,
                        # 화물빼기는 짐을 끌어낼 때 장력이 3 ~ 4N 같이 흔들려 3N 이면 뒤 y 를 140 · 375mm 로 잘못 배웠다
PRIOR_N = 5.0           # N. 시작값을 이만큼 조인 한 번으로 친다 (15N 조임 한 번이면 데이터가 9배 무겁다)
GATE = 0.35             # 셀 합 변화가 예상에서 이 비율 + GATE_G 넘게 벗어나면 버린다
GATE_G = 150            # g
RELEASE_N = 2.0         # N. 벨트가 이보다 느슨하면 푼 것으로 보고 그 벨트 줄을 버린다
K_RANGE = (1.2, 2.6)    # 그럴듯한 k (이론 2 sinα 는 최대 2, 10-25 실측 1.7 ~ 2.2)
Y_RANGE = 0.3           # 누름 y 가 시작값에서 셀 간격 W 의 이 비율(392mm 면 118mm) 안이어야 받는다 (10-25 실측 17 ~ 36mm 차이)


@dataclass
class Belt:
    k: float            # 장력 1N 이 화물을 누르는 힘(N). 시작값 BELT_K
    x: float | None     # 누름 자리 X (무게중심과 같은 단위). None = 화물 무게중심 X 를 누른다고 보고 X 를 안 옮긴다
    y: float | None     # 누름 자리 Y. 위치를 모르는 벨트(세 번째부터)는 None
    steps: int = 0      # 추정에 쓴 조임 · 풀림 횟수. 0 이면 시작값 그대로

    @property
    def g_per_n(self):
        """장력 1N 이 바닥 셀 합에 더하는 무게(g)."""
        return self.k / G * 1000


class _Still:
    """멈춘 구간. 값마다 구간 평균에서 허용 안에 머무는 동안 이어진다."""

    def __init__(self, t, v, a):
        self.t0 = self.t1 = t
        self.n = 1
        self.sum = list(v)
        self.a = a                              # 구간 첫 IMU 가속 (ax, ay, az). IMU 가 없으면 None

    def mean(self):
        return [s / self.n for s in self.sum]

    def fits(self, v, a, tol):
        if any(abs(x - m) > e for x, m, e in zip(v, self.mean(), tol)):
            return False
        return a is None or self.a is None or all(abs(p - q) <= STILL_A for p, q in zip(a, self.a))

    def add(self, t, v):
        self.t1 = t
        self.n += 1
        self.sum = [s + x for s, x in zip(self.sum, v)]


def _solve(a, b):
    """a x = b (작은 정방 행렬, 부분 피벗 가우스 소거). 풀 수 없으면 None."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        if abs(m[p][c]) < 1e-12:
            return None
        m[c], m[p] = m[p], m[c]
        for r in range(n):
            if r != c:
                f = m[r][c] / m[c][c]
                m[r] = [x - f * y for x, y in zip(m[r], m[c])]
    return [m[i][n] / m[i][i] for i in range(n)]


class BeltFit:
    def __init__(self, cfg, bed):
        _, _, self.corners = cfg.roles()
        self.length, self.width = bed or (100.0, 100.0)
        sides = cfg.belt_sides()
        # 위치를 아는 벨트(N · S)만 추정한다. 나머지는 시작 k 로 화물 무게중심을 누른다고 본다
        self.labels = [label for label, (_, y) in sides.items() if y is not None]
        self.learn = bool(cfg.belt_fit and self.corners and self.labels)
        self.start = {label: Belt(cfg.belt_k, None, y) for label, (_, y) in sides.items()}
        c0 = cfg.belt_k / G * 1000
        # 시작값 [c, c x, c y]. x 는 시작값이 없어 판 가운데로 두지만 그 벨트를 조이기 전에는 안 쓴다 (x = None)
        self.prior = [(c0, c0 * self.length / 2, c0 * sides[label][1]) for label in self.labels]
        # 장력 · 셀 합 멈춤 허용. 모멘트는 셀 합으로 같이 보므로 따로 안 본다
        self.tol = [STILL_G, float("inf"), float("inf")] + [STILL_N] * len(self.labels)
        self.rows = []                          # 확정한 줄 [(ΔT 목록, Δ셀 합, ΔX 모멘트, ΔY 모멘트)]
        self.ref = None                         # 마지막으로 확정한 멈춘 구간의 평균 [셀 합, X 모멘트, Y 모멘트, T...]
        self.ref_a = None                       # 그 구간의 IMU 가속 (IMU 가 없으면 None)
        self.still = None                       # 지금 이어지는 구간
        self.fixed = dict(self.start)           # 확정한 줄로만 푼 값 (새 줄을 거를 때 쓴다)
        self.belts = dict(self.start)           # 지금 쓰는 값 = 확정한 줄 + 지금 멈춘 구간
        self.frozen = False                     # 주행 모드: 추정을 멈추고 확정한 값을 그대로 쓴다

    def freeze(self):
        """주행 시작: 지금 멈춘 구간을 확정하고 추정을 멈춘다. 마지막 조임은 원래 다음 변화가 와야 확정되는데,
        주행 버튼이 그 '다음 변화' 역할을 한다. STILL_S 도 안 멈춘 구간(조이는 중에 누름)은 버린다."""
        if self.learn and not self.frozen:
            self._close()
            self.belts = dict(self.fixed)       # 확정한 줄로 푼 값만 쓴다. 줄로 못 받은 구간의 임시값은 버린다
        self.frozen = True

    def thaw(self):
        """적재로 돌아옴: 다시 추정한다. 주행 전 기준 구간은 버리고 CARGO_S 넘게 멈춘 구간을 새 기준으로 잡는다.
        확정한 줄과 지금 값은 그대로 둔다 (주행 중 푼 벨트의 줄은 새 기준을 잡을 때 버린다)."""
        self.frozen = False
        self.still = self.ref = self.ref_a = None

    def __call__(self, t, frame, imu):
        """-> {장력 라벨: Belt}. 장력 셀 전부 (위치를 모르는 벨트는 시작값 그대로)."""
        if not self.learn or self.frozen:
            return self.belts
        g = [frame.load_g.get(c) for c in self.corners]
        tension = [frame.tension_n.get(label) for label in self.labels]
        if None in g or None in tension:
            self._close()
            return self.belts
        fl, fr, rl, rr = g
        v = [fl + fr + rl + rr, (fr + rr) * self.length, (rl + rr) * self.width] + [max(0.0, n) for n in tension]
        a = (imu.ax, imu.ay, imu.az) if imu else None

        if self.still and self.still.fits(v, a, self.tol):
            self.still.add(t, v)
        else:
            self._close()
            self.still = _Still(t, v, a)
        # 멈춘 구간이 길어지는 동안 매 프레임 다시 푼다. 확정은 다음 변화가 올 때 (_close)
        if self.ref is not None and self.still.t1 - self.still.t0 >= STILL_S:
            row = self._row(self.still.mean(), self.still.a)
            if row:
                self.belts = self._fit(self.rows + [row], self.belts)
        return self.belts

    def _close(self):
        """지금 구간이 끝났다. STILL_S 넘게 멈춰 있었으면 줄로 확정하고 다음 비교의 기준으로 삼는다.
        장력은 그대로인데 무게만 달라진 구간은 CARGO_S 넘게 이어져야(짐을 싣거나 내림) 기준으로 받는다.
        조이다 손으로 화물을 잠깐 누른 구간이 '조이기 전' 값이 되면 k 가 틀린다
        (10-25 장력꽉22 2.5 ~ 2.9초: 셀 합 +480g 이 기준이 되어 앞 k 가 2.1 대신 1.8)."""
        s, self.still = self.still, None
        if s is None or s.t1 - s.t0 < STILL_S:
            return
        mean, settled = s.mean(), s.t1 - s.t0 >= CARGO_S
        if self.ref is None:
            if settled:
                self._drop_released(mean)
                self.ref, self.ref_a = mean, s.a
                self.fixed = self._fit(self.rows, self.fixed)
                self.belts = dict(self.fixed)
            return
        dts = [x - r for x, r in zip(mean[3:], self.ref[3:])]
        dt = max(abs(x) for x in dts)
        if min(dts) <= -STEP_N:
            pass                                # 풀렸다. 배우지 않고 기준만 옮긴다
        elif dt >= STEP_N:
            row = self._row(mean, s.a)
            if row:
                self.rows.append(row)
            elif not settled:
                return
        elif not settled and (dt > STILL_N or abs(mean[0] - self.ref[0]) > GATE_G):
            return
        self._drop_released(mean)
        self.ref, self.ref_a = mean, s.a
        self.fixed = self._fit(self.rows, self.fixed)
        self.belts = dict(self.fixed)

    def _drop_released(self, mean):
        """푼 벨트는 다시 맬 때 자리가 바뀐다. 그 벨트가 들어간 줄을 버린다."""
        for i, n in enumerate(mean[3:]):
            if n < RELEASE_N:
                self.rows = [r for r in self.rows if abs(r[0][i]) < STEP_N]

    def _row(self, mean, a):
        """기준 구간 -> mean 구간 (IMU 가속 a) 한 줄. 조인 게 아니거나(장력 변화가 작음 · 준 벨트가 있음),
        IMU 가 기준 구간과 다르거나, 셀 합 변화가 예상과 너무 다르면 None."""
        d = [x - r for x, r in zip(mean, self.ref)]
        dt = d[3:]
        if max(dt) < STEP_N or min(dt) <= -STEP_N:
            return None
        if a and self.ref_a and any(abs(p - q) > STILL_A for p, q in zip(a, self.ref_a)):
            return None
        cs = [self.fixed[label].g_per_n for label in self.labels]
        expect = sum(c * x for c, x in zip(cs, dt))
        scale = sum(c * abs(x) for c, x in zip(cs, dt))
        if abs(d[0] - expect) > GATE * scale + GATE_G:
            return None
        return dt, d[0], d[1], d[2]

    def _fit(self, rows, last):
        """줄 + 시작값으로 최소제곱. last = 풀이가 이상할 때 그대로 둘 값."""
        n = len(self.labels)
        w = PRIOR_N ** 2
        a = [[w if i == j else 0.0 for j in range(n)] for i in range(n)]
        bs = [[w * self.prior[i][q] for i in range(n)] for q in range(3)]
        for dt, *dq in rows:
            for i in range(n):
                for j in range(n):
                    a[i][j] += dt[i] * dt[j]
                for q in range(3):
                    bs[q][i] += dt[i] * dq[q]
        sol = [_solve(a, b) for b in bs]
        out = dict(self.start)
        for i, label in enumerate(self.labels):
            steps = sum(1 for dt, *_ in rows if abs(dt[i]) >= STEP_N)
            if not steps:
                continue
            c = sol[0][i] if None not in sol else 0.0
            k = c * G / 1000
            if not K_RANGE[0] <= k <= K_RANGE[1] or abs(sol[2][i] / c - self.start[label].y) > Y_RANGE * self.width:
                out[label] = last[label]
                continue
            out[label] = Belt(k, sol[1][i] / c, sol[2][i] / c, steps)
        return out
