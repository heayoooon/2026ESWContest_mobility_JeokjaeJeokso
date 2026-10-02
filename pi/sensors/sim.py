"""PC(VS Code)에서 파이 없이 파이프라인을 돌려보기 위한 가짜 센서.

    python pipeline.py --sim      (pi/ 에서)

HX711Bus / MPU6050 과 같은 메서드를 흉내 낸다. 값은 아래 시나리오로 만든다.

    40초 주기   0~6s 정차   6~10s 가속   12~16s 좌회전
                16~19s 제동   24~29s 우회전   34~38s 감속 정차
    18초에 한 번  제동하다 화물이 앞으로 밀리고(무게중심 이동) T1 벨트가 느슨해진다

실제 셀이 아니므로 보정값(offset/scale)도 여기서 정한 가짜 값을 쓴다 (calib.json 무시).
"""
import math
import random
import time

from sensors.hx711_bus import HX711Timeout, Sample
from sensors.mpu6050 import ACC_SCALE, GYR_SCALE, TEMP_OFFSET, TEMP_SCALE

CYCLE = 40.0            # 주행 시나리오 주기(초)
SLIP_AT = 18.0          # 화물이 밀리는 시각(초). 한 번만
CARGO_G = 3000.0        # 화물 무게(g)
G_TO_N = 9.80665 / 1000
NOISE_COUNTS = 60       # HX711 노이즈 (카운트, 표준편차)
SCALE = {"load": 0.02, "tension": 0.0047}   # g/count. 바닥 셀 / 인장형 10kg 셀 규모


def _window(c, a, b, amp, edge=0.8):
    """c 가 [a, b] 구간이면 amp. 양 끝 edge 초는 코사인으로 부드럽게 오르내린다."""
    if c <= a or c >= b:
        return 0.0
    k = min(1.0, (c - a) / edge, (b - c) / edge)
    return amp * (0.5 - 0.5 * math.cos(math.pi * k))


class Scenario:
    """시각 -> 차량/화물 상태. 모든 가짜 센서가 이걸 같이 본다."""

    def __init__(self):
        self.t0 = time.monotonic()

    def at(self):
        t = time.monotonic() - self.t0
        c = t % CYCLE
        ax = _window(c, 6, 10, 0.25) + _window(c, 16, 19, -0.35) + _window(c, 34, 38, -0.2)
        ay = _window(c, 12, 16, 0.2) + _window(c, 24, 29, -0.25)
        slipped = t >= SLIP_AT
        tight = min(1.0, t / 3.0)                       # 처음 3초 동안 벨트를 조인다
        return {
            "moving": 6 <= c < 38,
            "ax": ax,                                   # 종가속도(G). + = 가속
            "ay": ay,                                   # 횡가속도(G). + = 좌회전
            "yaw": _window(c, 12, 16, 15) + _window(c, 24, 29, -18),    # deg/s
            # 무게중심 (0~1). x 왼->오, y 앞->뒤. 가속하면 뒤로, 좌회전하면 오른쪽으로 쏠린다
            "x": 0.52 + 0.16 * ay + (0.05 if slipped else 0.0),
            "y": 0.47 + 0.12 * ax - (0.12 if slipped else 0.0),
            # 벨트 장력(N). 관성으로 화물이 벨트를 당기면 오른다
            "t1": 55 * tight - (28 if slipped else 0) + 25 * abs(ax),
            "t2": 60 * tight + 25 * abs(ay),
        }


class SimHX711Bus:
    def __init__(self, cfg, scenario):
        self.labels = list(cfg.hx_labels)
        self.n = len(self.labels)
        _, self.tension, self.corners = cfg.roles(self.labels)
        self.period = 1.0 / max(1, cfg.hx_rate)
        self.scn = scenario
        self.rng = random.Random(1)
        self.seq = 0
        self._next = time.monotonic() + self.period
        # 채널마다 제각각인 영점. 라벨로 시드를 잡아 실행할 때마다 같다
        self.offset = {l: random.Random(l).randint(-300000, 300000) for l in self.labels}
        self.scale = {l: SCALE["tension" if l in self.tension else "load"] for l in self.labels}

    def calib(self):
        """calib.json 대신 쓰는 가짜 보정값 (load_calib() 와 같은 모양)."""
        return {"offset": dict(self.offset), "scale_g_per_count": dict(self.scale)}

    def describe(self):
        return "HX711 시뮬레이션  %d채널 [%s]  %.0fHz" % (
            self.n, ", ".join(self.labels), 1 / self.period)

    def sync_reset(self, settle=0.5):
        time.sleep(min(settle, 0.1))
        self._next = time.monotonic() + self.period

    def missing(self, timeout=0.6):
        return []

    def ready(self):
        return time.monotonic() >= self._next

    def read(self, timeout=1.0):
        wait = self._next - time.monotonic()
        if wait > timeout:
            time.sleep(timeout)
            raise HX711Timeout(self.labels)
        if wait > 0:
            time.sleep(wait)
        t_ns = time.monotonic_ns()
        self._next = max(self._next + self.period, time.monotonic())
        g = self._grams(self.scn.at())
        vals = [int(round(self.offset[l] + g.get(l, 0.0) / self.scale[l]
                          + self.rng.gauss(0, NOISE_COUNTS)))
                for l in self.labels]
        self.seq += 1
        return Sample(t_ns=t_ns, seq=self.seq, values=vals, max_high_ns=7000, flags=[])

    def _grams(self, st):
        g = {}
        if self.corners:                                # 네 모서리에 하중을 나눈다
            x, y = st["x"], st["y"]
            fl, fr, rl, rr = self.corners
            g[fl] = CARGO_G * (1 - x) * (1 - y)
            g[fr] = CARGO_G * x * (1 - y)
            g[rl] = CARGO_G * (1 - x) * y
            g[rr] = CARGO_G * x * y
        for i, l in enumerate(self.tension):
            g[l] = (st["t1"] if i == 0 else st["t2"]) / G_TO_N
        return g

    def close(self):
        pass


class SimMPU6050:
    def __init__(self, label, scenario, index=0):
        self.label = label
        self.actual_hz = 100.0
        self.scn = scenario
        self.rng = random.Random(label)
        self.shake = 1.0 + 0.5 * index                  # 두 번째 센서(화물 쪽)가 더 흔들린다

    def check(self):
        return 0x68

    def read_raw(self):
        t_ns = time.monotonic_ns()
        st = self.scn.at()
        n = self.rng.gauss
        sd = (0.03 if st["moving"] else 0.003) * self.shake
        ax = st["ax"] + n(0, sd)
        ay = st["ay"] + n(0, sd)
        az = 1.0 + n(0, sd * 1.5)
        gx, gy = n(0, 0.4 * self.shake), n(0, 0.4 * self.shake)
        gz = st["yaw"] + n(0, 0.4)

        def lsb(v, scale):
            return max(-32768, min(32767, int(round(v * scale))))

        temp = int(round((25.0 - TEMP_OFFSET) * TEMP_SCALE))
        return (t_ns, lsb(ax, ACC_SCALE), lsb(ay, ACC_SCALE), lsb(az, ACC_SCALE),
                lsb(gx, GYR_SCALE), lsb(gy, GYR_SCALE), lsb(gz, GYR_SCALE), temp)

    def close(self):
        pass
