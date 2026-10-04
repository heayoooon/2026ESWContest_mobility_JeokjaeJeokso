"""PC(VS Code)에서 파이 없이 파이프라인을 돌려보기 위한 가짜 센서.

    python pipeline.py --sim      (pi/ 에서)

HX711Bus / MPU6050 과 같은 메서드를 흉내 낸다. 값은 아래 시나리오로 만든다.
SimCamera 는 D435i 트윈 노드(twin/pi_twin_node.py)의 UDP 패킷을 흉내 낸다 (numpy 필요).

    40초 주기   0~6s 정차   6~10s 가속   12~16s 좌회전
                16~19s 제동   24~29s 우회전   34~38s 감속 정차
    화물 12kg, 무게중심 높이 150mm. 가감속하면 그만큼 하중이 옮겨지고, 로드셀은 IMU 보다 0.2초 늦다
    18초에 한 번  제동하다 화물이 앞으로 밀리고(무게중심 이동) T1(N쪽) 벨트가 느슨해진다
    41~45초      정차 중에 4kg 을 더 실었다가 내린다 (최대 15kg 넘음)
    -> 판정(analysis/)의 적재 · 무게중심 · 화물 이동 · 장력 알림이 다 한 번씩 뜬다

실제 셀이 아니므로 보정값(offset/scale)도 여기서 정한 가짜 값을 쓴다 (calib.json 무시).
"""
import json
import math
import random
import struct
import time
import zlib

from sensors.hx711_bus import HX711Timeout, Sample
from sensors.mpu6050 import ACC_SCALE, GYR_SCALE, TEMP_OFFSET, TEMP_SCALE

CYCLE = 40.0            # 주행 시나리오 주기(초)
SLIP_AT = 18.0          # 화물이 밀리는 시각(초). 한 번만
EXTRA_AT = (41.0, 45.0) # 짐을 더 실었다 내리는 구간(초). 한 번만
CARGO_G = 12000.0       # 화물 무게(g)
EXTRA_G = 4000.0        # 더 싣는 짐(g)
CARGO_H = 150.0         # 화물 무게중심 높이(mm). 하중 이동 = h x 가속도(G)
BED_MM = (250.0, 392.0) # 셀 간격 (mm). pipeline.py --sim 의 BED_L/BED_W 기본값과 같다
LOAD_LAG = 0.2          # 로드셀이 IMU 보다 늦는 시간(초). 2026-10-03 실측
G_TO_N = 9.80665 / 1000
NOISE_COUNTS = 60       # HX711 노이즈 (카운트, 표준편차)
SCALE = {"load": 0.02, "tension": 0.0047}   # g/count. 바닥 셀 / 인장형 10kg 셀 규모
CAM_FPS = 15            # 가짜 카메라 프레임 속도
CAM_CELL = 0.02         # 높이맵 칸(m). pi_twin_node.py 의 CELL 과 같다
CAM_BED = (0.6, 0.4)    # 높이맵 범위(m). x 왼->오, y 앞->뒤 (시나리오 무게중심 좌표와 같은 방향)
CARGO_BOX = (0.20, 0.14, 0.15)  # 화물 상자 가로, 세로, 높이(m)


def _window(c, a, b, amp, edge=0.8):
    """c 가 [a, b] 구간이면 amp. 양 끝 edge 초는 코사인으로 부드럽게 오르내린다."""
    if c <= a or c >= b:
        return 0.0
    k = min(1.0, (c - a) / edge, (b - c) / edge)
    return amp * (0.5 - 0.5 * math.cos(math.pi * k))


class Scenario:
    """시각 -> 차량/화물 상태. 모든 가짜 센서가 이걸 같이 본다."""

    def __init__(self, t0=None):
        self.t0 = time.monotonic() if t0 is None else t0    # 다른 프로세스(SimCamera)와 같은 시각표를 쓰려면 t0 를 넘긴다

    def at(self, lag=0.0):
        t = time.monotonic() - self.t0 - lag
        c = t % CYCLE
        ax = _window(c, 6, 10, 0.25) + _window(c, 16, 19, -0.35) + _window(c, 34, 38, -0.2)
        ay = _window(c, 12, 16, 0.2) + _window(c, 24, 29, -0.25)
        slipped = t >= SLIP_AT
        tight = min(1.0, max(0.0, t) / 3.0)             # 처음 3초 동안 벨트를 조인다
        cx = 0.52 + (0.05 if slipped else 0.0)          # 화물이 실제로 놓인 자리 (0~1). 밀리면 앞오른쪽으로
        cy = 0.47 - (0.24 if slipped else 0.0)
        return {
            "moving": 6 <= c < 38,
            "ax": ax,                                   # 종가속도(G). + = 가속
            "ay": ay,                                   # 횡가속도(G). + = 좌회전
            "yaw": _window(c, 12, 16, 15) + _window(c, 24, 29, -18),    # deg/s
            "cargo_x": cx,
            "cargo_y": cy,
            # 무게중심 (0~1). x 왼->오, y 앞->뒤. 가속하면 뒤로, 좌회전하면 오른쪽으로 h x 가속도 만큼 쏠린다
            "x": cx + CARGO_H / BED_MM[0] * ay,
            "y": cy + CARGO_H / BED_MM[1] * ax,
            "load_g": CARGO_G + (EXTRA_G if EXTRA_AT[0] <= t < EXTRA_AT[1] else 0.0),
            # 벨트 장력(N). T1 = N(앞)쪽, T2 = S(뒤)쪽. 관성으로 화물이 벨트를 당기면 오른다
            "t1": 12 * tight - (8 if slipped else 0) + 10 * abs(ax),
            "t2": 11 * tight + 10 * abs(ay),
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
        g = self._grams(self.scn.at(LOAD_LAG))
        vals = [int(round(self.offset[l] + g.get(l, 0.0) / self.scale[l]
                          + self.rng.gauss(0, NOISE_COUNTS)))
                for l in self.labels]
        self.seq += 1
        return Sample(t_ns=t_ns, seq=self.seq, values=vals, max_high_ns=7000, flags=[])

    def _grams(self, st):
        g = {}
        if self.corners:                                # 네 모서리에 하중을 나눈다
            x, y, m = st["x"], st["y"], st["load_g"]
            fl, fr, rl, rr = self.corners
            g[fl] = m * (1 - x) * (1 - y)
            g[fr] = m * x * (1 - y)
            g[rl] = m * (1 - x) * y
            g[rr] = m * x * y
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


class SimCamera:
    """twin/pi_twin_node.py 와 같은 모양의 UDP 패킷을 만든다: 판 위 상자 하나를 2.5D 높이맵으로.

    상자는 화물이 실제로 밀릴 때(SLIP_AT)만 움직인다. 가감속 · 선회 중 로드셀 무게중심이 흔들리는 건
    하중 이동이라 카메라에는 안 보인다. 센서 엑셀과 카메라 엑셀을 나란히 보면 그 차이가 보인다.
    """

    def __init__(self, scenario):
        import numpy as np                              # PC 에 numpy 가 없으면 여기서 ImportError
        from twin.twin_core import summarize, world_rotation
        self.np, self.summarize = np, summarize
        acc0 = [0.0, 0.0, -9.81]                        # 기준면 1 m 위에서 똑바로 내려다보는 카메라 (z 앞 = 아래)
        self.calib = dict(ref_h=-1.0, R=np.round(world_rotation(acc0), 6).tolist(), accel0=acc0,
                          intr=dict(width=424, height=240, fx=215.174, fy=215.174, ppx=214.642, ppy=120.462),
                          depth_scale=0.001, max_range=3.0, stream="가짜 (sensors/sim.py)", filters="-",
                          camera="SimCamera (가짜)")
        self.scn = scenario
        self.rng = np.random.default_rng(2)
        self.nx = round(CAM_BED[0] / CAM_CELL)
        self.ny = round(CAM_BED[1] / CAM_CELL)
        self.gx, self.gy = np.meshgrid((np.arange(self.nx) + 0.5) * CAM_CELL,
                                       (np.arange(self.ny) + 0.5) * CAM_CELL)
        self.seq = 0

    def packet(self):
        np, st = self.np, self.scn.at()
        bw, bd, bh = CARGO_BOX
        cx, cy = st["cargo_x"] * CAM_BED[0], st["cargo_y"] * CAM_BED[1]
        box = (abs(self.gx - cx) <= bw / 2) & (abs(self.gy - cy) <= bd / 2)
        sd = 0.004 if st["moving"] else 0.002
        h = np.where(box, bh, 0.0) + self.rng.normal(0, sd, box.shape)
        h[self.rng.random(box.shape) < 0.03] = np.nan   # depth 가 안 잡힌 칸
        body = zlib.compress(np.nan_to_num(h * 1000, nan=-32768).astype(np.int16).tobytes(), 1)
        now = time.time()
        header = json.dumps(dict(
            seq=self.seq, t=now, frame_ts=now * 1000,
            grid=dict(nx=self.nx, ny=self.ny, cell=CAM_CELL, x0=0.0, y0=0.0, unit="mm", nodata=-32768),
            imu=dict(accel=self.calib["accel0"],
                     tilt_change_deg=round(abs(self.rng.normal(0, 0.2 if st["moving"] else 0.03)), 3)),
            stats=self.summarize(h, CAM_CELL),
            sensors={"loadcell_g": None, "mpu6050_tilt_deg": None},
            calib=self.calib,
        )).encode()
        self.seq += 1
        return struct.pack("!I", len(header)) + header + body
