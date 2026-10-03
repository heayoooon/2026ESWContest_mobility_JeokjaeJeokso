"""센서를 열고 Frame(acquisition/frame.py)을 낸다.

    src = FrameSource(cfg)              # 파이 센서
    src = FrameSource(cfg, sim=True)    # PC 가짜 센서 (sensors/sim.py)
    for frame in src.frames():          # src.stop() 을 부르면 끝난다
        ...
    src.close()

한 프레임 = HX711 샘플 한 번 (RATE 10SPS 면 약 10Hz). HX711 변환을 기다리는 ~100ms 동안 IMU 를
IMU_HZ 로 읽어 그 구간의 평균 · 최대 · 현재 기울기를 같은 프레임에 붙인다.
시작할 때 DT 가 안 떨어지는 모듈은 읽기에서 빼고, 프레임에는 못 연 IMU 와 함께 None 으로 남긴다.
"""
import os
import sys
import time
import types

from acquisition.console import say
from acquisition.frame import Frame, ImuReading
from sensors.hx711_bus import HX711Timeout
from sensors.mpu6050 import cal_key, find_cal, load_cal, tilt, to_units
from sensors.mpu6050_udp import MPU6050UDP, udp_key

G_TO_N = 9.80665 / 1000     # 질량(g) -> 힘(N)
ALPHA = 0.98                # 기울기 상보필터 (tools/imu_angle.py 와 같은 값)


# --------------------------------------------------------------------------
# IMU: HX711 을 기다리는 동안 읽어서 프레임마다 정리
# --------------------------------------------------------------------------
class ImuSet:
    """IMU 들을 IMU_HZ 로 읽어 쌓아두고, 프레임마다 구간 평균 · 최대 + 현재 기울기를 내준다."""

    def __init__(self, sensors, hz):
        self.sensors = sensors                  # [(라벨, 장치, imu_cal 항목 또는 None)]
        self.labels = [s[0] for s in sensors]
        self.period = 1.0 / max(1, hz)
        self.next_t = time.monotonic()
        self.sum = {l: [0.0] * 6 for l in self.labels}
        self.peak = {l: [0.0, 0.0] for l in self.labels}   # ax, ay
        self.n = {l: 0 for l in self.labels}
        self.angle = {l: None for l in self.labels}     # (roll, pitch, 마지막 시각)
        self.errors = {l: 0 for l in self.labels}

    def poll(self):
        """주기가 됐으면 전 IMU 를 한 번씩 읽는다. 아니면 바로 돌아간다."""
        now = time.monotonic()
        if not self.sensors or now < self.next_t:
            return
        self.next_t += self.period
        if self.next_t < now:
            self.next_t = now + self.period     # 밀렸으면 기준을 다시 잡는다
        for label, dev, cal in self.sensors:
            try:
                # ESP32 무선(drain)은 기다리지 않고 쌓인 패킷 전부, I2C 는 한 번 읽는다
                rows = dev.drain() if hasattr(dev, "drain") else [dev.read_raw()]
            except Exception:                   # I2C 한 번 실패는 흔하다. 세고 넘어간다
                self.errors[label] += 1
                continue
            for r in rows:
                self._add(label, cal, r)

    def _add(self, label, cal, r):
        v = to_units(r[1:7], cal)
        s = self.sum[label]
        for i in range(6):
            s[i] += v[i]
        p = self.peak[label]
        for i in range(2):
            if abs(v[i]) > abs(p[i]):
                p[i] = v[i]
        self.n[label] += 1

        # 상보필터: 자이로를 적분하고 가속도(중력 방향)로 천천히 끌어당긴다
        ar, ap = tilt(v[0], v[1], v[2])
        t = r[0] / 1e9
        prev = self.angle[label]
        if prev is None:
            self.angle[label] = (ar, ap, t)
        else:
            dt = t - prev[2]
            self.angle[label] = (ALPHA * (prev[0] + v[3] * dt) + (1 - ALPHA) * ar,
                                 ALPHA * (prev[1] + v[4] * dt) + (1 - ALPHA) * ap, t)

    def take(self):
        """지난 take() 이후 구간 값 {라벨: ImuReading}. 그 사이 한 번도 못 읽은 IMU 는 None."""
        out = {}
        for l in self.labels:
            n = self.n[l]
            if n == 0:
                out[l] = None
                continue
            ax, ay, az, gx, gy, gz = (x / n for x in self.sum[l])
            roll, pitch, _ = self.angle[l]
            out[l] = ImuReading(ax=ax, ay=ay, az=az, gx=gx, gy=gy, gz=gz,
                                roll=roll, pitch=pitch,
                                ax_peak=self.peak[l][0], ay_peak=self.peak[l][1], n=n)
            self.sum[l] = [0.0] * 6
            self.peak[l] = [0.0, 0.0]
            self.n[l] = 0
        return out

    def close(self):
        for _, dev, _ in self.sensors:
            dev.close()


# --------------------------------------------------------------------------
# 장치 열기
# --------------------------------------------------------------------------
def boost_priority():
    """비트뱅잉이 스케줄러에 밀리면 SCK 가 오래 HIGH 로 떠서 칩이 파워다운된다."""
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(10))
        return "SCHED_FIFO(10)"
    except Exception:
        pass
    try:
        os.nice(-10)
        return "nice -10"
    except Exception:
        return "일반 우선순위 (sudo -E 로 돌리면 타이밍이 안정된다)"


def open_hx(cfg, keep=None):
    """HX711Bus 를 연다. keep 을 주면 그 인덱스의 모듈만."""
    from sensors.hx711_bus import HX711Bus
    idx = list(range(len(cfg.hx_dt))) if keep is None else keep
    sck = cfg.hx_sck[0] if cfg.hx_shared_sck else [cfg.hx_sck[i] for i in idx]
    return HX711Bus(sck, [cfg.hx_dt[i] for i in idx], [cfg.hx_labels[i] for i in idx],
                    gain=cfg.hx_gain, pull=cfg.hx_pull, chip=cfg.hx_chip,
                    invert=cfg.hx_invert)


def open_real(cfg, notes):
    from calibration.calibrate import CALIB_PATH, load_calib
    from sensors.mpu6050 import MPU6050

    try:
        hx = open_hx(cfg)
    except Exception as e:
        sys.exit("HX711 열기 실패: %s: %s\nPC 에서 시험하려면: python pipeline.py --sim"
                 % (type(e).__name__, e))
    hx.sync_reset()                             # 전 모듈 변환 위상 정렬
    missing = hx.missing()
    if missing:
        if len(missing) == hx.n:
            hx.close()
            sys.exit("HX711 이 하나도 안 보인다 (DT 가 계속 HIGH). "
                     "전원/배선부터 확인: python3 tools/check_wiring.py")
        say("! 안 보이는 모듈 %s -> 읽기에서 빼고 빈칸으로 기록한다 (DT 가 계속 HIGH)" % ", ".join(missing))
        notes.append("시작할 때 안 보여서 뺀 채널: %s" % ", ".join(missing))
        keep = [i for i, l in enumerate(cfg.hx_labels) if l not in missing]
        hx.close()
        hx = open_hx(cfg, keep)
        hx.sync_reset()

    cal_path = find_cal(cfg.imu_cal)
    imu_cal = load_cal(cal_path) if cal_path else {}
    imus = []
    for addr, label in zip(cfg.imu_addrs, cfg.imu_i2c_labels):
        m = None
        try:
            m = MPU6050(addr, bus=cfg.imu_bus, hz=cfg.imu_hz, label=label)
            m.check()
        except Exception as e:
            if m is not None:
                m.close()
            say("! IMU %s(%s) 못 엶: %s" % (label, hex(addr), e))
            notes.append("IMU %s(%s) 못 엶: %s" % (label, hex(addr), e))
            continue
        imus.append((label, m, imu_cal.get(cal_key(cfg.imu_bus, addr))))
    if cfg.imu_udp_port:                        # ESP32 무선 센서
        try:
            u = MPU6050UDP(cfg.imu_udp_addr, port=cfg.imu_udp_port, label=cfg.imu_udp_label)
            imus.append((cfg.imu_udp_label, u, imu_cal.get(udp_key(cfg.imu_udp_addr))))
            say("IMU   %s ESP32 UDP %d 대기 (패킷이 와야 값이 찬다)" % (cfg.imu_udp_label, cfg.imu_udp_port))
        except OSError as e:
            say("! IMU %s UDP %d 못 엶: %s" % (cfg.imu_udp_label, cfg.imu_udp_port, e))
            notes.append("IMU %s UDP %d 못 엶: %s" % (cfg.imu_udp_label, cfg.imu_udp_port, e))
    if not imus:
        say("  i2cdetect -y %d 로 68/69 가 보이는지. 진단 스크립트를 돌린 뒤라면 "
            "pinctrl set 2 a3 pu; pinctrl set 3 a3 pu 로 I2C 핀을 되돌린다" % cfg.imu_bus)

    calib = load_calib()
    return types.SimpleNamespace(
        hx=hx, imus=imus, calib=calib,
        calib_src=("%s (%s)" % (CALIB_PATH, calib.get("updated", "?"))
                   if os.path.exists(CALIB_PATH) else "없음"),
        imu_cal_src=cal_path or ("IMU_CAL=%s 파일 없음" % cfg.imu_cal if cfg.imu_cal
                                 else "없음 (바이어스/축 보정 안 함)"))


def open_sim(cfg):
    from sensors import sim
    scn = sim.Scenario()
    hx = sim.SimHX711Bus(cfg, scn)
    imus = [(l, sim.SimMPU6050(l, scn, i), None) for i, l in enumerate(cfg.imu_labels)]
    return types.SimpleNamespace(
        hx=hx, imus=imus, calib=hx.calib(),
        calib_src="시뮬레이션 내장값", imu_cal_src="시뮬레이션 (평평하게 놓임)")


# --------------------------------------------------------------------------
# 프레임 소스
# --------------------------------------------------------------------------
class FrameSource:
    def __init__(self, cfg, sim=False):
        self.cfg = cfg
        self.is_sim = sim
        self.notes = []                         # 요약 시트 메모로 갈 것들
        dev = open_sim(cfg) if sim else open_real(cfg, self.notes)
        self.hx = dev.hx
        self.calib = dev.calib
        self.calib_src = dev.calib_src
        self.imu_cal_src = dev.imu_cal_src
        self.imu_cal = {l: c for l, _, c in dev.imus}    # 열린 IMU -> imu_cal 항목 (없으면 None)
        say(self.hx.describe())

        self.cal = self._usable_cal()           # {라벨: (offset, g/count)}
        self.imus = ImuSet(dev.imus, cfg.imu_hz)
        for l, c in self.imu_cal.items():
            say("IMU   %s 열림  보정 %s" % (l, ("있음 (up %s)" % c.get("up", "+z")) if c else "없음"))
        if not sim:
            say("우선순위  %s" % boost_priority())

        self.weight, self.tension, _ = cfg.roles()
        self.timeouts = 0
        self.t0_ns = None
        self.t0_wall = None
        self._stopping = False

    def _usable_cal(self):
        """calib.json 에서 지금 쓸 수 있는 채널 보정값. 부호 반전이 바뀐 채널은 뺀다."""
        labels = self.hx.labels
        flipped = []
        if not self.is_sim:
            from calibration.calibrate import stale
            flipped = stale(self.calib, self.cfg, labels)
        if flipped:
            say("! HX_INVERT 가 영점 잡을 때와 다른 채널 %s -> raw 만 적는다. calibrate.py tare 부터 다시"
                % ", ".join(flipped))
            self.notes.append("HX_INVERT 가 영점 잡을 때와 달라 보정을 안 쓴 채널: %s" % ", ".join(flipped))
        cal = {l: (self.calib["offset"][l], self.calib["scale_g_per_count"][l])
               for l in labels
               if l not in flipped
               and self.calib["offset"].get(l) is not None
               and self.calib["scale_g_per_count"].get(l) is not None}
        if not cal:
            say("보정  없음 -> 원시 카운트만 적는다. calibrate.py 로 tare / span / cell 을 먼저 하면 "
                "무게(g)·장력(N) 값이 생긴다.")
            self.notes.append("calib.json 에 보정값이 없어 무게(g)/장력(N) 없이 raw 만 적었다.")
        elif len(cal) < len(labels):
            miss = [l for l in labels if l not in cal]
            say("보정  %d/%d 채널 (없는 채널: %s -> raw 만)" % (len(cal), len(labels), ", ".join(miss)))
            self.notes.append("보정값이 없어 raw 만 적은 채널: %s" % ", ".join(miss))
        return cal

    def stop(self):
        """frames() 를 끝낸다. 시그널 핸들러에서 불러도 된다."""
        self._stopping = True

    def frames(self):
        cfg, hx = self.cfg, self.hx
        try:
            hx.read(timeout=cfg.hx_timeout)     # 파워다운 직후 첫 값은 버린다
        except Exception:
            pass
        self.t0_ns = time.monotonic_ns()
        self.t0_wall = time.time()

        while not self._stopping:
            # HX711 변환(~100ms)을 기다리는 동안 IMU 를 IMU_HZ 로 읽는다
            limit = time.monotonic() + cfg.hx_timeout
            while not hx.ready() and time.monotonic() < limit and not self._stopping:
                self.imus.poll()
                time.sleep(0.0005)
            if self._stopping:
                return
            try:
                s = hx.read(timeout=0.05)       # 준비됐으면 바로 클럭, 아니면 곧 타임아웃
            except HX711Timeout as e:
                self.timeouts += 1
                say("\n타임아웃 (DRDY 안 옴): %s" % ", ".join(e.waiting))
                hx.sync_reset(settle=0.3)       # 위상 다시 맞추고 계속
                continue
            yield self._frame(s, self.imus.take())

    def _frame(self, s, imu):
        raw = dict(zip(self.hx.labels, s.values))
        g = {l: (raw[l] - off) * sc for l, (off, sc) in self.cal.items()}
        flags = list(s.flags)
        for l in self.cfg.imu_labels:
            if imu.get(l) is None and l in self.imu_cal:   # 못 연 IMU 는 매번 플래그 대신 메모에
                flags.append("imu_miss:" + l)
        t_s = (s.t_ns - self.t0_ns) / 1e9
        return Frame(
            seq=s.seq, t_ns=s.t_ns, t_s=t_s, wall=self.t0_wall + t_s,
            load_g={l: g.get(l) for l in self.weight},
            tension_n={l: g[l] * G_TO_N if l in g else None for l in self.tension},
            raw={l: raw.get(l) for l in self.cfg.hx_labels},
            imu={l: imu.get(l) for l in self.cfg.imu_labels},
            flags=flags)

    def close(self):
        self.hx.close()
        self.imus.close()
