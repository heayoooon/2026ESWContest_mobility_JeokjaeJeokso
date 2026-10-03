#!/usr/bin/env python3
"""HX711(공유 SCK) 6채널 + MPU-6050 2개를 공통 시계로 기록한다.

    python3 acquisition/log_sync.py                  # Ctrl+C 까지
    LOG_SECONDS=60 python3 acquisition/log_sync.py   # 60초
    HX_DT=5,6,13 HX_LABELS=FL,FR,RL python3 acquisition/log_sync.py

핀 배치는 전부 환경변수다. config.py 의 docstring 참고.

만들어지는 파일 (LOG_DIR/<런 이름>/)
    load.csv    t,t_ns,seq,<라벨...>,sck_us,flags
                한 행 = 전 채널 동시 샘플. 공유 SCK 라 타임스탬프 하나가 전 채널에 유효하다.
    imu.csv     t,t_ns,imu,ax,ay,az,gx,gy,gz,temp      raw LSB
    events.log  타임아웃/경고
    meta.json   시계 기준점, 핀 배치, 변환계수, 통계

시계
    전부 CLOCK_MONOTONIC(ns). 시스템 전체 시계라 프로세스가 달라도 같은 값이다.
    NTP 가 시계를 점프시켜도 영향이 없다. 절대시각은 meta.json 의
    t0_mono_ns / t0_wall_unix 로 환산한다:
        wall = t0_wall_unix + (t_ns - t0_mono_ns) / 1e9
    csv 의 t 열은 이미 (t_ns - t0_mono_ns)/1e9 로 넣어둔 상대 초다.

구조
    HX711 수집 / IMU 수집 / 파일 쓰기를 각각 다른 프로세스에 둔다. 스레드로 묶으면
    GIL 때문에 HX711 비트뱅잉 도중 선점당해 SCK 가 HIGH 로 오래 머물 수 있고,
    그러면 칩이 파워다운으로 들어가 샘플이 깨진다. 프로세스를 나누면 그 간섭이 없다.
"""
import json
import os
import signal
import statistics
import sys
import time
from multiprocessing import get_context

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # pi/

from acquisition.config import Config  # noqa: E402

STATUS_EVERY = 0.5      # 화면 갱신 주기(초)
FLUSH_EVERY = 1.0       # 파일 flush 주기(초)


# --------------------------------------------------------------------------
# 수집 프로세스
# --------------------------------------------------------------------------
def _boost_priority(q, who):
    """비트뱅잉이 스케줄러에 밀리지 않게. 권한 없으면 조용히 넘어간다."""
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(10))
        q.put(("info", who, "SCHED_FIFO(10) 적용"))
        return
    except Exception:
        pass
    try:
        os.nice(-10)
        q.put(("info", who, "nice -10 적용"))
    except Exception:
        q.put(("info", who, "우선순위 조정 못 함(일반 권한). sudo 로 돌리면 타이밍이 안정된다"))


def hx_worker(cfg, q, stop):
    signal.signal(signal.SIGINT, signal.SIG_IGN)    # 종료는 stop 이벤트로만
    from sensors.hx711_bus import HX711Bus, HX711Timeout

    _boost_priority(q, "hx711")
    try:
        bus = HX711Bus(cfg.hx_sck, cfg.hx_dt, cfg.hx_labels,
                       gain=cfg.hx_gain, pull=cfg.hx_pull, chip=cfg.hx_chip,
                       invert=cfg.hx_invert)
    except Exception as e:
        q.put(("fatal", "hx711", "%s: %s" % (type(e).__name__, e)))
        return

    q.put(("info", "hx711", bus.describe()))
    bus.sync_reset()                                # 전 모듈 변환 위상 정렬
    missing = bus.missing()
    if missing:
        q.put(("warn", "hx711", "DT 가 계속 HIGH = 모듈 안 보임: " + ", ".join(missing)))

    try:
        bus.read(timeout=cfg.hx_timeout)            # 파워다운 직후 첫 값은 버린다
    except Exception as e:
        q.put(("warn", "hx711", "첫 읽기 실패(무시): %s" % e))

    while not stop.is_set():
        try:
            s = bus.read(timeout=cfg.hx_timeout)
        except HX711Timeout as e:
            q.put(("timeout", "hx711", time.monotonic_ns(), ", ".join(e.waiting)))
            bus.sync_reset(settle=0.3)              # 위상 다시 맞추고 계속
            continue
        except Exception as e:
            q.put(("fatal", "hx711", "%s: %s" % (type(e).__name__, e)))
            break
        q.put(("hx", s.t_ns, s.seq, s.values, s.max_high_ns, s.flags))

    bus.close()
    q.put(("info", "hx711", "정지"))


def imu_worker(cfg, q, stop):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    from sensors.mpu6050 import MPU6050

    mpus = []
    for addr, label in zip(cfg.imu_addrs, cfg.imu_i2c_labels):
        try:
            m = MPU6050(addr, bus=cfg.imu_bus, hz=cfg.imu_hz, label=label)
            who = m.check()
            mpus.append(m)
            q.put(("info", "imu", "%s %s 열림 (WHO_AM_I=%s, %.1fHz)"
                   % (label, hex(addr), hex(who), m.actual_hz)))
        except Exception as e:
            q.put(("warn", "imu", "%s %s 못 엶: %s" % (label, hex(addr), e)))

    if not mpus:
        q.put(("fatal", "imu", "MPU-6050 이 하나도 안 열림. "
                               "i2cdetect -y %d 로 68/69 보이는지 확인. "
                               "안 보이면 GPIO2/3 이 GPIO 모드로 남은 것이니 재부팅" % cfg.imu_bus))
        return

    period = 1.0 / max(1, cfg.imu_hz)
    next_t = time.monotonic()
    errors = 0
    while not stop.is_set():
        for m in mpus:
            try:
                r = m.read_raw()
            except Exception as e:
                errors += 1
                if errors in (1, 10, 100) or errors % 1000 == 0:
                    q.put(("warn", "imu", "%s 읽기 실패 %d회: %s" % (m.label, errors, e)))
                continue
            q.put(("imu", r[0], m.label, r[1], r[2], r[3], r[4], r[5], r[6], r[7]))

        next_t += period
        slack = next_t - time.monotonic()
        if slack > 0:
            time.sleep(slack)
        else:
            next_t = time.monotonic()               # 밀렸으면 기준을 다시 잡는다

    for m in mpus:
        m.close()
    q.put(("info", "imu", "정지 (읽기 실패 %d회)" % errors))


# --------------------------------------------------------------------------
# 통계
# --------------------------------------------------------------------------
class Stat:
    """샘플 간격 통계. 전부 담아두면 메모리가 커지니 요약값 + 일부 표본만."""
    MAX_KEEP = 20000

    def __init__(self, expect_dt):
        self.n = 0
        self.prev = None
        self.first = None
        self.last = None
        self.dt_min = None
        self.dt_max = None
        self.dt_sum = 0.0
        self.late = 0                # 기대 간격의 2배를 넘은 횟수
        self.expect_dt = expect_dt
        self.keep = []

    def add(self, t_ns):
        self.n += 1
        if self.first is None:
            self.first = t_ns
        self.last = t_ns
        if self.prev is not None:
            dt = (t_ns - self.prev) / 1e9
            self.dt_sum += dt
            if self.dt_min is None or dt < self.dt_min:
                self.dt_min = dt
            if self.dt_max is None or dt > self.dt_max:
                self.dt_max = dt
            if self.expect_dt and dt > 2 * self.expect_dt:
                self.late += 1
            if len(self.keep) < self.MAX_KEEP:
                self.keep.append(dt)
        self.prev = t_ns

    def summary(self):
        if self.n < 2:
            return {"count": self.n}
        span = (self.last - self.first) / 1e9
        return {
            "count": self.n,
            "span_s": round(span, 3),
            "rate_hz": round((self.n - 1) / span, 3) if span > 0 else None,
            "dt_min_ms": round(self.dt_min * 1e3, 3),
            "dt_med_ms": round(statistics.median(self.keep) * 1e3, 3) if self.keep else None,
            "dt_max_ms": round(self.dt_max * 1e3, 3),
            "late_count": self.late,
            "late_threshold_ms": round(self.expect_dt * 2e3, 1) if self.expect_dt else None,
        }


# --------------------------------------------------------------------------
# 메인
# --------------------------------------------------------------------------
def main():
    cfg = Config()
    print(cfg.describe())
    print()

    # 끝까지 다 기록해놓고 마지막에 import 로 죽는 일이 없게 여기서 확인한다
    try:
        from sensors.mpu6050 import scales
        imu_scales = scales()
    except Exception as e:
        print("! mpu6050 import 실패 (%s). IMU 없이 진행한다. pip install smbus2\n" % e)
        imu_scales = {}

    run_dir = os.path.join(cfg.log_dir, cfg.log_name)
    os.makedirs(run_dir, exist_ok=True)

    # 모든 프로세스가 같은 기준점을 쓴다
    t0_mono = time.monotonic_ns()
    t0_wall = time.time()

    ctx = get_context("fork")
    q = ctx.Queue(maxsize=100000)
    stop = ctx.Event()

    procs = [
        ctx.Process(target=hx_worker, args=(cfg, q, stop), name="hx711"),
        ctx.Process(target=imu_worker, args=(cfg, q, stop), name="imu"),
    ]
    for p in procs:
        p.start()

    load_f = open(os.path.join(run_dir, "load.csv"), "w", buffering=1 << 16)
    imu_f = open(os.path.join(run_dir, "imu.csv"), "w", buffering=1 << 16)
    ev_f = open(os.path.join(run_dir, "events.log"), "w", buffering=1)
    load_f.write("t,t_ns,seq,%s,sck_us,flags\n" % ",".join(cfg.hx_labels))
    imu_f.write("t,t_ns,imu,ax,ay,az,gx,gy,gz,temp\n")

    st_hx = Stat(1.0 / cfg.hx_rate)
    st_imu = {l: Stat(1.0 / cfg.imu_hz) for l in cfg.imu_i2c_labels}
    n_flagged = 0
    n_timeout = 0
    fatal = []
    last_status = 0.0
    last_flush = time.monotonic()
    last_vals = None

    def event(text):
        line = "[%7.3f] %s" % ((time.monotonic_ns() - t0_mono) / 1e9, text)
        ev_f.write(line + "\n")
        print("\n" + line, flush=True)

    deadline = None if cfg.log_seconds <= 0 else t0_mono + int(cfg.log_seconds * 1e9)
    print("기록 시작. Ctrl+C 로 종료.\n")

    try:
        while True:
            if deadline and time.monotonic_ns() >= deadline:
                break
            try:
                msg = q.get(timeout=0.2)
            except Exception:
                msg = None

            if msg is not None:
                kind = msg[0]
                if kind == "hx":
                    _, t_ns, seq, vals, max_high, flags = msg
                    st_hx.add(t_ns)
                    if flags:
                        n_flagged += 1
                    last_vals = vals
                    load_f.write("%.6f,%d,%d,%s,%.1f,%s\n" % (
                        (t_ns - t0_mono) / 1e9, t_ns, seq,
                        ",".join(str(v) for v in vals),
                        max_high / 1e3, "|".join(flags)))
                elif kind == "imu":
                    _, t_ns, label, ax, ay, az, gx, gy, gz, temp = msg
                    s = st_imu.get(label)
                    if s is None:
                        s = st_imu[label] = Stat(1.0 / cfg.imu_hz)
                    s.add(t_ns)
                    imu_f.write("%.6f,%d,%s,%d,%d,%d,%d,%d,%d,%d\n" % (
                        (t_ns - t0_mono) / 1e9, t_ns, label, ax, ay, az, gx, gy, gz, temp))
                elif kind == "timeout":
                    n_timeout += 1
                    event("HX711 타임아웃: %s" % msg[3])
                elif kind in ("info", "warn"):
                    event("[%s] %s" % (msg[1], msg[2]))
                elif kind == "fatal":
                    fatal.append("%s: %s" % (msg[1], msg[2]))
                    event("치명적 [%s] %s" % (msg[1], msg[2]))

            now = time.monotonic()
            if now - last_status >= STATUS_EVERY:
                last_status = now
                el = (time.monotonic_ns() - t0_mono) / 1e9
                imu_n = sum(s.n for s in st_imu.values())
                shown = ""
                if last_vals:
                    shown = "  " + " ".join("%s=%d" % (l, v)
                                            for l, v in zip(cfg.hx_labels, last_vals))
                sys.stdout.write(
                    "\r%6.1fs  load %5d (%4.1fHz)  imu %6d (%5.1fHz)  타임아웃 %d  의심 %d%s   "
                    % (el, st_hx.n, st_hx.n / el if el else 0,
                       imu_n, imu_n / el if el else 0, n_timeout, n_flagged, shown))
                sys.stdout.flush()

            if now - last_flush >= FLUSH_EVERY:
                last_flush = now
                load_f.flush()
                imu_f.flush()

            if len(fatal) >= 2:
                break
    except KeyboardInterrupt:
        pass

    print("\n정리 중...")
    stop.set()
    for p in procs:
        p.join(timeout=3)
        if p.is_alive():
            p.terminate()

    # 큐에 남은 것 비우기
    drained = 0
    while True:
        try:
            msg = q.get_nowait()
        except Exception:
            break
        drained += 1
        if msg[0] == "hx":
            _, t_ns, seq, vals, max_high, flags = msg
            st_hx.add(t_ns)
            load_f.write("%.6f,%d,%d,%s,%.1f,%s\n" % (
                (t_ns - t0_mono) / 1e9, t_ns, seq,
                ",".join(str(v) for v in vals), max_high / 1e3, "|".join(flags)))
        elif msg[0] == "imu":
            _, t_ns, label, ax, ay, az, gx, gy, gz, temp = msg
            st_imu.setdefault(label, Stat(1.0 / cfg.imu_hz)).add(t_ns)
            imu_f.write("%.6f,%d,%s,%d,%d,%d,%d,%d,%d,%d\n" % (
                (t_ns - t0_mono) / 1e9, t_ns, label, ax, ay, az, gx, gy, gz, temp))

    for f in (load_f, imu_f, ev_f):
        f.flush()
        f.close()

    meta = {
        "clock": {
            "source": "CLOCK_MONOTONIC (time.monotonic_ns)",
            "t0_mono_ns": t0_mono,
            "t0_wall_unix": t0_wall,
            "t0_wall_iso": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(t0_wall)),
            "wall_from_t_ns": "t0_wall_unix + (t_ns - t0_mono_ns)/1e9",
        },
        "config": cfg.as_dict(),
        "imu_scales": imu_scales,
        "stats": {
            "load": st_hx.summary(),
            "imu": {k: v.summary() for k, v in st_imu.items()},
            "hx_timeouts": n_timeout,
            "hx_flagged_samples": n_flagged,
            "drained_after_stop": drained,
        },
        "fatal": fatal,
    }
    with open(os.path.join(run_dir, "meta.json"), "w") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    print("\n저장: %s" % run_dir)
    print(json.dumps(meta["stats"], indent=2, ensure_ascii=False))
    if n_flagged:
        print("\n! 의심 샘플 %d개 (flags 열 확인). sck_high 가 많으면 sudo 로 실행해 볼 것." % n_flagged)
    if fatal:
        print("\n! 치명적 오류: " + " / ".join(fatal))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
