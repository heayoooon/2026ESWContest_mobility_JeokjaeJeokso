#!/usr/bin/env python3
"""Angle calibration for every MPU-6050 found (any I2C bus).

Put each sensor in its REST pose (the pose you want to call 0 degrees) and keep
it still. The script detects which chip axis points up (so a module standing
on its side is fine), stores true biases + that "up" axis, and rotates the
axes at read time so roll/pitch are relative to the rest pose.
Result: calibration/imu_cal.json (read by sensors/mpu6050.py).

  python3 calibration/imu_calibrate.py                     # interactive, Enter per sensor
  python3 calibration/imu_calibrate.py --auto              # no prompts
  python3 calibration/imu_calibrate.py --auto --gyro-only  # gyro bias only (needs still, keeps accel/up)
  python3 calibration/imu_calibrate.py --only bus1:0x68    # limit to one sensor
  python3 calibration/imu_calibrate.py --udp --only esp32:0x68   # ESP32 wireless sensor (UDP, IMU_UDP_PORT, default 5005)
                                                                  # stop pipeline.py first (it holds the port)
"""
import json
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # pi/

from sensors.mpu6050 import (ACC_SCALE, CAL_PATH, MPU6050, cal_key, detect_up,  # noqa: E402
                             find_all, tilt, to_units)
from sensors.mpu6050_udp import MPU6050UDP, udp_key  # noqa: E402

AUTO = "--auto" in sys.argv
GYRO_ONLY = "--gyro-only" in sys.argv
UDP = "--udp" in sys.argv
ONLY = sys.argv[sys.argv.index("--only") + 1] if "--only" in sys.argv else None
AXES = ("ax", "ay", "az", "gx", "gy", "gz")    # read_raw()[1:7] order

# (key, open function). ESP32 over UDP has no bus; its key is esp32:<addr>
targets = [(cal_key(bus, a), lambda bus=bus, a=a: MPU6050(a, bus=bus)) for bus, a in find_all()]
if UDP:
    addr = int(os.environ.get("IMU_UDP_ADDR", "0x68"), 0)
    port = int(os.environ.get("IMU_UDP_PORT", "5005"), 0)
    targets.append((udp_key(addr), lambda: MPU6050UDP(addr, port=port, timeout=3.0)))
if not targets:
    raise SystemExit("no MPU-6050 found. run tools/imu_scan.py (ESP32: add --udp)")

cal = json.load(open(CAL_PATH)) if os.path.exists(CAL_PATH) else {}
STILL_SD = 25  # LSB; resting gyro sd is ~5

for key, open_dev in targets:
    if ONLY and key != ONLY:
        continue
    try:
        m = open_dev()
    except OSError as e:
        print(f"[{key}] cannot open: {e}")
        continue
    is_udp = key.startswith("esp32:")
    if is_udp:
        m.drain()                           # throw away packets queued before the sensor was laid flat
    if AUTO:
        print(f"\n[{key}] sampling now ({'still' if GYRO_ONLY else 'rest pose, still'})")
    else:
        input(f"\n[{key}] put this sensor in its rest pose, keep it still, then press Enter...")

    for attempt in range(1, 8):
        S = {k: [] for k in AXES}
        for _ in range(300):
            try:
                r = m.read_raw()
            except OSError as e:            # socket.timeout: ESP32 silent
                raise SystemExit(f"[{key}] no packet: {e}. Is the ESP32 on the same Wi-Fi / is pipeline.py still running?")
            for k, v in zip(AXES, r[1:7]):
                S[k].append(v)
            if not is_udp:
                time.sleep(0.01)
        noise = {k: statistics.pstdev(S[k]) for k in S}
        worst = max(noise["gx"], noise["gy"], noise["gz"])
        if worst <= STILL_SD:
            break
        print(f"  attempt {attempt}: sensor moved (gyro sd {worst:.0f} > {STILL_SD}), retrying in 2 s...")
        time.sleep(2)
    else:
        print("  WARNING: never got a still window; offsets may be off")

    mean = {k: statistics.mean(S[k]) for k in S}
    prev = cal.get(key, {})

    if GYRO_ONLY:
        up = prev.get("up", "+z")
        off = {k: prev.get(k, 0) for k in ("ax", "ay", "az")}
    else:
        up = detect_up(mean["ax"], mean["ay"], mean["az"])
        g_exp = {"ax": 0.0, "ay": 0.0, "az": 0.0}
        g_exp["a" + up[1]] = ACC_SCALE if up[0] == "+" else -ACC_SCALE
        off = {k: mean[k] - g_exp[k] for k in ("ax", "ay", "az")}  # true bias only
    off.update({k: mean[k] for k in ("gx", "gy", "gz")})

    gmag = (mean["ax"] ** 2 + mean["ay"] ** 2 + mean["az"] ** 2) ** 0.5 / ACC_SCALE
    entry = dict(off)
    entry["up"] = up
    cal[key] = entry
    roll, pitch = tilt(*to_units(m.read_raw()[1:7], entry)[:3])
    print(f"  up axis : {up}   |accel| = {gmag:.3f} g")
    print(f"  offsets : { {k: round(v) for k, v in off.items()} }")
    print(f"  noise sd: { {k: round(v, 1) for k, v in noise.items()} }")
    print(f"  tilt now: roll={roll:+.2f} pitch={pitch:+.2f}  (rest pose = 0)")
    m.close()

json.dump(cal, open(CAL_PATH, "w"), indent=2)
print(f"\nsaved {CAL_PATH}")
