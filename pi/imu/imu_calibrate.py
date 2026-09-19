#!/usr/bin/env python3
"""Angle calibration for every MPU-6050 found (any I2C bus).

Put each sensor in its REST pose (the pose you want to call 0 degrees) and keep
it still. The script detects which chip axis points up (so a module standing
on its side is fine), stores true biases + that "up" axis, and rotates the
axes at read time so roll/pitch are relative to the rest pose.

  python3 imu_calibrate.py                     # interactive, Enter per sensor
  python3 imu_calibrate.py --auto              # no prompts
  python3 imu_calibrate.py --auto --gyro-only  # gyro bias only (needs still, keeps accel/up)
  python3 imu_calibrate.py --only bus1:0x68    # limit to one sensor
"""
import json
import os
import statistics
import sys
import time

from mpu6050_lib import ACC_SCALE, CAL, MPU6050, detect_up, find_all

AUTO = "--auto" in sys.argv
GYRO_ONLY = "--gyro-only" in sys.argv
ONLY = sys.argv[sys.argv.index("--only") + 1] if "--only" in sys.argv else None

mpus = find_all()
if not mpus:
    raise SystemExit("no MPU-6050 found. run imu_scan.py")

cal = json.load(open(CAL)) if os.path.exists(CAL) else {}
STILL_SD = 25  # LSB; resting gyro sd is ~5

for bus, a in mpus:
    m = MPU6050(a, bus)
    if ONLY and m.key != ONLY:
        m.close()
        continue
    m.off = {"ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0}
    m.up = "+z"
    if AUTO:
        print(f"\n[{m.key}] sampling now ({'still' if GYRO_ONLY else 'rest pose, still'})")
    else:
        input(f"\n[{m.key}] put this sensor in its rest pose, keep it still, then press Enter...")

    for attempt in range(1, 8):
        S = {k: [] for k in ("ax", "ay", "az", "gx", "gy", "gz")}
        for _ in range(300):
            r = m.read_raw()
            for k in S:
                S[k].append(r[k])
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
    prev = cal.get(m.key, {})

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
    cal[m.key] = entry
    m.off = off
    m.up = up
    roll, pitch = MPU6050.tilt(m.read())
    print(f"  up axis : {up}   |accel| = {gmag:.3f} g")
    print(f"  offsets : { {k: round(v) for k, v in off.items()} }")
    print(f"  noise sd: { {k: round(v, 1) for k, v in noise.items()} }")
    print(f"  tilt now: roll={roll:+.2f} pitch={pitch:+.2f}  (rest pose = 0)")
    m.close()

json.dump(cal, open(CAL, "w"), indent=2)
print(f"\nsaved {CAL}")
