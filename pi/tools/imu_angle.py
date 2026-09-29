#!/usr/bin/env python3
"""Live roll / pitch for every MPU-6050 (complementary filter). Ctrl+C to stop."""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # pi/

from sensors.mpu6050 import MPU6050, cal_key, find_all, find_cal, load_cal, tilt, to_units  # noqa: E402

mpus = [(cal_key(b, a), MPU6050(a, bus=b)) for b, a in find_all()]
if not mpus:
    raise SystemExit("no MPU-6050 found. run tools/imu_scan.py")

cal_path = find_cal()
cal = load_cal(cal_path) if cal_path else {}
state = {key: [0.0, 0.0] for key, _ in mpus}
ALPHA = 0.98
t = time.time()
print("   ".join(f"[{key}] roll  pitch" for key, _ in mpus))
try:
    while True:
        dt = time.time() - t
        t = time.time()
        out = []
        for key, m in mpus:
            d = to_units(m.read_raw()[1:7], cal.get(key))
            ar, ap = tilt(d[0], d[1], d[2])
            r, p = state[key]
            r = ALPHA * (r + d[3] * dt) + (1 - ALPHA) * ar
            p = ALPHA * (p + d[4] * dt) + (1 - ALPHA) * ap
            state[key] = [r, p]
            out.append(f"[{key}] {r:+6.1f} {p:+6.1f}")
        print("   ".join(out), end="\r", flush=True)
        time.sleep(0.02)
except KeyboardInterrupt:
    print()
finally:
    for _, m in mpus:
        m.close()
