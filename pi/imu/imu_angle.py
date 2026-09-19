#!/usr/bin/env python3
"""Live roll / pitch for every MPU-6050 (complementary filter). Ctrl+C to stop."""
import time

from mpu6050_lib import MPU6050, find_all

mpus = [MPU6050(a, b) for b, a in find_all()]
if not mpus:
    raise SystemExit("no MPU-6050 found. run imu_scan.py")

state = {m.key: [0.0, 0.0] for m in mpus}
ALPHA = 0.98
t = time.time()
print("   ".join(f"[{m.key}] roll  pitch" for m in mpus))
try:
    while True:
        dt = time.time() - t
        t = time.time()
        out = []
        for m in mpus:
            d = m.read()
            ar, ap = MPU6050.tilt(d)
            r, p = state[m.key]
            r = ALPHA * (r + d["gx"] * dt) + (1 - ALPHA) * ar
            p = ALPHA * (p + d["gy"] * dt) + (1 - ALPHA) * ap
            state[m.key] = [r, p]
            out.append(f"[{m.key}] {r:+6.1f} {p:+6.1f}")
        print("   ".join(out), end="\r", flush=True)
        time.sleep(0.02)
except KeyboardInterrupt:
    print()
finally:
    for m in mpus:
        m.close()
