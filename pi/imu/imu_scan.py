#!/usr/bin/env python3
"""Find every MPU-6050 on every I2C bus and show one sample from each."""
import os

from smbus2 import SMBus

from mpu6050_lib import BUSES, MPU6050, find_all

for bus in BUSES:
    if not os.path.exists(f"/dev/i2c-{bus}"):
        continue
    devs = []
    with SMBus(bus) as b:
        for a in range(0x03, 0x78):
            try:
                b.read_byte(a)
                devs.append(hex(a))
            except OSError:
                pass
    print(f"i2c-{bus}: devices {devs}")

mpus = find_all()
print("MPU-6050 found:", [f"bus{b}:{hex(a)}" for b, a in mpus] or "NONE")
for bus, a in mpus:
    m = MPU6050(a, bus)
    r = m.read_raw()
    roll, pitch = MPU6050.tilt(m.read())
    raw = {k: r[k] for k in ("ax", "ay", "az", "gx", "gy", "gz")}
    print(f"  bus{bus}:{hex(a)}: WHO_AM_I=0x{m.who_am_i():02x} raw={raw} "
          f"temp={r['temp']:.1f}C  roll={roll:+.1f} pitch={pitch:+.1f}")
    m.close()

if len(mpus) < 2:
    print("\nTip: a 2nd sensor on the same bus needs AD0 -> 3.3V (address 0x69).")
    print("     A sensor wired to non-I2C pins (e.g. GPIO5/6) needs the i2c-gpio overlay or rewiring to GPIO2/3.")
