#!/usr/bin/env python3
"""Find every MPU-6050 on every I2C bus and show one sample from each."""
import os
import sys

from smbus2 import SMBus

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # pi/

from sensors.mpu6050 import (BUSES, MPU6050, TEMP_OFFSET, TEMP_SCALE, cal_key,  # noqa: E402
                             find_all, find_cal, load_cal, tilt, to_units)

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

cal_path = find_cal()
cal = load_cal(cal_path) if cal_path else {}
mpus = find_all()
print("MPU-6050 found:", [cal_key(b, a) for b, a in mpus] or "NONE")
for bus, a in mpus:
    m = MPU6050(a, bus=bus)
    r = m.read_raw()
    roll, pitch = tilt(*to_units(r[1:7], cal.get(cal_key(bus, a)))[:3])
    raw = dict(zip(("ax", "ay", "az", "gx", "gy", "gz"), r[1:7]))
    print(f"  {cal_key(bus, a)}: WHO_AM_I=0x{m.who_am_i():02x} raw={raw} "
          f"temp={r[7] / TEMP_SCALE + TEMP_OFFSET:.1f}C  roll={roll:+.1f} pitch={pitch:+.1f}")
    m.close()

if len(mpus) < 2:
    print("\nTip: a 2nd sensor on the same bus needs AD0 -> 3.3V (address 0x69).")
    print("     A sensor wired to non-I2C pins (e.g. GPIO5/6) needs the i2c-gpio overlay or rewiring to GPIO2/3.")
