#!/usr/bin/env python3
"""Why is the I2C bus empty? Probe addresses with errno, read SDA/SCL idle levels."""
import errno
import time

from smbus2 import SMBus

STUCK = (errno.ETIMEDOUT, errno.EREMOTEIO, errno.EBUSY, errno.EAGAIN, errno.EIO)

print("--- probe on i2c-1 (errno explains the failure) ---")
with SMBus(1) as b:
    for a in (0x68, 0x69):
        for _ in range(3):
            try:
                v = b.read_byte_data(a, 0x75)
                print(f"  {hex(a)}: OK WHO_AM_I=0x{v:02x}")
                break
            except OSError as e:
                name = errno.errorcode.get(e.errno, "?")
                if e.errno == errno.ENXIO:
                    why = "no ACK -> nothing answering at this address"
                elif e.errno in STUCK:
                    why = "bus stuck / held low / timeout -> short or wiring fault"
                else:
                    why = "other"
                print(f"  {hex(a)}: errno {e.errno} {name} -> {why}")
                time.sleep(0.2)

print("--- SDA(GPIO2) / SCL(GPIO3) idle level: 1 = released (normal), 0 = pulled low (short/stuck) ---")
try:
    import lgpio
    h = lgpio.gpiochip_open(0)
    for p, n in ((2, "SDA"), (3, "SCL")):
        try:
            lgpio.gpio_claim_input(h, p)
            vals = [lgpio.gpio_read(h, p) for _ in range(20)]
            lgpio.gpio_free(h, p)
            print(f"  GPIO{p} {n}: {vals[0]}   ({sum(vals)}/20 samples high)")
        except Exception as e:  # pin owned by the I2C controller
            print(f"  GPIO{p} {n}: cannot read via GPIO ({e})")
    lgpio.gpiochip_close(h)
except Exception as e:
    print("  lgpio unavailable:", e)

print("--- full scan i2c-1 ---")
found = []
with SMBus(1) as b:
    for a in range(0x03, 0x78):
        try:
            b.read_byte(a)
            found.append(hex(a))
        except OSError:
            pass
print("  devices:", found or "none")
