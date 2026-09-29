#!/usr/bin/env python3
"""Is the bus held by a slave during a transaction, or is the controller itself stuck?

Samples SDA/SCL at high speed in a thread while a transaction is attempted.
  - lines stay high the whole time and the transfer still fails  -> controller stuck (needs I2C reset / reboot)
  - SDA (or SCL) goes low and stays low                          -> a slave/wire is holding the bus (wiring fault)
  - lines toggle briefly then release                            -> controller is driving, slaves just not ACKing
"""
import threading
import time

import lgpio
from smbus2 import SMBus

samples = []
stop = False


def sampler():
    h = lgpio.gpiochip_open(0)
    lgpio.gpio_claim_input(h, 2)
    lgpio.gpio_claim_input(h, 3)
    t0 = time.time()
    while not stop:
        samples.append((time.time() - t0, lgpio.gpio_read(h, 2), lgpio.gpio_read(h, 3)))
    lgpio.gpio_free(h, 2)
    lgpio.gpio_free(h, 3)
    lgpio.gpiochip_close(h)


th = threading.Thread(target=sampler)
th.start()
time.sleep(0.05)
t_tx = time.time()
try:
    with SMBus(1) as b:
        b.read_byte_data(0x68, 0x75)
    result = "transfer OK"
except OSError as e:
    result = f"transfer failed errno {e.errno}"
t_end = time.time()
time.sleep(0.05)
stop = True
th.join()

n = len(samples)
sda_low = sum(1 for _, s, _ in samples if s == 0)
scl_low = sum(1 for _, _, c in samples if c == 0)
print(f"{result}; transfer took {t_end - t_tx:.3f}s; {n} samples")
print(f"SDA low in {sda_low}/{n} samples, SCL low in {scl_low}/{n} samples")
if sda_low == 0 and scl_low == 0:
    print("-> lines never moved: the I2C CONTROLLER is stuck (not the wiring). Reset needed:")
    print("   sudo dtparam i2c_arm=off && sleep 1 && sudo dtparam i2c_arm=on")
elif sda_low > n * 0.5 or scl_low > n * 0.5:
    print("-> a line is being HELD LOW: wiring short or a slave holding the bus (check sensor 2 wires)")
else:
    print("-> controller is toggling the lines; bus is electrically alive")
# show first transitions
last = None
shown = 0
for t, s, c in samples:
    if (s, c) != last and shown < 12:
        print(f"   t={t*1000:7.2f} ms  SDA={s} SCL={c}")
        last = (s, c)
        shown += 1
