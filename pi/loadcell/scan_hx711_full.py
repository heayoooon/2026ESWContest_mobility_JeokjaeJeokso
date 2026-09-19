#!/usr/bin/env python3
"""Find every HX711 on the 40-pin header, INCLUDING GPIO2/3 and shared-SCK setups.

Method: for each candidate SCK pin, hold it low 0.5 s (wakes a powered-down
HX711), read all other pins, send 25 clock pulses, read again. Any pin that
went LOW -> HIGH is a DT (DOUT) paired with that SCK. Several DTs reacting to
one SCK means those modules share the clock line, which is fine.

WARNING: touches GPIO2/3 -> the I2C controller must be reset afterwards
(sudo dtparam i2c_arm=off && sleep 1 && sudo dtparam i2c_arm=on).
"""
import json
import time

import lgpio

PINS = list(range(2, 28))
h = lgpio.gpiochip_open(0)


def claim_in(p):
    try:
        lgpio.gpio_free(h, p)
    except Exception:
        pass
    lgpio.gpio_claim_input(h, p)


def read_all(pins):
    return {p: lgpio.gpio_read(h, p) for p in pins}


for p in PINS:
    claim_in(p)
idle = read_all(PINS)
print("idle levels:", {f"GPIO{p}": v for p, v in idle.items() if v == 0}, "(pins reading LOW; a ready HX711 DOUT reads LOW)")

pairs = {}   # dt -> sck
for sck in PINS:
    others = [p for p in PINS if p != sck]
    try:
        lgpio.gpio_free(h, sck)
        lgpio.gpio_claim_output(h, sck, 0)
    except Exception as e:
        print(f"skip GPIO{sck}: {e}")
        continue
    time.sleep(0.5)
    before = read_all(others)
    for _ in range(25):
        lgpio.gpio_write(h, sck, 1)
        lgpio.gpio_write(h, sck, 0)
    after = read_all(others)
    hits = [d for d in others if before[d] == 0 and after[d] == 1]
    if hits:
        print(f"SCK GPIO{sck:2d} -> DT {['GPIO%d' % d for d in hits]}")
        for d in hits:
            pairs.setdefault(d, sck)
    lgpio.gpio_free(h, sck)
    claim_in(sck)

for p in PINS:
    try:
        lgpio.gpio_free(h, p)
    except Exception:
        pass
lgpio.gpiochip_close(h)

cells = [{"dt": d, "sck": s} for d, s in sorted(pairs.items())]
print("\nHX711 modules found:", len(cells))
for c in cells:
    print(f"  DT=GPIO{c['dt']:2d}  SCK=GPIO{c['sck']:2d}")
json.dump(cells, open("/home/abcd/loadcell/cells.json", "w"), indent=1)
print("saved cells.json")
if any(c["dt"] in (2, 3) or c["sck"] in (2, 3) for c in cells):
    print("\nNOTE: a module is on GPIO2/3 = the I2C pins used by the MPU-6050s. Move it to other GPIOs.")
