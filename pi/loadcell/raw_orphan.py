#!/usr/bin/env python3
"""짝을 못 찾은 DOUT 후보(GPIO6,16,20)만 SCK 후보 전부에 대해 원시값 출력."""
import time
import lgpio

PHYS = {4: 7, 5: 29, 6: 31, 7: 26, 8: 24, 9: 21, 10: 19, 11: 23, 12: 32,
        13: 33, 14: 8, 15: 10, 16: 36, 17: 11, 18: 12, 19: 35, 20: 38,
        21: 40, 22: 15, 23: 16, 24: 18, 25: 22, 26: 37, 27: 13}
DOUTS = [6, 16, 20]
h = lgpio.gpiochip_open(0)


def free(p):
    try:
        lgpio.gpio_free(h, p)
    except Exception:
        pass


def s24(v):
    return v - 0x1000000 if v & 0x800000 else v


for p in DOUTS:
    free(p)
    lgpio.gpio_claim_input(h, p)

print("핀별 1초 관찰 (HX711이면 읽기 전엔 LOW로 대기):")
for p in DOUTS:
    lo = hi = 0
    t0 = time.time()
    while time.time() - t0 < 1.0:
        if lgpio.gpio_read(h, p):
            hi += 1
        else:
            lo += 1
        time.sleep(0.0005)
    print(f"  물리{PHYS[p]}(GPIO{p}): LOW {lo*100//(lo+hi)}% / HIGH {hi*100//(lo+hi)}%")

print("\nSCK 후보별 원시 24비트:")
for sck in [p for p in range(4, 28) if p not in DOUTS]:
    free(sck)
    try:
        lgpio.gpio_claim_output(h, sck, 0)
    except Exception:
        continue
    acc = {p: 0 for p in DOUTS}
    for _ in range(24):
        lgpio.gpio_write(h, sck, 1)
        lgpio.gpio_write(h, sck, 0)
        for p in DOUTS:
            acc[p] = (acc[p] << 1) | lgpio.gpio_read(h, p)
    lgpio.gpio_write(h, sck, 1)
    lgpio.gpio_write(h, sck, 0)
    vals = {p: s24(acc[p]) for p in DOUTS}
    mark = "  <== 반응" if any(v not in (0, -1) for v in vals.values()) else ""
    print(f"  SCK=물리{PHYS[sck]:<2d}(GPIO{sck:<2d})  "
          + "  ".join(f"G{p}={vals[p]:+9d}" for p in DOUTS) + mark)
    free(sck)
    lgpio.gpio_claim_input(h, sck)
    time.sleep(0.11)

lgpio.gpiochip_close(h)
