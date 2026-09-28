#!/usr/bin/env python3
"""GPIO16(전원은 들어왔는데 짝을 못 찾은 DOUT)만 집중 확인.
필터 없이 SCK 후보별 원시 24비트를 전부 출력한다."""
import time
import lgpio

SKIP = {0, 1, 2, 3}
DOUTS = [6, 13, 16]
h = lgpio.gpiochip_open(0)


def free(p):
    try:
        lgpio.gpio_free(h, p)
    except Exception:
        pass


def to_signed24(v):
    return v - 0x1000000 if v & 0x800000 else v


for p in DOUTS:
    free(p)
    lgpio.gpio_claim_input(h, p)

for sck in [p for p in range(0, 28) if p not in DOUTS and p not in SKIP]:
    free(sck)
    try:
        lgpio.gpio_claim_output(h, sck, 0)
    except Exception:
        continue
    vals = {p: [] for p in DOUTS}
    for _ in range(3):
        t0 = time.time()
        while time.time() - t0 < 0.25:
            if lgpio.gpio_read(h, 16) == 0:
                break
            time.sleep(0.001)
        acc = {p: 0 for p in DOUTS}
        for _ in range(24):
            lgpio.gpio_write(h, sck, 1)
            bits = {p: lgpio.gpio_read(h, p) for p in DOUTS}
            lgpio.gpio_write(h, sck, 0)
            for p in DOUTS:
                acc[p] = (acc[p] << 1) | bits[p]
        lgpio.gpio_write(h, sck, 1)
        lgpio.gpio_write(h, sck, 0)
        for p in DOUTS:
            vals[p].append(to_signed24(acc[p]))
    v16 = vals[16]
    mark = "  <== 반응!" if any(v != 0 for v in v16) else ""
    print(f"SCK=GPIO{sck:<2d}  DT16={v16}{mark}")
    free(sck)
    lgpio.gpio_claim_input(h, sck)

lgpio.gpiochip_close(h)
