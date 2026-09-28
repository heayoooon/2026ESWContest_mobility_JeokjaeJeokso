#!/usr/bin/env python3
"""물리핀 2개만 양방향으로 테스트.  사용법:  python3 pair.py 40 38"""
import sys
import time
import lgpio

P2B = {7: 4, 29: 5, 31: 6, 26: 7, 24: 8, 21: 9, 19: 10, 23: 11, 32: 12,
       33: 13, 8: 14, 10: 15, 36: 16, 11: 17, 12: 18, 35: 19, 38: 20,
       40: 21, 15: 22, 16: 23, 18: 24, 22: 25, 37: 26, 13: 27, 3: 2, 5: 3}
a, b = int(sys.argv[1]), int(sys.argv[2])
for x in (a, b):
    if x not in P2B:
        sys.exit(f"물리{x}번은 GPIO 핀이 아닙니다 (전원/GND이거나 스캔 제외)")
h = lgpio.gpiochip_open(0)


def free(p):
    try:
        lgpio.gpio_free(h, p)
    except Exception:
        pass


def s24(v):
    return v - 0x1000000 if v & 0x800000 else v


print("=== 핀 상태 (풀업/풀다운) ===")
for x in (a, b):
    p = P2B[x]
    free(p)
    lgpio.gpio_claim_input(h, p, lgpio.SET_PULL_UP)
    time.sleep(0.02)
    up = lgpio.gpio_read(h, p)
    free(p)
    lgpio.gpio_claim_input(h, p, lgpio.SET_PULL_DOWN)
    time.sleep(0.02)
    dn = lgpio.gpio_read(h, p)
    free(p)
    lgpio.gpio_claim_input(h, p)
    st = ("구동됨(레벨 %d) - 칩 핀에 닿아 있음" % up) if up == dn \
        else "플로팅 - 아무것도 안 물림"
    print(f"  물리{x:<2d}(GPIO{p:<2d}): pu={up} pd={dn} -> {st}")


def test(sck, dt, n=8):
    ps, pd_ = P2B[sck], P2B[dt]
    free(pd_)
    lgpio.gpio_claim_input(h, pd_)
    free(ps)
    lgpio.gpio_claim_output(h, ps, 0)
    vals, highs = [], 0
    for _ in range(n):
        t0 = time.time()
        while time.time() - t0 < 0.5:
            if lgpio.gpio_read(h, pd_) == 0:
                break
            time.sleep(0.001)
        acc = 0
        for _ in range(24):
            lgpio.gpio_write(h, ps, 1)
            lgpio.gpio_write(h, ps, 0)
            acc = (acc << 1) | lgpio.gpio_read(h, pd_)
        lgpio.gpio_write(h, ps, 1)
        lgpio.gpio_write(h, ps, 0)
        highs += lgpio.gpio_read(h, pd_)
        vals.append(s24(acc))
        time.sleep(0.11)
    free(ps)
    lgpio.gpio_claim_input(h, ps)
    good = all(v not in (0, -1) for v in vals) and highs >= n - 1
    spread = max(vals) - min(vals)
    print(f"\n  SCK=물리{sck} / DT=물리{dt}")
    print(f"    값 {vals}")
    print(f"    변동폭 {spread}   읽은뒤 DOUT HIGH {highs}/{n}   "
          f"-> {'정상' if good else '무응답'}")
    return good


print("\n=== 양방향 테스트 ===")
r1 = test(a, b)
r2 = test(b, a)
lgpio.gpiochip_close(h)
print("\n=== 결과 ===")
if r1 or r2:
    s, d = (a, b) if r1 else (b, a)
    print(f"  ★ HX711 정상:  SCK=물리{s}  DT=물리{d}")
else:
    print("  ✗ 무응답 - 이 두 핀으로는 HX711이 안 잡힙니다")
