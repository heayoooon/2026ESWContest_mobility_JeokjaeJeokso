#!/usr/bin/env python3
"""물리29(GPIO5)/물리31(GPIO6) 조합을 양방향으로 시도.
모듈 쪽에서 DT/SCK를 반대로 꽂았을 수도 있으므로 둘 다 본다."""
import time
import lgpio

h = lgpio.gpiochip_open(0)


def free(p):
    try:
        lgpio.gpio_free(h, p)
    except Exception:
        pass


def s24(v):
    return v - 0x1000000 if v & 0x800000 else v


def test(sck, dt, n=6):
    free(dt)
    lgpio.gpio_claim_input(h, dt)
    free(sck)
    lgpio.gpio_claim_output(h, sck, 0)
    vals, highs = [], 0
    for _ in range(n):
        t0 = time.time()
        while time.time() - t0 < 0.5:
            if lgpio.gpio_read(h, dt) == 0:
                break
            time.sleep(0.001)
        acc = 0
        for _ in range(24):
            lgpio.gpio_write(h, sck, 1)
            lgpio.gpio_write(h, sck, 0)
            acc = (acc << 1) | lgpio.gpio_read(h, dt)
        lgpio.gpio_write(h, sck, 1)
        lgpio.gpio_write(h, sck, 0)
        highs += lgpio.gpio_read(h, dt)
        vals.append(s24(acc))
        time.sleep(0.11)
    free(sck)
    lgpio.gpio_claim_input(h, sck)
    ok = "정상" if all(v not in (0, -1) for v in vals) and highs >= n - 1 else "무응답"
    print(f"  SCK=GPIO{sck}/DT=GPIO{dt}: {vals}  읽은뒤HIGH {highs}/{n}  -> {ok}")


print("=== 정방향 / 역방향 ===")
test(5, 6)
test(6, 5)
print("\n=== 물리36(GPIO16), 물리38(GPIO20)도 역방향 포함 ===")
test(5, 16)
test(5, 20)
test(19, 16)
test(19, 20)
lgpio.gpiochip_close(h)
