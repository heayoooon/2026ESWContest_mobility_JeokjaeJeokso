#!/usr/bin/env python3
"""의심 조합만 집중 재시도: (SCK18,DT20) 노이즈인지, (SCK5,DT6) 정말 죽었는지."""
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


def test(sck, dt, n=8):
    free(dt)
    lgpio.gpio_claim_input(h, dt)
    free(sck)
    lgpio.gpio_claim_output(h, sck, 0)
    out = []
    for _ in range(n):
        t0 = time.time()
        rdy = False
        while time.time() - t0 < 0.5:
            if lgpio.gpio_read(h, dt) == 0:
                rdy = True
                break
            time.sleep(0.001)
        acc = 0
        for _ in range(24):
            lgpio.gpio_write(h, sck, 1)
            lgpio.gpio_write(h, sck, 0)
            acc = (acc << 1) | lgpio.gpio_read(h, dt)
        lgpio.gpio_write(h, sck, 1)
        lgpio.gpio_write(h, sck, 0)
        after = lgpio.gpio_read(h, dt)
        out.append((s24(acc), rdy, after))
        time.sleep(0.11)
    free(sck)
    lgpio.gpio_claim_input(h, sck)
    vals = [v for v, _, _ in out]
    highs = sum(1 for _, _, a in out if a == 1)
    print(f"  SCK=GPIO{sck} DT=GPIO{dt}: {vals}")
    print(f"      읽은 뒤 DOUT이 HIGH로 오른 횟수 {highs}/{len(out)}  "
          f"(진짜 HX711이면 거의 매번 O)")


print("=== 재시도 ===")
test(18, 20)
test(5, 6)
test(19, 13)   # 대조군: 정상 동작하는 것
lgpio.gpiochip_close(h)
