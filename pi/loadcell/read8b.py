#!/usr/bin/env python3
"""타이밍 교정판. SCK HIGH 구간을 최소화(하강엣지 뒤에 읽기)하고
실제 SCK HIGH 폭을 측정해서 같이 출력한다.
HX711은 SCK를 60us 이상 HIGH로 들면 파워다운되고 DOUT이 HIGH로 굳는다.
"""
import time
import lgpio

PHYS = {5: 29, 6: 31, 13: 33, 19: 35, 16: 36, 26: 37, 20: 38, 21: 40}
ALL = [5, 6, 13, 16, 19, 20, 21, 26]
DOUTS = [6, 13, 16]
SCKS = [5, 19, 20, 21, 26]
h = lgpio.gpiochip_open(0)


def free(p):
    try:
        lgpio.gpio_free(h, p)
    except Exception:
        pass


def s24(v):
    return v - 0x1000000 if v & 0x800000 else v


def lbl(p):
    return f"GPIO{p:<2d}(물리{PHYS[p]})"


for p in ALL:
    free(p)
    lgpio.gpio_claim_input(h, p)

print("=== SCK HIGH 폭 실측 (write 2회 사이) ===")
free(5)
lgpio.gpio_claim_output(h, 5, 0)
worst = 0.0
for _ in range(200):
    t0 = time.perf_counter()
    lgpio.gpio_write(h, 5, 1)
    lgpio.gpio_write(h, 5, 0)
    worst = max(worst, time.perf_counter() - t0)
print(f"  최악 HIGH 폭 ~= {worst*1e6:.1f} us  (60us 넘으면 파워다운)")
free(5)
lgpio.gpio_claim_input(h, 5)

print("\n=== DOUT 정상상태 1초 관찰 (HX711은 변환완료시 LOW로 내려 대기) ===")
for p in ALL:
    lo = hi = 0
    t0 = time.time()
    while time.time() - t0 < 1.0:
        if lgpio.gpio_read(h, p):
            hi += 1
        else:
            lo += 1
        time.sleep(0.0005)
    if lo and hi:
        v = f"토글중 (LOW {lo*100//(lo+hi)}%)  <== 살아있는 DOUT 신호"
    elif lo:
        v = "계속 LOW"
    else:
        v = "계속 HIGH"
    print(f"  {lbl(p)} {v}")

print("\n=== SCK 후보별 24비트 읽기 (하강엣지 뒤 샘플, 4회) ===")
for sck in SCKS:
    free(sck)
    lgpio.gpio_claim_output(h, sck, 0)
    print(f"\n  --- SCK = {lbl(sck)} ---")
    res = {p: [] for p in DOUTS}
    for _ in range(4):
        t0 = time.time()
        ready = False
        while time.time() - t0 < 0.4:
            if all(lgpio.gpio_read(h, p) == 0 for p in DOUTS):
                ready = True
                break
            time.sleep(0.001)
        acc = {p: 0 for p in DOUTS}
        for _ in range(24):
            lgpio.gpio_write(h, sck, 1)
            lgpio.gpio_write(h, sck, 0)
            for p in DOUTS:
                acc[p] = (acc[p] << 1) | lgpio.gpio_read(h, p)
        lgpio.gpio_write(h, sck, 1)
        lgpio.gpio_write(h, sck, 0)
        for p in DOUTS:
            res[p].append(s24(acc[p]))
        time.sleep(0.12)
    for p in DOUTS:
        vs = res[p]
        note = ""
        if all(v == 0 for v in vs):
            note = "  (전부 0 = 클럭 안 닿음/GND)"
        elif all(v == -1 for v in vs):
            note = "  (전부 -1 = DOUT 계속 HIGH = 변환 미완료/파워다운)"
        elif all(abs(v) > 8000000 for v in vs):
            note = "  (포화 = 로드셀 A+/A- 미연결 의심)"
        elif max(vs) - min(vs) < 50000:
            note = "  <== 정상 데이터"
        else:
            note = f"  (변동폭 {max(vs)-min(vs)} 큼)"
        print(f"      DT={lbl(p)} {vs}{note}")
    free(sck)
    lgpio.gpio_claim_input(h, sck)

lgpio.gpiochip_close(h)
