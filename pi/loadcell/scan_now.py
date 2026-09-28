#!/usr/bin/env python3
"""전체 핀 스캔 (타이밍 교정판 + I2C 안전판).
- GPIO0~3은 건드리지 않는다 (I2C alt 유지)
- SCK는 반드시 sck=1; sck=0; 뒤에 DOUT을 읽는다 (HIGH 폭 최소화 -> 파워다운 방지)
"""
import time
import lgpio

PHYS = {4: 7, 5: 29, 6: 31, 7: 26, 8: 24, 9: 21, 10: 19, 11: 23, 12: 32,
        13: 33, 14: 8, 15: 10, 16: 36, 17: 11, 18: 12, 19: 35, 20: 38,
        21: 40, 22: 15, 23: 16, 24: 18, 25: 22, 26: 37, 27: 13}
PINS = list(range(4, 28))
h = lgpio.gpiochip_open(0)


def free(p):
    try:
        lgpio.gpio_free(h, p)
    except Exception:
        pass


def s24(v):
    return v - 0x1000000 if v & 0x800000 else v


def lbl(p):
    return f"물리{PHYS[p]:<2d}(GPIO{p})"


print("=== 1단계: 풀업/풀다운 비교 ===")
driven, floating = [], []
for p in PINS:
    free(p)
    lgpio.gpio_claim_input(h, p, lgpio.SET_PULL_UP)
    time.sleep(0.003)
    up = lgpio.gpio_read(h, p)
    free(p)
    lgpio.gpio_claim_input(h, p, lgpio.SET_PULL_DOWN)
    time.sleep(0.003)
    dn = lgpio.gpio_read(h, p)
    free(p)
    (driven if up == dn else floating).append((p, up) if up == dn else p)

for p, v in driven:
    print(f"  {lbl(p)} 구동됨 (레벨 {v})")
if not driven:
    print("  (없음)")
print(f"  플로팅: {[lbl(p) for p in floating]}")
print(f"  -> DOUT 후보 {len(driven)}개")

douts = [p for p, _ in driven]
for p in PINS:
    free(p)
    lgpio.gpio_claim_input(h, p)

print("\n=== 2단계: SCK 짝짓기 (하강엣지 뒤 샘플, 3회) ===")
pairs = {}
for sck in [p for p in PINS if p not in douts]:
    free(sck)
    try:
        lgpio.gpio_claim_output(h, sck, 0)
    except Exception:
        continue
    res = {p: [] for p in douts}
    for _ in range(3):
        t0 = time.time()
        while time.time() - t0 < 0.3:
            if all(lgpio.gpio_read(h, p) == 0 for p in douts):
                break
            time.sleep(0.001)
        acc = {p: 0 for p in douts}
        for _ in range(24):
            lgpio.gpio_write(h, sck, 1)
            lgpio.gpio_write(h, sck, 0)
            for p in douts:
                acc[p] = (acc[p] << 1) | lgpio.gpio_read(h, p)
        lgpio.gpio_write(h, sck, 1)
        lgpio.gpio_write(h, sck, 0)
        for p in douts:
            res[p].append(s24(acc[p]))
        time.sleep(0.11)
    for p, vs in res.items():
        spread = max(vs) - min(vs)
        if all(v not in (0, -1, 8388607, -8388608) for v in vs) and spread < 50000:
            print(f"  HX711 발견: SCK={lbl(sck)}  DT={lbl(p)}   "
                  f"값 {vs[0]:+d} (변동폭 {spread})")
            pairs[p] = sck
    free(sck)
    lgpio.gpio_claim_input(h, sck)

lgpio.gpiochip_close(h)
print("\n=== 요약 ===")
print(f"  정상 동작 HX711: {len(pairs)}개")
for dt, sck in sorted(pairs.items(), key=lambda kv: kv[1]):
    print(f"    SCK=물리{PHYS[sck]}(GPIO{sck})  DT=물리{PHYS[dt]}(GPIO{dt})")
orph = [p for p in douts if p not in pairs]
if orph:
    print(f"  전원/클럭 문제로 안 잡힌 DOUT 후보: {[lbl(p) for p in orph]}")
