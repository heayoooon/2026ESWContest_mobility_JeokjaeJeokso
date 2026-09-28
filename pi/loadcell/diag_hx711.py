#!/usr/bin/env python3
"""HX711이 몇 개 붙어 있고 어느 핀인지 — 단계별로 원인을 좁히는 진단.

  python3 diag_hx711.py

1단계 (구동 핀 탐지): 핀마다 풀업으로 한 번, 풀다운으로 한 번 읽는다.
    값이 **같으면** 뭔가가 그 핀을 구동하고 있다 = HX711 DOUT 후보.
    값이 풀을 따라가면 아무것도 안 물려 있다 (또는 전원이 없다).
    HX711은 전원만 들어오면 DOUT을 항상 푸시풀로 구동한다(HIGH든 LOW든).
    → 이 단계에서 안 잡히면 그 모듈은 **전원 문제이거나 DOUT 배선이 끊긴 것**이다.

2단계 (짝짓기): 후보 SCK를 구동하며 **실제로 24비트를 읽어본다.** 값이 0도 포화도
    아니고 여러 번 읽어도 뭉치면 진짜 짝이다.
    "클럭 후 DOUT이 HIGH가 되나" 만 보는 방식은 쓰지 않는다 — 떠 있는 SCK 선이
    옆 핀 스위칭에 유도 결합해서 엉뚱한 핀이 줄줄이 짝으로 잡힌다(실측 확인).
    한 번의 클럭열로 DOUT 전부를 동시에 읽으므로 SCK 공유 배선도 그대로 잡힌다.

GPIO0~3은 SCK 후보로는 쓰되 DOUT 판정에서는 레벨만 참고한다 (ID EEPROM + I2C).
"""
import time

import lgpio

SKIP = {0, 1, 2, 3}
PINS = [p for p in range(0, 28) if p not in SKIP]
SAT = (-1, 8388607, -8388608)

h = lgpio.gpiochip_open(0)


def free(p):
    try:
        lgpio.gpio_free(h, p)
    except Exception:
        pass


def as_input(p):
    free(p)
    try:
        lgpio.gpio_claim_input(h, p)
    except Exception:
        pass


def read_with(p, pull):
    free(p)
    lgpio.gpio_claim_input(h, p, pull)
    time.sleep(0.003)
    v = lgpio.gpio_read(h, p)
    free(p)
    return v


def to_signed24(v):
    return v - 0x1000000 if v & 0x800000 else v


def clock_bank(sck, douts, runs=3, settle=0.25):
    """sck를 클럭하며 douts 전부를 매 클럭 동시 샘플링. 핀별 값 리스트를 반환."""
    out = {p: [] for p in douts}
    for _ in range(runs):
        t0 = time.time()
        while time.time() - t0 < settle:
            if all(lgpio.gpio_read(h, p) == 0 for p in douts):
                break
            time.sleep(0.001)
        acc = {p: 0 for p in douts}
        for _ in range(24):
            lgpio.gpio_write(h, sck, 1)
            bits = {p: lgpio.gpio_read(h, p) for p in douts}
            lgpio.gpio_write(h, sck, 0)
            for p in douts:
                acc[p] = (acc[p] << 1) | bits[p]
        lgpio.gpio_write(h, sck, 1)          # 25번째 = 다음도 채널A/게인128
        lgpio.gpio_write(h, sck, 0)
        for p in douts:
            out[p].append(to_signed24(acc[p]))
    return out


print("=== 칩 정보 ===")
for n in (0, 4):
    try:
        g = lgpio.gpiochip_open(n)
        print(f"  gpiochip{n}: {lgpio.gpio_get_chip_info(g)}")
        lgpio.gpiochip_close(g)
    except Exception as e:
        print(f"  gpiochip{n}: 열기 실패 ({e})")

print("\n=== 0단계: GPIO0~3 레벨 (기본 풀업) ===")
for p in sorted(SKIP):
    as_input(p)
    print(f"  GPIO{p}: {lgpio.gpio_read(h, p)}")

print("\n=== 1단계: 구동되는 핀 찾기 (풀업/풀다운 비교) ===")
driven, floating, failed = [], [], []
for p in PINS:
    try:
        up = read_with(p, lgpio.SET_PULL_UP)
        dn = read_with(p, lgpio.SET_PULL_DOWN)
    except Exception as e:
        failed.append((p, e))
        continue
    (driven if up == dn else floating).append((p, up) if up == dn else p)

for p, v in driven:
    state = "LOW  (변환 완료 = DRDY, 클럭 대기 중)" if v == 0 else "HIGH (변환 중이거나 파워다운)"
    print(f"  GPIO{p:<2d} 구동됨 → {state}")
if not driven:
    print("  (없음)")
print(f"\n  플로팅(아무것도 안 물림): {['GPIO%d' % p for p in floating]}")
if failed:
    print(f"  claim 실패: {['GPIO%d' % p for p, _ in failed]}")
print(f"\n  → 전원이 들어온 HX711 = {len(driven)}개")
if len(driven) < 4:
    print("  → 모자란 모듈은 **전원(VCC/GND) 또는 DOUT 배선** 문제다. SCK와 무관.")

dout_pins = [p for p, _ in driven]
for p in PINS:
    as_input(p)

print("\n=== 2단계: SCK 짝짓기 (실제 24비트를 읽어서 판정) ===")
pairs = {}
data = {}
for sck in [p for p in range(0, 28) if p not in dout_pins]:
    free(sck)
    try:
        lgpio.gpio_claim_output(h, sck, 0)
    except Exception:
        continue
    vals = clock_bank(sck, dout_pins)
    for p, vs in vals.items():
        spread = max(vs) - min(vs)
        real = all(v != 0 and v not in SAT for v in vs) and spread < 50000
        if real:
            print(f"  HX711: SCK=GPIO{sck}  DT=GPIO{p}   "
                  f"값 {vs[0]:+d} (변동폭 {spread})")
            pairs[p] = sck
            data[p] = vs
    free(sck)
    as_input(sck)

print(f"\n  → 짝 {len(pairs)}개")
orphan = [p for p in dout_pins if p not in pairs]
if orphan:
    print(f"  → 전원은 들어왔는데 아무 핀도 클럭을 못 주는 DOUT: "
          f"{['GPIO%d' % p for p in orphan]}")
    print("     GPIO0~27 전부를 SCK로 찔러봤는데 24비트가 전부 0으로만 나온다.")
    print("     = 그 모듈의 SCK 선이 GPIO에 안 닿아 있다 (안 꽂혔거나 GND/전원핀에 꽂힘).")

lgpio.gpiochip_close(h)

print("\n=== 요약 ===")
print(f"  전원이 들어온 HX711(DOUT 구동): {len(driven)}개")
print(f"  SCK까지 정상인 HX711:           {len(pairs)}개")
if pairs:
    print("\n  corners.json에 넣을 핀:")
    for dt, sck in sorted(pairs.items(), key=lambda kv: kv[1]):
        print(f'    {{"dt": {dt}, "sck": {sck}}}')
