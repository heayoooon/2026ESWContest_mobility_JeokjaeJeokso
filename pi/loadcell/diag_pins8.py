#!/usr/bin/env python3
"""사용자가 꽂은 8개 핀만 집중 진단.
물리핀 29,31,33,35,36,37,38,40 = BCM 5,6,13,16,19,20,21,26
전원: 물리1(3.3V), 물리6(GND)

A: 풀업/풀다운 비교 -> 구동중(=HX711 DOUT 후보) / 플로팅(=아무것도 없음 or SCK선)
B: 핀마다 SCK로 삼아 25클럭 -> 나머지 7핀 동시 24비트 읽기 (필터 없이 원시값)
   + 클럭 직후 DOUT이 HIGH로 올라가는지 확인
     = 진짜 HX711이면 읽은 뒤 다음 변환까지 DOUT을 HIGH로 올린다.
       GND에 그냥 물린 선은 절대 HIGH가 안 된다.
"""
import time
import lgpio

PHYS = {5: 29, 6: 31, 13: 33, 19: 35, 16: 36, 26: 37, 20: 38, 21: 40}
PINS = [5, 6, 13, 16, 19, 20, 21, 26]
h = lgpio.gpiochip_open(0)


def free(p):
    try:
        lgpio.gpio_free(h, p)
    except Exception:
        pass


def s24(v):
    return v - 0x1000000 if v & 0x800000 else v


def lbl(p):
    return f"GPIO{p}(물리{PHYS[p]})"


print("=== A: 풀업/풀다운 비교 ===")
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
    if up == dn:
        driven.append((p, up))
        print(f"  {lbl(p):<16} 구동됨  레벨={up}  -> HX711 DOUT 후보")
    else:
        floating.append(p)
        print(f"  {lbl(p):<16} 플로팅          -> 아무것도 안 물림 (또는 SCK선)")

print(f"\n  구동 {len(driven)}개 / 플로팅 {len(floating)}개")

for p in PINS:
    free(p)
    lgpio.gpio_claim_input(h, p)

print("\n=== B: 핀마다 SCK로 찔러보기 (원시 24비트 + 클럭후 HIGH 여부) ===")
hit = {}
for sck in PINS:
    douts = [p for p in PINS if p != sck]
    free(sck)
    try:
        lgpio.gpio_claim_output(h, sck, 0)
    except Exception as e:
        print(f"  SCK={lbl(sck)} claim 실패: {e}")
        continue
    print(f"\n  --- SCK = {lbl(sck)} ---")
    for _ in range(2):
        t0 = time.time()
        while time.time() - t0 < 0.3:
            if any(lgpio.gpio_read(h, p) == 0 for p in douts):
                break
            time.sleep(0.001)
        acc = {p: 0 for p in douts}
        for _ in range(24):
            lgpio.gpio_write(h, sck, 1)
            b = {p: lgpio.gpio_read(h, p) for p in douts}
            lgpio.gpio_write(h, sck, 0)
            for p in douts:
                acc[p] = (acc[p] << 1) | b[p]
        lgpio.gpio_write(h, sck, 1)
        lgpio.gpio_write(h, sck, 0)
        went_high = {p: False for p in douts}
        t0 = time.time()
        while time.time() - t0 < 0.02:
            for p in douts:
                if lgpio.gpio_read(h, p) == 1:
                    went_high[p] = True
            time.sleep(0.0005)
    for p in douts:
        v = s24(acc[p])
        if v != 0:
            sat = " [포화=로드셀 미연결/단선 의심]" if abs(v) > 8000000 else ""
            print(f"      DT={lbl(p):<16} 값 {v:+9d}  "
                  f"클럭후HIGH={'O' if went_high[p] else 'X'}{sat}")
            hit.setdefault(p, []).append(sck)
        elif went_high[p]:
            print(f"      DT={lbl(p):<16} 값 0 인데 클럭후 HIGH=O (뭔가 반응)")
    free(sck)
    lgpio.gpio_claim_input(h, sck)

lgpio.gpiochip_close(h)
print("\n=== 요약 ===")
print(f"  전원 들어온 DOUT 후보: {[lbl(p) for p, _ in driven]}")
print(f"  실제 데이터 나온 짝  : "
      + (", ".join(f"DT={lbl(p)}<-SCK={[lbl(s) for s in v]}"
                   for p, v in hit.items()) or "없음"))
