#!/usr/bin/env python3
"""헤더에서 HX711을 찾는다. scan_hx711_full.py와 같은 방법이되 GPIO2/3은 건드리지 않는다.

scan_hx711_full.py는 I2C 핀(GPIO2/3)까지 출력으로 잡아서 MPU6050용 I2C 컨트롤러가
죽는다. 복구에 sudo dtparam이 필요하다. 이 스크립트는 2/3을 건너뛰므로 그 문제가 없다.

방법: 후보 SCK 핀을 0.5초 LOW로 잡아 HX711을 깨운 뒤(파워다운 해제), 나머지 핀을
읽고 → 25클럭 인가 → 다시 읽는다. LOW에서 HIGH로 바뀐 핀이 그 SCK와 짝인 DT다.
여러 DT가 한 SCK에 반응하면 클럭 공유 배선이다(정상).

결과는 화면에만 출력한다. cells.json을 덮어쓰지 않는다.
"""
import time

import lgpio

SKIP = {0, 1, 2, 3}                       # ID EEPROM + I2C
PINS = [p for p in range(0, 28) if p not in SKIP]

h = lgpio.gpiochip_open(0)                # Pi5 헤더 = pinctrl-rp1


def claim_in(p):
    try:
        lgpio.gpio_free(h, p)
    except Exception:
        pass
    lgpio.gpio_claim_input(h, p)


usable = []
for p in PINS:
    try:
        claim_in(p)
        usable.append(p)
    except Exception as e:
        print(f"  건너뜀 GPIO{p}: {e}")

low_idle = [p for p in usable if lgpio.gpio_read(h, p) == 0]
print(f"검사 대상: GPIO{usable[0]}~{usable[-1]} 중 {len(usable)}개 (GPIO2/3 제외)")
print(f"idle LOW인 핀: {['GPIO%d' % p for p in low_idle] or '없음'}"
      "   <- 준비된 HX711의 DOUT은 LOW로 보인다")
print()

pairs = {}
for sck in usable:
    others = [p for p in usable if p != sck]
    try:
        lgpio.gpio_free(h, sck)
        lgpio.gpio_claim_output(h, sck, 0)
    except Exception:
        continue
    time.sleep(0.5)                        # 변환 완료 대기 + 파워다운 해제
    before = {p: lgpio.gpio_read(h, p) for p in others}
    for _ in range(25):
        lgpio.gpio_write(h, sck, 1)
        lgpio.gpio_write(h, sck, 0)
    after = {p: lgpio.gpio_read(h, p) for p in others}

    hits = [p for p in others if before[p] == 0 and after[p] == 1]
    if hits:
        print(f"  SCK=GPIO{sck:<2d} -> DT {['GPIO%d' % p for p in hits]}")
        for p in hits:
            pairs.setdefault(p, sck)

    lgpio.gpio_free(h, sck)
    claim_in(sck)

for p in usable:
    try:
        lgpio.gpio_free(h, p)
    except Exception:
        pass
lgpio.gpiochip_close(h)

print()
print(f"HX711 발견: {len(pairs)}개")
for dt, sck in sorted(pairs.items()):
    print(f"  DT=GPIO{dt:<2d} SCK=GPIO{sck}")
if not pairs:
    print("  없음 — VCC/GND 결선, 모듈 전원 LED, 케이블 접촉을 확인할 것")
