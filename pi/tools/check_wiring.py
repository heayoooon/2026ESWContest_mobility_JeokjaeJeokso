#!/usr/bin/env python3
"""공유 SCK 배선 점검. 모듈을 하나씩 붙이면서 이걸로 확인한다.

    python3 tools/check_wiring.py          # 1) 핀 상태 2) 모듈 검출 3) 한 번 읽기
    python3 tools/check_wiring.py live     # 실시간 raw + 기준 대비 변화량

배선을 6개 한 번에 하지 말 것. 모듈 1개 -> 확인 -> 2개 추가 -> 확인 순으로 간다.
6개 다 꽂고 안 되면 어디가 문제인지 찾는 데 몇 시간 쓴다.

live 모드가 라벨 확정용이다. 모서리를 하나씩만 눌러보면 그 채널만 크게 움직인다.
움직인 채널을 보고 HX_LABELS 순서를 정하면 된다.
"""
import os
import sys
import time

import lgpio

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # pi/

from acquisition.config import Config  # noqa: E402
from sensors.hx711_bus import HX711Bus, HX711Timeout  # noqa: E402


def probe_pins(cfg):
    """풀업/풀다운으로 두 번 읽어 능동 구동 여부를 본다.

    단순 레벨 읽기로는 '플로팅 LOW' 와 'DOUT 이 LOW 로 구동 중' 을 구분할 수 없다.
    풀을 바꿔도 값이 같으면 무언가가 그 핀을 실제로 구동하고 있다는 뜻이다.
    """
    print("1) 핀 상태 (풀업/풀다운 비교)")
    h = None
    for c in ([cfg.hx_chip] if cfg.hx_chip is not None else [4, 0]):
        try:
            h = lgpio.gpiochip_open(c)
            print("   gpiochip%d 열림" % c)
            break
        except Exception:
            continue
    if h is None:
        print("   gpiochip 못 엶")
        return

    def read_with(pin, pull):
        lgpio.gpio_claim_input(h, pin, pull)
        time.sleep(0.005)
        v = lgpio.gpio_read(h, pin)
        lgpio.gpio_free(h, pin)
        return v

    for label, pin in zip(cfg.hx_labels, cfg.hx_dt):
        try:
            up = read_with(pin, lgpio.SET_PULL_UP)
            dn = read_with(pin, lgpio.SET_PULL_DOWN)
        except Exception as e:
            print("   %-6s GPIO%-2d  읽기 실패: %s" % (label, pin, e))
            continue
        if up == dn:
            verdict = "구동됨 (HX711 살아있음, 레벨 %d)" % up
        else:
            verdict = "플로팅 (모듈 없음 / 전원 없음 / DT 선 안 꽂힘)"
        print("   %-6s GPIO%-2d  풀업=%d 풀다운=%d  -> %s" % (label, pin, up, dn, verdict))

    lgpio.gpiochip_close(h)
    print()


def open_bus(cfg):
    bus = HX711Bus(cfg.hx_sck, cfg.hx_dt, cfg.hx_labels,
                   gain=cfg.hx_gain, pull=cfg.hx_pull, chip=cfg.hx_chip,
                   invert=cfg.hx_invert)
    print(bus.describe())
    print("   sync_reset (전 모듈 동시 파워다운 -> 재시작, 변환 위상 정렬)")
    bus.sync_reset()
    return bus


def detect(bus):
    print("\n2) 모듈 검출 (DT 가 한 번이라도 LOW 로 떨어지는가)")
    present = bus.present(timeout=1.0)
    for label, pin, ok in zip(bus.labels, bus.dts, present):
        print("   %-6s GPIO%-2d  %s" % (label, pin, "OK" if ok else "안 보임"))
    if not any(present):
        print("\n   하나도 안 보인다. 확인할 것:")
        print("   - VCC 3.3V / GND 가 레일에 제대로 물렸는지")
        print("   - SCK 레일과 파이 GPIO%d 가 이어졌는지" % bus.sck)
        print("   - 로드셀(빨강 E+ / 검정 E- / 초록 A+ / 흰색 A-)이 모듈에 물렸는지")
    print()
    return present


def read_once(bus, cfg):
    print("3) 한 번 읽기")
    try:
        bus.read(timeout=cfg.hx_timeout)        # 파워다운 직후 첫 값 버림
        s = bus.read(timeout=cfg.hx_timeout)
    except HX711Timeout as e:
        print("   타임아웃: %s" % e)
        return
    for label, v in zip(bus.labels, s.values):
        print("   %-6s %10d" % (label, v))
    print("   SCK HIGH 최대 %.1fus%s" % (s.max_high_ns / 1e3,
                                         "  (60us 초과 - 값 의심)" if s.max_high_ns > 60_000 else ""))
    if s.flags:
        print("   flags: %s" % ", ".join(s.flags))
    print()


def live(bus, cfg):
    print("실시간. 모서리를 하나씩만 눌러 어느 채널이 움직이는지 본다. Ctrl+C 로 종료.")
    print("(기준값은 시작할 때의 값. r 대신 그냥 다시 실행하면 기준이 새로 잡힌다)\n")
    base = None
    try:
        while True:
            try:
                s = bus.read(timeout=cfg.hx_timeout)
            except HX711Timeout as e:
                sys.stdout.write("\r타임아웃: %-50s" % e)
                sys.stdout.flush()
                bus.sync_reset(settle=0.3)
                continue
            if base is None:
                base = list(s.values)
            d = [v - b for v, b in zip(s.values, base)]
            big = max(range(len(d)), key=lambda i: abs(d[i])) if d else None
            cells = "  ".join(
                "%s%s:%+8d" % ("*" if i == big and abs(d[i]) > 1000 else " ", l, d[i])
                for i, l in enumerate(bus.labels))
            sys.stdout.write("\r%s   sck%5.1fus %-12s" %
                             (cells, s.max_high_ns / 1e3, "|".join(s.flags)))
            sys.stdout.flush()
    except KeyboardInterrupt:
        print()


def main():
    cfg = Config()
    print(cfg.describe())
    print()
    mode = sys.argv[1] if len(sys.argv) > 1 else ""

    probe_pins(cfg)
    bus = open_bus(cfg)
    try:
        if mode == "live":
            live(bus, cfg)
        else:
            detect(bus)
            read_once(bus, cfg)
            print("모서리별 라벨을 정하려면:  python3 tools/check_wiring.py live")
    finally:
        bus.close()


if __name__ == "__main__":
    main()
