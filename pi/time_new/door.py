"""적재함 문 열림 센서 MC-38 (리드 스위치) — lgpio.

배선: 스위치 두 가닥 중 하나는 DOOR_PIN, 다른 하나는 GND. 핀에 내부 풀업을 건다.
    문 닫힘 = 자석이 붙어 스위치가 닫힘 -> 핀이 GND 로 끌림 -> LOW
    문 열림 = 자석이 떨어져 스위치가 열림 -> 풀업으로       -> HIGH
반대로 나오면(스위치가 NO 타입이면) DOOR_INVERT=1.
"""


class Door:
    def __init__(self, pin, invert=False, chip=None):
        import lgpio                    # 파이 전용. PC 에서는 sim.SimDoor 를 쓴다
        self._lg = lgpio
        self.pin = pin
        self.invert = bool(invert)
        self._h = None
        last = None
        for c in ([chip] if chip is not None else [4, 0]):
            try:
                self._h = lgpio.gpiochip_open(c)
                self.chip = c
                break
            except Exception as e:      # 다음 후보로 넘어간다
                last = e
        if self._h is None:
            raise RuntimeError("gpiochip 열기 실패: %s" % last)
        r = lgpio.gpio_claim_input(self._h, pin, lgpio.SET_PULL_UP)
        if isinstance(r, int) and r < 0:
            raise RuntimeError("문 센서 GPIO%d 확보 실패 (lgpio %d)" % (pin, r))

    def is_open(self):
        """1 = 열림, 0 = 닫힘."""
        return self._lg.gpio_read(self._h, self.pin) ^ int(self.invert)

    def describe(self):
        return "GPIO%d 풀업%s" % (self.pin, " (반전)" if self.invert else "")

    def close(self):
        try:
            self._lg.gpio_free(self._h, self.pin)
        except Exception:               # 정리 중 오류는 무시
            pass
        try:
            self._lg.gpiochip_close(self._h)
        except Exception:
            pass
