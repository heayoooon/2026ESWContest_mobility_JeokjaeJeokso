"""HX711 다채널 드라이버 (lgpio). SCK 는 공유 배선과 모듈별 배선을 둘 다 받는다.

배선 A — 공유 SCK (`sck=16`)
    파이 GPIO 하나 --+-- SCK -- HX711 #1 -- DT -- 파이 GPIO (모듈마다 따로)
                     +-- SCK -- HX711 #2 -- DT -- ...
                     +-- ...                      (SCK 만 전부 병렬)

배선 B — 모듈별 SCK (`sck=[21, 26, 17, 13]`)
    모듈마다 SCK 가 따로 파이로 온다. 이 경우 드라이버가 `group_write` 로
    **여러 SCK 를 호출 한 번에 같이 토글**한다. 커널이 한 번에 세팅하므로 채널 간
    엣지 시차가 사실상 없고, 공유 배선과 같은 동시성을 얻는다. 납땜을 안 바꿔도 된다.
    (그룹을 못 잡으면 핀별 write 로 물러서는데, 이때는 채널 간 수 us 시차가 생긴다)

어느 쪽이든
  * 24비트를 한 번 클럭하면 전 채널이 동시에 쏟아진다  -> 타임스탬프 1개면 충분
  * 읽기가 끝나는 순간 모든 칩의 다음 변환이 같이 시작된다 -> 매 읽기마다 위상 재정렬

주의: PD_SCK 가 60us 넘게 HIGH 면 HX711 은 파워다운으로 들어간다. 파이썬은 언제든
선점될 수 있으므로
  * 상승/하강 엣지를 붙여서 치고, 데이터는 SCK 를 내린 뒤에 읽는다
    (DOUT 은 다음 상승엣지까지 값을 유지하므로 읽어도 된다)
  * 실제 HIGH 폭을 매번 재서 60us 를 넘긴 샘플에는 sck_high 플래그를 남긴다
"""
import time
from collections import namedtuple

try:
    import lgpio
except ImportError:         # PC 에서도 Sample / HX711Timeout 은 import 할 수 있게 (sim.py)
    lgpio = None

GAIN_PULSES = {128: 1, 64: 3, 32: 2}   # 128/64 = 채널 A, 32 = 채널 B
POWERDOWN_NS = 60_000                  # 데이터시트: PD_SCK HIGH 60us 이상 -> 파워다운
FULL_SCALE = 0x7FFFFF

Sample = namedtuple("Sample", "t_ns seq values max_high_ns flags")


class HX711Timeout(RuntimeError):
    """정해진 시간 안에 모든 DT 가 LOW 로 떨어지지 않았다."""

    def __init__(self, waiting):
        self.waiting = list(waiting)
        super().__init__("DRDY 안 온 채널: " + (", ".join(self.waiting) or "?"))


def _ok(r, what):
    """lgpio 는 설정에 따라 음수 코드를 돌려주기도 한다. 둘 다 오류로 본다."""
    if isinstance(r, int) and r < 0:
        raise RuntimeError("%s 실패 (lgpio %d)" % (what, r))
    return r


class HX711Bus:
    def __init__(self, sck, dts, labels=None, gain=128, pull="up", chip=None, invert=()):
        if lgpio is None:
            raise RuntimeError("lgpio 모듈이 없다 (라즈베리파이 전용)")
        if len(set(dts)) != len(dts):
            raise ValueError("DT 핀 중복: %r" % (dts,))
        if gain not in GAIN_PULSES:
            raise ValueError("gain 은 128/64/32")

        # sck 는 정수 하나(공유 배선) 또는 리스트(모듈별 배선) 둘 다 받는다
        self.scks = [sck] if isinstance(sck, int) else list(sck)
        if not self.scks:
            raise ValueError("SCK 핀이 비어 있다")
        if len(set(self.scks)) != len(self.scks):
            raise ValueError("SCK 핀 중복: %r" % (self.scks,))
        clash = set(self.scks) & set(dts)
        if clash:
            raise ValueError("SCK 와 DT 에 같은 핀: %s"
                             % ", ".join("GPIO%d" % p for p in sorted(clash)))
        if len(self.scks) > 1 and len(self.scks) != len(dts):
            raise ValueError("모듈별 SCK 는 DT 와 개수가 같아야 한다 (SCK %d개, DT %d개)"
                             % (len(self.scks), len(dts)))
        self.shared_sck = len(self.scks) == 1
        self.sck = self.scks[0]         # 공유 배선일 때의 이름. 로그/메타용
        self._sck_all = (1 << len(self.scks)) - 1

        self.dts = list(dts)
        self.n = len(self.dts)
        self.labels = list(labels) if labels else ["c%d" % (i + 1) for i in range(self.n)]
        if len(self.labels) != self.n:
            raise ValueError("labels 개수가 DT 개수와 다르다")
        # 누르면 값이 줄어드는 셀(장착 방향이 반대 등). A+/A- 를 바꿔 꽂은 것과 같다.
        # 없는 라벨은 무시한다 (안 보이는 모듈을 빼고 다시 연 경우)
        self.invert = [l for l in self.labels if l in invert]
        self._neg = {i for i, l in enumerate(self.labels) if l in invert}
        self.gain = gain
        self._gain_pulses = GAIN_PULSES[gain]
        self.seq = 0
        self.notes = []

        self._pull = {"up": lgpio.SET_PULL_UP,
                      "down": lgpio.SET_PULL_DOWN,
                      "none": lgpio.SET_PULL_NONE}[pull]

        self._h, self.chip = self._open_chip(chip)
        self._sck_grouped = self._claim_scks()
        self._grouped = self._claim_dts()

    # --- 초기화 ----------------------------------------------------------
    @staticmethod
    def _open_chip(chip):
        candidates = [chip] if chip is not None else [4, 0]
        last = None
        for c in candidates:
            try:
                return lgpio.gpiochip_open(c), c
            except Exception as e:      # 다음 후보로 넘어간다
                last = e
        raise RuntimeError("gpiochip 열기 실패 (시도 %r, 마지막 오류 %s)" % (candidates, last))

    def _claim_scks(self):
        """SCK 가 여럿이면 그룹 출력으로 잡는다. group_write 한 번이면 커널이 전 핀을
        같이 세팅하므로 채널 간 엣지 시차가 없다. 그룹을 못 잡으면 핀별 출력."""
        if self.shared_sck:
            _ok(lgpio.gpio_claim_output(self._h, self.sck, 0),
                "SCK GPIO%d 출력 확보" % self.sck)
            return False
        try:
            # levels 는 핀 개수만큼의 리스트다. 정수 하나를 주면 lgpio 가 len() 을 쳐서 깨진다
            _ok(lgpio.group_claim_output(self._h, self.scks, [0] * len(self.scks)),
                "SCK 그룹 확보")
            lgpio.group_write(self._h, self.scks[0], 0)
            return True
        except Exception as e:
            self.notes.append("SCK 그룹 못 씀(%s) -> 핀별 write 로 전환 "
                              "(채널 간 수 us 시차)" % e)
            try:
                lgpio.group_free(self._h, self.scks[0])
            except Exception:           # 애초에 안 잡혔을 수 있다
                pass
            for p in self.scks:
                _ok(lgpio.gpio_claim_output(self._h, p, 0), "SCK GPIO%d 출력 확보" % p)
            return False

    def _sck_write(self, level):
        if self._sck_grouped:
            lgpio.group_write(self._h, self.scks[0], self._sck_all if level else 0)
        else:
            for p in self.scks:
                lgpio.gpio_write(self._h, p, level)

    def _claim_dts(self):
        """가능하면 그룹으로 잡는다. 그룹 읽기는 호출 한 번에 전 채널을 같이 읽어서
        채널 간 미세 시차가 없고 비트당 호출 수도 1/n 로 준다."""
        try:
            _ok(lgpio.group_claim_input(self._h, self.dts, self._pull), "DT 그룹 확보")
            r = lgpio.group_read(self._h, self.dts[0])
            if not (isinstance(r, (tuple, list)) and len(r) == 2 and r[0] == self.n):
                raise RuntimeError("group_read 응답이 예상과 다름: %r" % (r,))
            return True
        except Exception as e:          # 핀별 읽기로 물러선다
            self.notes.append("그룹 읽기 못 씀(%s) -> 핀별 읽기로 전환" % e)
            try:
                lgpio.group_free(self._h, self.dts[0])
            except Exception:           # 애초에 안 잡혔을 수 있다
                pass
            for p in self.dts:
                _ok(lgpio.gpio_claim_input(self._h, p, self._pull), "DT GPIO%d 확보" % p)
            return False

    def describe(self):
        how = "group_read" if self._grouped else "gpio_read x%d" % self.n
        if self.shared_sck:
            wiring = "SCK=GPIO%d(공유)  " % self.sck
            wiring += ", ".join("%s=GPIO%d" % (l, d)
                                for l, d in zip(self.labels, self.dts))
        else:
            wiring = "SCK=%s  " % ("group_write" if self._sck_grouped
                                   else "gpio_write x%d" % len(self.scks))
            wiring += ", ".join("%s=SCK GPIO%d/DT GPIO%d" % (l, s, d)
                                for l, s, d in zip(self.labels, self.scks, self.dts))
        head = ("HX711Bus  gpiochip%s  %s  gain=%d  읽기=%s"
                % (self.chip, wiring, self.gain, how))
        if self.invert:
            head += "  부호반전=%s" % ",".join(self.invert)
        return head + "".join("\n  ! " + n for n in self.notes)

    # --- 저수준 ----------------------------------------------------------
    def _read_bits(self):
        """DT 들을 한 번에 읽어 비트마스크로. 비트 i = dts[i]."""
        if self._grouped:
            return lgpio.group_read(self._h, self.dts[0])[1]
        bits = 0
        for i, p in enumerate(self.dts):
            bits |= lgpio.gpio_read(self._h, p) << i
        return bits

    def _pulse(self):
        """상승->하강을 붙여서 친다. 걸린 시간(=SCK HIGH 폭 상한)을 ns 로 돌려준다."""
        t0 = time.monotonic_ns()
        self._sck_write(1)
        self._sck_write(0)
        return time.monotonic_ns() - t0

    # --- 동기화 / 점검 ---------------------------------------------------
    def sync_reset(self, settle=0.5):
        """SCK 를 길게 HIGH 로 -> 전 모듈 동시 파워다운. 내리면 동시에 재시작.
        모든 칩의 변환 위상을 같은 순간으로 맞추는 용도. 시작할 때와
        타임아웃에서 복구할 때 쓴다."""
        self._sck_write(1)
        time.sleep(0.001)          # > 60us
        self._sck_write(0)
        time.sleep(settle)         # 파워업 + 첫 변환

    def present(self, timeout=0.6):
        """각 채널이 살아있는지. 풀업으로 잡아뒀으므로 모듈이 없거나 전원이 안 들어오면
        DT 가 계속 HIGH 로 뜬다. 한 번이라도 LOW 로 떨어지면 살아있는 것."""
        seen_low = [False] * self.n
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            bits = self._read_bits()
            for i in range(self.n):
                if not (bits >> i) & 1:
                    seen_low[i] = True
            if all(seen_low):
                break
            time.sleep(0.001)
        return seen_low

    def missing(self, timeout=0.6):
        return [self.labels[i] for i, ok in enumerate(self.present(timeout)) if not ok]

    # --- 읽기 ------------------------------------------------------------
    def ready(self):
        """지금 전 채널 DT 가 LOW(변환 끝)인가. 변환을 기다리는 동안 다른 센서를 읽고
        싶을 때 쓴다. True 를 본 뒤 read() 를 부르면 기다리지 않고 바로 클럭한다."""
        return self._read_bits() == 0

    def wait_ready(self, timeout=1.0, poll=0.0002):
        """모든 DT 가 LOW 가 되는 순간의 시각(ns)을 돌려준다.
        이 순간이 변환이 끝난 시각이고, 샘플의 타임스탬프로 쓸 값이다."""
        deadline = time.monotonic() + timeout
        while True:
            bits = self._read_bits()
            if bits == 0:
                return time.monotonic_ns()
            if time.monotonic() > deadline:
                raise HX711Timeout(self.labels[i] for i in range(self.n) if (bits >> i) & 1)
            time.sleep(poll)

    def read(self, timeout=1.0):
        """전 채널 동시 샘플 하나."""
        t_ns = self.wait_ready(timeout)

        vals = [0] * self.n
        max_high = 0
        for _ in range(24):
            high = self._pulse()
            if high > max_high:
                max_high = high
            bits = self._read_bits()
            for i in range(self.n):
                vals[i] = (vals[i] << 1) | ((bits >> i) & 1)

        for _ in range(self._gain_pulses):      # 다음 변환의 게인/채널 지정
            high = self._pulse()
            if high > max_high:
                max_high = high

        flags = []
        if max_high > POWERDOWN_NS:
            # 클럭 도중 선점당해 SCK 가 오래 HIGH -> 파워다운 가능, 이 샘플은 의심
            flags.append("sck_high")

        out = []
        for i, v in enumerate(vals):
            if v == 0xFFFFFF:
                flags.append("stuck_hi:" + self.labels[i])   # DT 가 계속 HIGH = 모듈 없음/죽음
            elif v == 0:
                flags.append("stuck_lo:" + self.labels[i])
            if v & 0x800000:
                v -= 0x1000000
            if abs(v) >= FULL_SCALE:
                flags.append("saturated:" + self.labels[i])  # 과부하 또는 셀 단선
            if i in self._neg:
                v = -v
            out.append(v)

        self.seq += 1
        return Sample(t_ns=t_ns, seq=self.seq, values=out,
                      max_high_ns=max_high, flags=flags)

    def close(self):
        try:
            if self._grouped:
                lgpio.group_free(self._h, self.dts[0])
            else:
                for p in self.dts:
                    lgpio.gpio_free(self._h, p)
        except Exception:               # 정리 중 오류는 무시
            pass
        try:
            if self._sck_grouped:
                lgpio.group_free(self._h, self.scks[0])
            else:
                for p in self.scks:
                    lgpio.gpio_free(self._h, p)
        except Exception:
            pass
        try:
            lgpio.gpiochip_close(self._h)
        except Exception:
            pass
