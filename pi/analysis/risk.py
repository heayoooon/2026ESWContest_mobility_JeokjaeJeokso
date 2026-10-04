"""위험도 3단계와 판정 하나의 결과. 판정 모듈(load / cog / transfer / tension)이 같이 쓴다.

판정마다 Check 하나를 낸다. 값이 없어 판정을 못 하면 level 이 None 이다.
한 프레임만 튀고 마는 값에 알림이 깜빡이지 않게 Hold 로 거른다.
판정 모듈이 같이 쓰는 평활(Ema)과 위아래 출렁임 빼기(static_g)도 여기 둔다.
"""
import math
from dataclasses import dataclass

OK, CAUTION, DANGER = 0, 1, 2
NAMES = ("정상", "주의", "위험")


def name(level):
    return "-" if level is None else NAMES[level]


@dataclass
class Check:
    key: str            # CSV 열 이름용 (load, cog, move, <장력 라벨>)
    name: str           # 화면 · 엑셀용 (적재, 무게중심, 화물 이동, 장력 T1)
    level: int | None   # OK | CAUTION | DANGER. 판정 못 하면 None
    text: str           # 이유. 정상이면 지금 값


class Hold:
    """레벨이 on_s 초 이어져야 올리고, 내려간 뒤에도 hold_s 초 동안 붙잡아 둔다.
    주의 -> 위험은 따로 센다 (주의가 이어지는 중에 위험이 잠깐 튀면 주의만 올라간다)."""

    def __init__(self, on_s, hold_s):
        self.on_s, self.hold_s = on_s, hold_s
        self.since = {CAUTION: None, DANGER: None}      # 원래 레벨이 k 이상으로 이어진 시작 시각
        self.until = {CAUTION: -1e18, DANGER: -1e18}    # k 를 언제까지 붙잡을지
        self.text = {CAUTION: "", DANGER: ""}           # k 일 때 마지막 이유

    def __call__(self, t, check):
        lv = check.level
        for k in (CAUTION, DANGER):
            if lv is not None and lv >= k:
                if self.since[k] is None:
                    self.since[k] = t
                if t - self.since[k] >= self.on_s:
                    self.until[k] = t + self.hold_s
                    self.text[k] = check.text
            else:
                self.since[k] = None
        held = max([k for k in (CAUTION, DANGER) if self.until[k] > t], default=OK)
        if lv == held or (lv is None and held == OK):
            return check
        # 붙잡는 중이면 그때 이유를, 더 높은 레벨이 아직 on_s 가 안 찼으면 지금 이유를 낮은 레벨로
        text = self.text[held] if lv is None or lv < held else check.text + " (확인 중)"
        return Check(check.key, check.name, held, text)


class Ema:
    """시정수 tau_s 초의 지수 평활. 프레임 간격이 들쭉날쭉해도 시간으로 맞춘다. 값이 None 이면 None."""

    def __init__(self, tau_s):
        self.tau = tau_s
        self.v = self.t = None

    def __call__(self, t, x):
        if x is None:
            return None
        if self.v is None:
            self.v = x
        else:
            self.v += (1 - math.exp(-max(0.0, t - self.t) / self.tau)) * (x - self.v)
        self.t = t
        return self.v


def static_g(g, imu):
    """셀이 받은 무게(g)를 정지 상태 무게로. 차가 위아래로 출렁이면 셀 값도 az(G) 배로 출렁이므로 az 로 나눈다.
    IMU 가 없거나 az 가 엉뚱하면(충격 · 점프) 그대로 둔다."""
    if g is None or imu is None or not 0.5 < imu.az < 1.5:
        return g
    return g / imu.az
