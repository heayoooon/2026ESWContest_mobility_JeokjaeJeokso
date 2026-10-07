"""바닥 하중 셀로 총하중과 무게중심을 낸다. Frame 만 본다 (실측 · sim · 기록 재생 공통).

좌표는 계획서 식을 따른다. 앞왼 셀이 원점, X 는 앞왼->앞오, Y 는 앞왼->뒤왼 방향.
    X = (FR + RR) / M * L        Y = (RL + RR) / M * W        M = 네 셀 합
L, W 는 셀 간격(mm, BED_L / BED_W). 안 주면 % 로 낸다 (50 = 가운데).

zone() 은 무게중심이 판 가장자리에 얼마나 붙었는지로 위험도 4단계를 낸다
(COG_CAUTION / COG_WARNING / COG_DANGER).
"""
from dataclasses import dataclass

from analysis.risk import CAUTION, DANGER, OK, WARNING, Check

COG_MIN_G = 100     # 네 셀 합이 이보다 작으면 무게중심은 노이즈뿐이라 내지 않는다


@dataclass
class CoG:
    x: float
    y: float
    unit: str       # "mm" | "%"


def total_load(frame):
    """바닥 셀 합(g). 하나라도 값이 없으면 합이 틀리므로 None."""
    vals = list(frame.load_g.values())
    if not vals or any(v is None for v in vals):
        return None
    return sum(vals)


def cog(frame, corners, bed=None):
    """corners = (앞왼, 앞오, 뒤왼, 뒤오) 라벨 (Config.roles() 의 세 번째 값).
    bed = (L, W) mm. 모서리 값이 빠졌거나 하중이 COG_MIN_G 미만이면 None."""
    if not corners:
        return None
    g = [frame.load_g.get(c) for c in corners]
    if any(v is None for v in g):
        return None
    fl, fr, rl, rr = g
    m = fl + fr + rl + rr
    if m < COG_MIN_G:
        return None
    length, width = bed or (100.0, 100.0)
    return CoG((fr + rr) / m * length, (rl + rr) / m * width, "mm" if bed else "%")


def zone(c, bed, caution, warning, danger):
    """무게중심이 가장 가까운 가장자리에서 셀 간격의 몇 % 안쪽인지 -> Check.
    caution 보다 가까우면 주의, warning 보다 가까우면 경고, danger 보다 가까우면 위험
    (셀 사각형 밖이면 음수라 당연히 위험). 0 <= danger < warning < caution < 50."""
    if c is None:
        return Check("cog", "무게중심", None, "값 없음")
    length, width = bed or (100.0, 100.0)
    fx, fy = 100 * c.x / length, 100 * c.y / width
    margin, side = min((fx, "왼쪽"), (100 - fx, "오른쪽"), (fy, "앞쪽"), (100 - fy, "뒤쪽"))
    where = "X %.0f%% Y %.0f%%" % (fx, fy)
    if margin < danger:
        return Check("cog", "무게중심", DANGER, "%s 끝 %.0f%% (%s)" % (side, margin, where))
    if margin < warning:
        return Check("cog", "무게중심", WARNING, "%s 많이 쏠림 %.0f%% (%s)" % (side, margin, where))
    if margin < caution:
        return Check("cog", "무게중심", CAUTION, "%s 쏠림 %.0f%% (%s)" % (side, margin, where))
    return Check("cog", "무게중심", OK, where)
