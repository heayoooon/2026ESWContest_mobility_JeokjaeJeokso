"""바닥 하중 셀로 총하중과 무게중심을 낸다. Frame 만 본다 (실측 · sim · 기록 재생 공통).

좌표는 계획서 식을 따른다. 앞왼 셀이 원점, X 는 앞왼->앞오, Y 는 앞왼->뒤왼 방향.
    X = (FR + RR) / M * L        Y = (RL + RR) / M * W        M = 네 셀 합
L, W 는 셀 간격(mm, BED_L / BED_W). 안 주면 % 로 낸다 (50 = 가운데).
"""
from dataclasses import dataclass

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
