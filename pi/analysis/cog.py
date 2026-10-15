"""바닥 하중 셀로 총하중과 무게중심을 낸다. Frame 만 본다 (실측 · sim · 기록 재생 공통).

좌표는 계획서 식을 따른다. 앞왼 셀이 원점, X 는 앞왼->앞오, Y 는 앞왼->뒤왼 방향.
    X = (FR + RR) / M * L        Y = (RL + RR) / M * W        M = 네 셀 합
L, W 는 셀 간격(mm, BED_L / BED_W). 안 주면 % 로 낸다 (50 = 가운데).

벨트는 판 아래(차체)에 걸려 있어서 화물을 2 T sinα 로 누르고(total.py), 그 힘은 벨트 위치에서 바닥 셀에 실린다.
그래서 셀로 잰 무게중심에는 벨트 누름이 섞여, 화물은 그대로인데 장력만 바꿔도 무게중심이 벨트 쪽으로 끌려간다.
위치를 아는 벨트(N · S, BELT_N_Y / BELT_S_Y)마다 누름을 그 위치에서 빼고 화물만의 무게중심을 낸다.
    Y = ((RL + RR) x W - Σ 누름 x 벨트 위치) / (M - Σ 누름)
X 는 그대로 둔다. 벨트가 화물 위를 동서로 넘어가므로 누름이 화물 가운데 X 에 실려 X 를 옮기지 않는다.
위치를 모르는 벨트(세 번째부터)는 화물 무게중심을 누른다고 보고 안 뺀다 (무게중심을 안 옮긴다).
2026-10-15 전에는 누름을 안 빼서, 벨트를 조이거나 풀면 무게중심 판정 · 화물 이동 판정 · 화면 위치가 같이 움직였다.
BELT_N_Y = 0, BELT_S_Y = BED_W(기본값)이면 벨트가 앞뒤 셀 줄 바로 위에 있다고 보고 뺀다.

zone() 은 무게중심이 판 가장자리에 얼마나 붙었는지로 위험도 4단계를 낸다
(COG_CAUTION / COG_WARNING / COG_DANGER).
"""
from dataclasses import dataclass

from analysis.risk import CAUTION, DANGER, OK, WARNING, Check

COG_MIN_G = 100     # 화물(네 셀 합 - 벨트 누름)이 이보다 가벼우면 무게중심은 노이즈뿐이라 내지 않는다


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


def belt_press(frame, labels, g_per_n):
    """{장력 라벨: 그 벨트가 화물을 눌러 바닥 셀에 더한 무게 g}. g_per_n = Config.belt_push_g_per_n().
    느슨해 음수로 읽히면 0, 장력 값이 없어도 0 (못 빼서 셀 값 그대로 쓴다)."""
    out = {}
    for label in labels:
        n = frame.tension_n.get(label)
        out[label] = g_per_n * max(0.0, n) if n is not None else 0.0
    return out


def cog(frame, corners, bed=None, belts=()):
    """corners = (앞왼, 앞오, 뒤왼, 뒤오) 라벨 (Config.roles() 의 세 번째 값).
    bed = (L, W) mm. belts = [(벨트 누름 g, 벨트 남북 위치)] 위치를 아는 벨트마다. 위치는 무게중심 Y 와 같은 단위.
    모서리 값이 빠졌거나 화물(셀 합 - 누름)이 COG_MIN_G 미만이면 None."""
    if not corners:
        return None
    g = [frame.load_g.get(c) for c in corners]
    if any(v is None for v in g):
        return None
    fl, fr, rl, rr = g
    m = fl + fr + rl + rr
    cargo = m - sum(push for push, _ in belts)
    if cargo < COG_MIN_G:
        return None
    length, width = bed or (100.0, 100.0)
    y = ((rl + rr) * width - sum(push * belt_y for push, belt_y in belts)) / cargo
    return CoG((fr + rr) / m * length, y, "mm" if bed else "%")


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
