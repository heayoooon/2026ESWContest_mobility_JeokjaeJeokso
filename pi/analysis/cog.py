"""바닥 하중 셀로 총하중과 무게중심을 낸다. Frame 만 본다 (실측 · sim · 기록 재생 공통).

좌표는 계획서 식을 따른다. 앞왼 셀이 원점, X 는 앞왼->앞오, Y 는 앞왼->뒤왼 방향.
    X = (FR + RR) / M * L        Y = (RL + RR) / M * W        M = 네 셀 합
L, W 는 셀 간격(mm, BED_L / BED_W). 안 주면 % 로 낸다 (50 = 가운데).

벨트는 판 아래(차체)에 걸려 있어서 화물을 k T 로 누르고(total.py), 그 힘은 누름 자리 (x, y) 에서 바닥 셀에 실린다.
그래서 셀로 잰 무게중심에는 벨트 누름이 섞여, 화물은 그대로인데 장력만 바꿔도 무게중심이 벨트 쪽으로 끌려간다.
위치를 아는 벨트(N · S)마다 누름을 그 자리에서 빼고 화물만의 무게중심을 낸다.
    Y = ((RL + RR) x W - Σ 누름 x 누름 y) / (M - Σ 누름)
    X = ((FR + RR) x L - Σ 누름 x 누름 x) / (M - Σ x 를 아는 누름)
누름 자리는 belt_fit.py 가 벨트를 조일 때의 셀 변화로 추정한다. 시작값은 y = BELT_N_Y / BELT_S_Y, x 는 모름이다.
x 를 모르는 벨트는 화물 무게중심 X 를 누른다고 보고 X 에서 안 뺀다 (X 를 안 옮긴다).
2026-10-25 기록에서 벨트 누름은 화물 가운데 X 가 아니라 앞 95mm / 뒤 53mm 에 실려, 조이면 X 도 8 ~ 32mm 끌려갔다.
위치를 모르는 벨트(세 번째부터)는 화물 무게중심을 누른다고 보고 안 뺀다 (무게중심을 안 옮긴다).
2026-10-15 전에는 누름을 안 빼서, 벨트를 조이거나 풀면 무게중심 판정 · 화물 이동 판정 · 화면 위치가 같이 움직였다.

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


def belt_press(frame, belts):
    """{장력 라벨: 그 벨트가 화물을 눌러 바닥 셀에 더한 무게 g}. belts = {장력 라벨: Belt} (belt_fit.py).
    느슨해 음수로 읽히면 0, 장력 값이 없어도 0 (못 빼서 셀 값 그대로 쓴다)."""
    out = {}
    for label, belt in belts.items():
        n = frame.tension_n.get(label)
        out[label] = belt.g_per_n * max(0.0, n) if n is not None else 0.0
    return out


def cog(frame, corners, bed=None, belts=()):
    """corners = (앞왼, 앞오, 뒤왼, 뒤오) 라벨 (Config.roles() 의 세 번째 값).
    bed = (L, W) mm. belts = [(벨트 누름 g, 누름 x 또는 None, 누름 y)] 위치를 아는 벨트마다. 위치는 무게중심과 같은 단위.
    모서리 값이 빠졌거나 화물(셀 합 - 누름)이 COG_MIN_G 미만이면 None."""
    if not corners:
        return None
    g = [frame.load_g.get(c) for c in corners]
    if any(v is None for v in g):
        return None
    fl, fr, rl, rr = g
    m = fl + fr + rl + rr
    cargo = m - sum(push for push, _, _ in belts)
    if cargo < COG_MIN_G:
        return None
    length, width = bed or (100.0, 100.0)
    # x 를 모르는 누름은 화물 무게중심 X 에 실리므로 X 모멘트 · 무게 양쪽에 남겨 둔다
    known_x = [(push, x) for push, x, _ in belts if x is not None]
    x = ((fr + rr) * length - sum(push * px for push, px in known_x)) / (m - sum(push for push, _ in known_x))
    y = ((rl + rr) * width - sum(push * py for push, _, py in belts)) / cargo
    return CoG(x, y, "mm" if bed else "%")


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
