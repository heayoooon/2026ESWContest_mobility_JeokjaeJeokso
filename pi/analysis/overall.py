"""종합 위험도: 적재 · 무게중심 · 장력 판정의 단계(0 정상 · 1 주의 · 2 경고 · 3 위험)를 '안전 여유를 곱하는' 방식으로 합친다.
화물 이동은 넣지 않는다. 화물이 실제로 움직였을 때 화면에 따로 알린다 (Result.checks 의 key "move").

판정 하나가 한 단계마다 안전 여유를 1/4 씩 깎는다고 본다 (남은 여유: 정상 1, 주의 3/4, 경고 1/2, 위험 1/4).
판정마다 남은 여유를 곱한 것이 전체에 남은 여유이고, 그걸 다시 단계로 돌린다.
    종합 = 4 x (1 - (1 - 적재/4) x (1 - 무게중심/4) x (1 - 장력/4)) 의 정수 부분
평균을 내면 위험한 판정 하나가 정상 판정들에 묻힌다 (위험 + 정상 + 정상 -> 평균 1 = 주의). 곱하면
    한 판정만 정상이 아니면         그 단계 그대로          위험 하나 -> 위험
    주의 + 주의                     주의 (1.75)
    주의 + 주의 + 주의              경고 (2.31)             한 단계 올림
    경고 + 주의, 경고 + 주의 + 주의  경고 (2.5, 2.88)
    경고 + 경고                     위험 (3)                한 단계 올림
종합은 가장 나쁜 판정보다 낮아지지 않고, 겹쳐도 한 단계까지만 오른다.

장력은 벨트 중 단계가 높은 쪽 하나만 넣는다 (둘 다 풀렸다고 따로 곱하지 않는다).
판정마다 analyzer 의 Hold 로 거른 단계를 쓰므로(올릴 때는 이어져야 올리고, 내릴 때는 붙잡아 둔다) 종합은 따로 거르지 않는다.
"""
from analysis.risk import name

NOT_IN_RISK = ("move",)     # 종합 위험도에 안 넣는 판정 key. 화물 이동은 따로 알린다


def combine(levels):
    """판정 단계들 -> 종합 단계. 남은 여유가 1/4 의 곱이라 소수 오차 없이 정확하다."""
    left = 1.0                                  # 남은 안전 여유
    for lv in levels:
        left *= 1 - lv / 4
    return int(4 * (1 - left))


def overall(checks):
    """판정들(Hold 로 거른 것) -> (종합 단계, 이유). 들어갈 판정이 하나도 값이 없으면 (None, 이유)."""
    items = {}      # {항목: Check} 적재 · 무게중심 · 장력(벨트 중 단계가 높은 쪽)
    for ch in checks:
        if ch.key in NOT_IN_RISK or ch.level is None:
            continue
        item = ch.key if ch.key in ("load", "cog") else "tension"
        if item not in items or ch.level > items[item].level:
            items[item] = ch
    if not items:
        return None, "판정 불가"
    level = combine([ch.level for ch in items.values()])
    bad = sorted((ch for ch in items.values() if ch.level), key=lambda ch: -ch.level)
    text = " · ".join("%s %s" % (name(ch.level), ch.name) for ch in bad) or "전부 정상"
    if bad and level > bad[0].level:
        text += " (겹쳐서 한 단계 올림)"
    return level, text
