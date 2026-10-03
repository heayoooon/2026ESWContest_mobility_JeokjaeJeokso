"""판정 단계의 입구: Frame 하나 -> Result 하나. 판정은 pi 에서 하고 display 는 결과를 그린다.

판정을 추가할 때는 Result 에 필드를 하나 더하고 Analyzer.__call__ 에서 채운다.
기록 · 전송 쪽(sink)은 Result 를 통째로 받으므로 함수 모양이 바뀌지 않는다.
시간 추세가 필요한 판정(예: 몇 초간 기울기 유지)은 상태를 Analyzer 에 둔다.
"""
from dataclasses import dataclass

from analysis.cog import CoG, cog, total_load


@dataclass
class Result:
    total_g: float | None       # 바닥 셀 합. 하나라도 값이 없으면 None
    cog: CoG | None             # 무게중심. 모서리 값이 없거나 하중이 작으면 None


class Analyzer:
    def __init__(self, cfg):
        _, _, self.corners = cfg.roles()
        self.bed = (cfg.bed_l, cfg.bed_w) if cfg.bed_l else None

    def __call__(self, frame):
        return Result(total_g=total_load(frame), cog=cog(frame, self.corners, self.bed))
