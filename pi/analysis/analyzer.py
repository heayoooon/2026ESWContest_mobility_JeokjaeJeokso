"""판정 단계의 입구: Frame 하나 -> Result 하나. 판정은 pi 에서 하고 display 는 결과를 그린다.

판정 4개 (analysis/risk.py). 적재 · 무게중심 · 화물 이동은 정상 / 주의 / 경고 / 위험, 장력은 정상 / 주의 / 위험.
기준값은 환경변수 (acquisition/config.py), 적재만 total.py 맨 위 상수
    적재        셀 합에서 벨트 누름을 뺀 화물 무게 -> 적정 적재중량의 %      total.py
    무게중심    판 가장자리에 얼마나 붙었나 COG_CAUTION / COG_WARNING / COG_DANGER   cog.py zone()
                (하중 이동을 뺀 무게중심으로 본다)
    화물 이동   주행 중 h 를 추정해 IMU 로 무게중심 변화를 예측, 실측과 비교  transfer.py
                (허용이 가속에 따라 커지고, 최근 구간 비율과 지속 시간으로 4단계)
    장력 <라벨> N쪽 · S쪽 무게로 정한 장력 하한과 비교                    tension.py
판정마다 Hold 로 깜빡임을 거르고(HOLD), 전체 위험도는 그중 가장 높은 것이다.

판정을 추가할 때는 Result 에 필드를 하나 더하고 Analyzer.__call__ 에서 채운다.
기록 · 전송 쪽(sink)은 Result 를 통째로 받으므로 함수 모양이 바뀌지 않는다.
시간 추세가 필요한 판정(예: 몇 초간 기울기 유지)은 상태를 Analyzer 에 둔다.
"""
from dataclasses import dataclass

from analysis.cog import CoG, cog, total_load, zone
from analysis.risk import Hold
from analysis.tension import TensionCheck
from analysis.total import TotalLoadCheck
from analysis.transfer import Transfer, TransferCheck

# 판정별 (on_s, hold_s): 이만큼 이어져야 알리고, 풀린 뒤에도 이만큼 알림을 유지한다
#   화물 이동은 transfer.py 가 최근 구간 비율로 이미 단발성 값을 걸러서 on_s 를 0 으로 둔다
HOLD = {"load": (2.0, 2.0), "cog": (0.3, 1.0), "move": (0.0, 5.0)}
HOLD_TENSION = (0.5, 2.0)


def check_list(cfg):
    """판정 목록 [(key, 이름)]. 설정으로 정해지고, CSV 판정 열 순서이기도 하다."""
    _, tension, corners = cfg.roles()
    out = [("load", "적재")]
    if corners:
        out += [("cog", "무게중심"), ("move", "화물 이동")]
    return out + [(t, "장력 " + t) for t in tension]


@dataclass
class Result:
    total_g: float | None       # 바닥 셀 합. 하나라도 값이 없으면 None
    cog: CoG | None             # 무게중심. 모서리 값이 없거나 하중이 작으면 None
    load_kg: float | None       # 적재 무게 = 셀 합에서 벨트 누름을 뺀 화물 무게. IMU az 로 나누고 평활
    measured_kg: float | None   # 측정 무게 = 셀 합 그대로 (벨트 누름 포함). az 로 나누고 평활
    transfer: Transfer | None   # IMU 로 예측한 무게중심 + 추정한 하중 이동 계수(h)
    tension_min: dict           # {장력 라벨: 그쪽 무게로 정한 장력 하한 N | None}
    checks: list                # [Check] Analyzer.checks 순서. 판정 못 하면 level None
    level: int | None           # 전체 위험도 = checks 중 가장 높은 것. 하나도 판정 못 하면 None
    changed: list               # [(Check, 이전 레벨)] 이번 프레임에 알림 레벨이 바뀐 판정


class Analyzer:
    def __init__(self, cfg):
        _, _, self.corners = cfg.roles()
        self.bed = (cfg.bed_l, cfg.bed_w) if cfg.bed_l else None
        self.imu = cfg.judge_imu or None
        self.zone = (cfg.cog_caution, cfg.cog_warning, cfg.cog_danger)
        self.load = TotalLoadCheck(cfg)
        self.transfer = TransferCheck(cfg, self.bed) if self.corners else None
        self.tension = TensionCheck(cfg)
        self.checks = check_list(cfg)
        self.holds = {k: Hold(*HOLD.get(k, HOLD_TENSION)) for k, _ in self.checks}
        self.last = {k: None for k, _ in self.checks}

    def __call__(self, frame):
        t = frame.t_s
        imu = frame.imu.get(self.imu) if self.imu else None
        total = total_load(frame)
        c = cog(frame, self.corners, self.bed)

        load_kg, measured_kg, check = self.load(t, frame, imu)
        checks = [check]
        tr = None
        if self.corners:
            tr, move = self.transfer(t, c, imu)
            # 위치 판정은 하중 이동(h x a)을 뺀 무게중심으로 한다. 급제동 순간의 쏠림에 놀라지 않게
            still = CoG(tr.static_x, tr.static_y, c.unit) if tr and c else c
            checks.append(zone(still, self.bed, *self.zone))
            checks.append(move)
        mins, more = self.tension(t, frame, imu)
        checks += more

        checks = [self.holds[ch.key](t, ch) for ch in checks]
        changed = []
        for ch in checks:
            old = self.last[ch.key]
            # 정상 <-> 판정 불가 는 알릴 거리가 아니다
            if ch.level != old and max(ch.level or 0, old or 0) > 0:
                changed.append((ch, old))
            self.last[ch.key] = ch.level
        level = max((ch.level for ch in checks if ch.level is not None), default=None)
        return Result(total_g=total, cog=c, load_kg=load_kg, measured_kg=measured_kg, transfer=tr,
                      tension_min=mins, checks=checks, level=level, changed=changed)
