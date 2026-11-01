"""판정 단계의 입구: Frame 하나 -> Result 하나. 판정은 pi 에서 하고 display 는 결과를 그린다.

판정 4개 (analysis/risk.py). 넷 다 정상 / 주의 / 경고 / 위험.
기준값은 환경변수 (acquisition/config.py), 적재 · 장력 단계 기준만 total.py · tension.py 맨 위 상수
    적재        셀 합에서 벨트 누름을 뺀 화물 무게 -> 적정 적재중량의 %      total.py
    무게중심    판 가장자리에 얼마나 붙었나 COG_CAUTION / COG_WARNING / COG_DANGER   cog.py zone()
                (벨트 누름과 하중 이동을 뺀 무게중심으로 본다)
    화물 이동   주행 중 h 를 추정해 IMU 로 무게중심 변화를 예측, 실측과 비교  transfer.py
                (허용이 가속에 따라 커지고, 최근 구간 비율과 지속 시간으로 4단계)
                경고 이상으로 확정되면 사람이 기준 재설정을 누를 때까지 붙잡는다 (MOVE_LATCH=1, 아래 _latch_move)
    장력 N / S  N쪽 · S쪽 벨트마다 맡은 무게(벨트 위치로 나눈 몫)로 정한 하한의 % · 상한의 %    tension.py
판정마다 Hold 로 깜빡임을 거른다(HOLD).
무게중심(Result.cog)은 벨트 누름을 벨트 위치에서 뺀 화물 무게중심이다 (cog.py). 무게중심 · 화물 이동 · 장력 판정과
화면 위치가 다 이 값을 쓴다. 그래서 화물은 그대로인데 장력만 바뀌면 무게중심이 안 움직인다.
벨트마다 누름 계수 k 와 누름 자리 (x, y) 는 정차 중 벨트를 조일 때의 셀 변화로 추정한다 (belt_fit.py, Result.belts).
적재 · 무게중심 · 장력 판정이 다 이 추정값으로 누름을 뺀다.

적재 · 주행 모드 (set_mode): 판정은 두 모드가 같다. 적재 모드에서만 벨트 k, x, y 를 추정하고, 주행으로 바꾸는 순간
그 값을 확정해 주행 내내 쓰며 화물 이동 기준을 다시 잡는다 (rebase). 처음은 적재 모드다.

종합 위험도(Result.level)는 적재 · 무게중심 · 장력의 단계(0~3)를 안전 여유 곱으로 합친 것이다 (overall.py).
판정 하나가 위험이면 종합도 위험이고, 여러 판정이 같이 나쁘면 한 단계 위로 올라간다 (경고 둘 -> 위험, 주의 셋 -> 경고).
화물 이동은 종합에 안 넣고 따로 알린다 (checks 의 "move").

판정을 추가할 때는 Result 에 필드를 하나 더하고 Analyzer.__call__ 에서 채운다.
기록 · 전송 쪽(sink)은 Result 를 통째로 받으므로 함수 모양이 바뀌지 않는다.
시간 추세가 필요한 판정(예: 몇 초간 기울기 유지)은 상태를 Analyzer 에 둔다.
"""
from dataclasses import dataclass

from analysis.belt_fit import BeltFit
from analysis.cog import CoG, belt_press, cog, total_load, zone
from analysis.overall import overall
from analysis.risk import WARNING, Check, Hold, name as level_name
from analysis.tension import TensionCheck, check_name
from analysis.total import TotalLoadCheck
from analysis.transfer import Transfer, TransferCheck

# 판정별 (on_s, hold_s): 이만큼 이어져야 알리고, 풀린 뒤에도 이만큼 알림을 유지한다
#   화물 이동은 transfer.py 가 최근 구간 비율로 이미 단발성 값을 걸러서 on_s 를 0 으로 둔다
HOLD = {"load": (2.0, 2.0), "cog": (0.3, 1.0), "move": (0.0, 8.0)}
HOLD_TENSION = (0.5, 2.0)
LATCH_DIR_MIN = 5.0     # 붙잡은 경보 문구에 방향을 적는 최소 이동량 (CoG 단위, mm). 이보다 작으면 원위치로 본다


def check_list(cfg):
    """판정 목록 [(key, 이름)]. 설정으로 정해지고, CSV 판정 열 순서이기도 하다."""
    _, _, corners = cfg.roles()
    out = [("load", "적재")]
    if corners:
        out += [("cog", "무게중심"), ("move", "화물 이동")]
    return out + [(t, check_name(side, t)) for t, (side, _) in cfg.belt_sides().items()]


@dataclass
class Result:
    total_g: float | None       # 바닥 셀 합. 하나라도 값이 없으면 None
    cog: CoG | None             # 화물 무게중심 = 벨트 누름을 벨트 위치에서 뺀 값. 모서리 값이 없거나 하중이 작으면 None
    load_kg: float | None       # 적재 무게 = 셀 합에서 벨트 누름을 뺀 화물 무게. IMU az 로 나누고 평활
    measured_kg: float | None   # 측정 무게 = 셀 합 그대로 (벨트 누름 포함). az 로 나누고 평활
    transfer: Transfer | None   # IMU 로 예측한 무게중심 + 추정한 하중 이동 계수(h)
    tension_min: dict           # {장력 라벨: 그 벨트가 맡은 무게로 정한 장력 하한 N | None}
    belts: dict                 # {장력 라벨: Belt} 벨트마다 추정한 누름 계수 k · 누름 자리 x, y (belt_fit.py)
    checks: list                # [Check] Analyzer.checks 순서. 판정 못 하면 level None
    level: int | None           # 종합 위험도 = 적재 · 무게중심 · 장력 단계의 안전 여유 곱 (overall.py).
                                #   화물 이동은 안 들어간다. 하나도 판정 못 하면 None
    changed: list               # [(Check, 이전 레벨)] 이번 프레임에 알림 레벨이 바뀐 판정.
                                #   종합 위험도가 바뀌면 key "risk" 인 Check 가 맨 뒤에 붙는다


class Analyzer:
    def __init__(self, cfg):
        _, _, self.corners = cfg.roles()
        self.bed = (cfg.bed_l, cfg.bed_w) if cfg.bed_l else None
        self.imu = cfg.judge_imu or None
        # 벨트마다 누름 계수 · 자리 추정. 위치를 아는 벨트(N · S)는 무게중심에서 그 누름을 그 자리로 뺀다
        self.belt_fit = BeltFit(cfg, self.bed)
        self.zone = (cfg.cog_caution, cfg.cog_warning, cfg.cog_danger)
        self.load = TotalLoadCheck(cfg)
        self.transfer = TransferCheck(cfg, self.bed) if self.corners else None
        self.tension = TensionCheck(cfg)
        self.checks = check_list(cfg)
        self.holds = {k: Hold(*HOLD.get(k, HOLD_TENSION)) for k, _ in self.checks}
        self.last = {k: None for k, _ in self.checks}
        self.last_level = None                  # 지난 프레임 종합 위험도
        self.rebase_n = 0                       # 기준 재설정을 받은 횟수
        self.move_latch_on = bool(cfg.move_latch)
        self.move_latch = None                  # 붙잡고 있는 화물 이동 최고 단계 (경고 이상). 기준 재설정이 지운다
        self.mode = "load"                      # 적재 / 주행 (set_mode). 벨트 추정과 화물 이동 새 자리 받아들이기가 다르다
        self._spot_switch()

    def _spot_switch(self):
        """주행 모드에서는 이동한 화물의 새 자리를 기준으로 받아들이지 않는다 (MOVE_LATCH=1). 적재 모드는 싣고 내리는 중이라 받아들인다.
        MOVE_LATCH=0 이면 모드와 상관없이 예전처럼 받아들인다."""
        if self.transfer:
            self.transfer.accept_new_spot = self.mode == "load" or not self.move_latch_on

    def set_mode(self, mode):
        """적재("load") / 주행("drive"). 바뀌었으면 True.
        적재 -> 주행: 벨트 k, x, y 를 지금 값으로 확정 · 고정하고(belt_fit.freeze) 화물 이동 기준을 다시 잡는다(rebase).
        주행 -> 적재: 벨트 추정을 다시 켠다(belt_fit.thaw). 경보음을 끄는 것은 화면 쪽 일이다."""
        if mode not in ("load", "drive"):
            raise ValueError("mode 는 load 또는 drive: %r" % mode)
        if mode == self.mode:
            return False
        if mode == "drive":
            self.belt_fit.freeze()
            self.rebase()
        else:
            self.belt_fit.thaw()
        self.mode = mode
        self._spot_switch()
        return True

    def rebase(self):
        """기준 재설정: 지금 상태를 정상 기준으로 삼는다. 화물 이동 판정(기준 위치, 학습한 h, 붙잡고 있던 알림)만
        지우고 직후 몇 초는 화물이 자리 잡을 때까지 판정을 보류한다 (transfer.py REBASE_MUTE_S).
        과적 · 무게중심 쏠림 · 장력 풀림은 지금 상태를 그대로 보는 판정이라 건드리지 않는다. 재설정이 진짜 위험을 가려서는 안 된다."""
        if self.transfer:
            self.transfer.rebase()
            self.holds["move"] = Hold(*HOLD.get("move", HOLD_TENSION))     # 붙잡고 있던 화물 이동 알림을 지운다
        self.move_latch = None                                              # 확정돼 붙잡던 경보도 지운다 (이게 해제 수단이다)
        self.rebase_n += 1

    def _latch_move(self, move, c, tr):
        """화물 이동이 경고 이상으로 확정되면 그 최고 단계를 붙잡는다. 판정이 정상 · 주의로 돌아오거나(화물이 원위치로 돌아옴),
        무게중심이 없어져도(화물이 판에서 떨어짐) 사람이 기준 재설정을 누를 때까지 유지한다.
        주의는 한 번의 흔들림일 수 있어 붙잡지 않는다. 종합 위험도에는 원래 화물 이동이 안 들어간다.
        단계는 붙잡아도 이동량은 지금 값을 보여 준다: 디스플레이가 문구에서 방향과 mm 를 읽어 화면에 그리므로,
        판정이 낮아진 동안의 문구를 '지금 잔차(실측 - 예측)'로 만든다. 주행 모드에서는 기준이 원위치에 남아 있어
        이 잔차가 곧 원위치 대비 이동량이고, 화물이 돌아오면 0 에 가까워진다."""
        lv = move.level
        if lv is not None and lv >= WARNING:
            self.move_latch = max(self.move_latch or 0, lv)
        if self.move_latch is None or (lv is not None and lv >= self.move_latch):
            return move
        now = "지금 %s" % level_name(lv) if lv is not None else "지금 판정 불가"
        tail = "[유지: 최고 %s, %s - 확인 후 기준 재설정으로 해제]" % (level_name(self.move_latch), now)
        if c is None or tr is None:                                  # 화물이 판에서 사라졌거나 IMU 가 없다: 숫자는 없다
            return Check(move.key, move.name, self.move_latch, "화물 이동 감지됨 %s" % tail)
        rx, ry = c.x - tr.pred_x, c.y - tr.pred_y
        mm = max(abs(rx), abs(ry))
        if mm < LATCH_DIR_MIN:                                       # 원위치 근처에서는 방향을 말하지 않는다
            where = ""
        elif abs(rx) >= abs(ry):
            where = ("오른쪽 " if rx > 0 else "왼쪽 ")
        else:
            where = ("뒤쪽 " if ry > 0 else "앞쪽 ")
        return Check(move.key, move.name, self.move_latch,
                     "가감속으로 설명 안 되는 이동 %s%.0f%s %s" % (where, mm, c.unit, tail))

    def __call__(self, frame):
        t = frame.t_s
        imu = frame.imu.get(self.imu) if self.imu else None
        total = total_load(frame)
        belts = self.belt_fit(t, frame, imu)
        located = {label: b for label, b in belts.items() if b.y is not None}
        push = belt_press(frame, located)
        c = cog(frame, self.corners, self.bed, [(push[label], b.x, b.y) for label, b in located.items()])

        load_kg, measured_kg, check = self.load(t, frame, imu, belts)
        checks = [check]
        tr = None
        if self.corners:
            tr, move = self.transfer(t, c, imu)
            if self.move_latch_on:
                move = self._latch_move(move, c, tr)
            # 위치 판정은 하중 이동(h x a)을 뺀 무게중심으로 한다. 급제동 순간의 쏠림에 놀라지 않게
            still = CoG(tr.static_x, tr.static_y, c.unit) if tr and c else c
            checks.append(zone(still, self.bed, *self.zone))
            checks.append(move)
        mins, more = self.tension(t, frame, imu, c, belts)
        checks += more

        checks = [self.holds[ch.key](t, ch) for ch in checks]
        changed = []
        for ch in checks:
            old = self.last[ch.key]
            # 정상 <-> 판정 불가 는 알릴 거리가 아니다
            if ch.level != old and max(ch.level or 0, old or 0) > 0:
                changed.append((ch, old))
            self.last[ch.key] = ch.level
        level, text = overall(checks)
        old = self.last_level
        if level != old and max(level or 0, old or 0) > 0:
            changed.append((Check("risk", "종합 위험도", level, text), old))
        self.last_level = level
        return Result(total_g=total, cog=c, load_kg=load_kg, measured_kg=measured_kg, transfer=tr,
                      tension_min=mins, belts=belts, checks=checks, level=level, changed=changed)
