"""적재 안정성 위험 등급: 화물 이동(motion_comp 의 MotionResult) + 벨트 장력 -> 정상/주의/경고/위험.

기준값은 전부 RiskParams 에 모았다. 지금 값은 모형 대차 규모로 잡은 **임의의 초기값**이다
(EN 12195-1 로 산출한 최소 필요 장력이 나오면 t_min_n, 실측이 쌓이면 나머지를 교체한다).

등급 표 (행 = 화물 이동 상태, 열 = 벨트 상태)

                    장력 정상   약간 이완   많이 이완   불량·미체결
  이동 없음            0          1          2          2
  이동 의심            1          2          2          3
  이동 확정            2          3          3          3

  이동 의심 = 슬립 플래그가 켜진 지 t_confirm_s 미만 (급제동 보상 오차일 수 있다)
  이동 확정 = t_confirm_s 이상 지속. 확정에 잔차 R 이 r_danger_cm 이상이면 장력과 무관하게 3.
  벨트 상태는 가장 나쁜 벨트 기준. 이완 속도가 빠르면 한 단계 올린다 (예측).
등급은 올라갈 때는 즉시, 내려갈 때는 hold_s 동안 계속 낮을 때만 내려간다.
"""
from collections import deque
from dataclasses import dataclass, field

import numpy as np

LEVELS = ("정상", "주의", "경고", "위험")
BELT_NAMES = ("정상", "약간 이완", "많이 이완", "불량")

# [이동 없음, 이동 의심, 이동 확정][벨트 상태]
_TABLE = (
    (0, 1, 2, 2),
    (1, 2, 2, 3),
    (2, 3, 3, 3),
)
SLIP_NAMES = ("없음", "의심", "확정")


@dataclass
class RiskParams:
    # 벨트 장력 (N). 임의값
    unfastened_n: float = 1.0       # 이보다 작으면 미체결 / 풀림 (벨트 없는 런 c1 0.02~0.4 N)
    t_min_n: float = 3.0            # 최소 필요 장력. 조인 상태로 보이는 런이 3.4~16 N 이라 잠정값 (EN 12195-1 계산값으로 교체)
    dt_ratio_pct: tuple = (10.0, 25.0, 50.0)   # 기준 장력 대비 감소율 경계 (약간/많이/불량)
    loosen_rate_pct_s: float = 15.0  # 감소율 증가 속도가 이 이상이면 벨트 상태를 한 단계 올림 (손으로 기울인 런이 최대 약 10 %/s)
    fault_n: float = -1.0           # 이보다 작은 장력은 센서 이상(stuck_lo 런 -4.6 N)이라 벨트 상태로 안 본다
    base_n: int = 20                # 기준 장력 평균 샘플 수 (10Hz 에서 2초)
    tension_win: int = 10           # 장력 평활 창 (10Hz 에서 1초). 주행 진동을 눌러준다
    # 화물 이동
    t_confirm_s: float = 1.5        # 슬립 플래그 지속이 이 이상이면 이동 확정
    r_danger_cm: float = 8.0        # 확정 + 잔차가 이 이상이면 곧바로 위험 (손으로 거칠게 기울여도 보상 후 최대 5.2 cm)
    # 등급 안정화
    hold_s: float = 3.0


@dataclass
class RiskResult:
    level: int                      # 0 정상, 1 주의, 2 경고, 3 위험
    name: str
    slip_state: str                 # 없음 / 의심 / 확정
    belt_state: str | None          # 가장 나쁜 벨트 기준. 판단 불가면 None
    worst_belt: str | None
    tension_n: dict = field(default_factory=dict)       # 평활한 현재 장력 {라벨: N | None}
    base_n: dict = field(default_factory=dict)          # 기준 장력 {라벨: N | None}
    dt_ratio_pct: dict = field(default_factory=dict)    # 감소율 {라벨: % | None}
    reasons: list = field(default_factory=list)


class RiskAssessor:
    def __init__(self, params=None):
        self.p = params or RiskParams()
        self.reset()

    def reset(self):
        """기준 장력부터 다시. 벨트를 다시 조였을 때 부른다."""
        self._base = {}             # 라벨 -> deque (기준 모으는 중) 또는 float (확정)
        self._hist = {}             # 라벨 -> deque 장력
        self._ratio_hist = {}       # 라벨 -> deque (t, 감소율%)
        self.level = 0
        self._low_since = None

    def set_baseline(self, base):
        """출발 전 기준 장력을 직접 준다 ({라벨: N}). 주면 자동 수집은 그 라벨에 대해 멈춘다."""
        for k, v in base.items():
            self._base[k] = float(v)

    # ------------------------------------------------------------------
    def update(self, t_s, motion, tension):
        """motion = MotionResult 또는 None(이동 판정 없이 장력만), tension = Frame.tension_n."""
        p = self.p
        reasons = []

        freezing = motion is None or motion.state != "rest"
        smooth, base, ratio = {}, {}, {}
        states = {}
        for lab, v in tension.items():
            if v is not None and v < p.fault_n:
                reasons.append("%s 장력 센서 이상 (%.1f N)" % (lab, v))
                v = None
            smooth[lab] = self._smooth(lab, v)
            base[lab] = self._baseline(lab, smooth[lab], v, freezing)
            ratio[lab] = self._ratio(lab, t_s, smooth[lab], base[lab])
            states[lab] = self._belt_state(lab, smooth[lab], base[lab], ratio[lab], t_s, reasons)

        known = {k: s for k, s in states.items() if s is not None}
        if known:
            worst = max(known, key=known.get)
            belt = known[worst]
        else:
            worst, belt = None, None
            reasons.append("장력 데이터 없음")

        slip = self._slip_state(motion, reasons)
        cand = _TABLE[slip][belt if belt is not None else 0]
        if slip == 2 and motion.r is not None and motion.r >= p.r_danger_cm:
            cand = 3
            reasons.append("화물 이동량 %.1f cm >= %.1f cm" % (motion.r, p.r_danger_cm))

        level = self._stabilize(t_s, cand)
        return RiskResult(
            level=level, name=LEVELS[level], slip_state=SLIP_NAMES[slip],
            belt_state=BELT_NAMES[belt] if belt is not None else None, worst_belt=worst,
            tension_n=smooth, base_n={k: (v if not isinstance(v, deque) else None) for k, v in self._base.items()},
            dt_ratio_pct=ratio, reasons=reasons)

    # ------------------------------------------------------------------
    def _smooth(self, lab, v):
        h = self._hist.setdefault(lab, deque(maxlen=self.p.tension_win))
        if v is not None:
            h.append(v)
        return float(np.mean(h)) if h else None

    def _baseline(self, lab, cur, raw, freezing):
        b = self._base.get(lab)
        if isinstance(b, float):
            return b
        if b is None:
            b = self._base[lab] = deque(maxlen=self.p.base_n)
        if raw is not None:
            b.append(raw)
        if len(b) >= self.p.base_n and freezing:
            self._base[lab] = float(np.mean(b))
            return self._base[lab]
        return float(np.mean(b)) if len(b) else None

    def _ratio(self, lab, t_s, cur, base):
        if cur is None or base is None or base <= 0:
            return None
        r = (base - cur) / base * 100.0
        h = self._ratio_hist.setdefault(lab, deque(maxlen=self.p.tension_win))
        h.append((t_s, r))
        return r

    def _rate(self, lab):
        h = self._ratio_hist.get(lab)
        if not h or len(h) < 5:
            return 0.0
        t, r = np.array(h).T
        if t[-1] - t[0] <= 0:
            return 0.0
        return float(np.polyfit(t, r, 1)[0])

    def _belt_state(self, lab, cur, base, ratio, t_s, reasons):
        p = self.p
        if cur is None:
            return None
        if cur < p.unfastened_n:
            reasons.append("%s 벨트 미체결/풀림 (%.1f N)" % (lab, cur))
            return 3
        if base is None or ratio is None:
            return None
        if isinstance(self._base.get(lab), deque):
            return 0        # 기준을 아직 모으는 중
        if base < p.unfastened_n:
            reasons.append("%s 벨트 기준 장력이 너무 낮음 (%.1f N)" % (lab, base))
            return 3
        a, b, c = p.dt_ratio_pct
        s = 0 if ratio < a else 1 if ratio < b else 2 if ratio < c else 3
        if cur < p.t_min_n and s < 2:
            s = 2
            reasons.append("%s 벨트 장력 %.1f N < 최소 필요 %.1f N" % (lab, cur, p.t_min_n))
        rate = self._rate(lab)
        if s >= 1 and s < 3 and rate >= p.loosen_rate_pct_s:
            s += 1
            reasons.append("%s 벨트 빠르게 이완 중 (%.1f %%/s)" % (lab, rate))
        if s >= 1:
            reasons.append("%s 벨트 장력 %.0f%% 감소" % (lab, ratio))
        return s

    def _slip_state(self, motion, reasons):
        if motion is None or motion.state != "ready" or not motion.flag_slip:
            return 0
        if motion.t_slip_s >= self.p.t_confirm_s:
            reasons.append("화물 이동 확정 (%.1f초 지속)" % motion.t_slip_s)
            return 2
        reasons.append("화물 이동 의심")
        return 1

    def _stabilize(self, t_s, cand):
        if cand >= self.level:
            self.level = cand
            self._low_since = None
        else:
            if self._low_since is None:
                self._low_since = t_s
            if t_s - self._low_since >= self.p.hold_s:
                self.level = cand
                self._low_since = None
        return self.level
