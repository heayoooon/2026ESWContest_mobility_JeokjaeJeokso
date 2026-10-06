"""벨트 장력 판정. 벨트마다 그 벨트가 잡는 쪽 무게로 장력 하한을 정한다 (Config.belt_sides()).
HX_TENSION 첫 번째 = N(앞)쪽 벨트 -> 앞쪽 두 셀 무게, 두 번째 = S(뒤)쪽 벨트 -> 뒤쪽 두 셀 무게.

화물이 미끄러지지 않으려면 바닥 마찰이 관성력보다 커야 한다. 벨트를 화물 위로 감아 당기면
양쪽 끈이 T sinα 씩 화물을 눌러 마찰을 늘린다 (EN 12195-1 top-over lashing):
    μ (m g + 2 T sinα) >= m g A        ->        T >= m g (A - μ) / (2 μ sinα)
m = 그쪽 무게(kg), A = 버텨야 할 가감속(TENSION_A, G), μ = TENSION_MU, α = TENSION_ANGLE.
하한(최소 TENSION_FLOOR)보다 낮으면 주의, 절반보다 낮으면 위험, TENSION_MAX 를 넘으면 주의.
그쪽 무게가 SIDE_MIN_KG 보다 가벼우면 잡을 화물이 없으니 판정하지 않는다(정상).

그쪽 셀 무게에는 이 벨트가 화물을 누른 몫(장력 x 2 sinα, Config.belt_push_g_per_n())이 들어 있으므로 빼고 쓴다.
안 빼면 세게 조일수록 그쪽이 무거워져 하한도 같이 올라간다 (2026-10-04: S쪽 c2 9N 이면 1.9kg 무겁게 잡힘).

가감속 중에는 앞뒤 셀 무게가 하중 이동으로 출렁이므로 그쪽 무게는 길게(SIDE_TAU_S) 평활한다.
"""
from analysis.risk import CAUTION, DANGER, OK, Check, Ema, static_g

SIDE_TAU_S = 3.0        # 그쪽 무게 평활(초)
TENSION_TAU_S = 0.3     # 장력 평활(초)
SIDE_MIN_KG = 0.2       # 이보다 가벼우면 화물 없음


class TensionCheck:
    def __init__(self, cfg):
        self.sides = cfg.belt_sides()           # {장력 라벨: [바닥 셀 라벨]}
        self.per_kg = cfg.tension_per_kg()
        self.push_g_per_n = cfg.belt_push_g_per_n()   # 장력 1N 이 그쪽 셀 무게에 더하는 g
        self.floor, self.max = cfg.tension_floor, cfg.tension_max
        self.side_kg = {t: Ema(SIDE_TAU_S) for t in self.sides}
        self.tension = {t: Ema(TENSION_TAU_S) for t in self.sides}

    def __call__(self, t, frame, imu):
        """-> ({장력 라벨: 하한 N 또는 None}, [Check] 벨트마다)"""
        mins, checks = {}, []
        for label, cells in self.sides.items():
            g = [frame.load_g.get(c) for c in cells]
            now_n = frame.tension_n.get(label)      # 이번 프레임 장력(N). 없으면 누름을 못 빼고 무겁게 둔다
            push_g = self.push_g_per_n * max(0.0, now_n) if now_n is not None else 0.0
            side = None if any(v is None for v in g) else static_g(sum(g) - push_g, imu) / 1000
            kg = self.side_kg[label](t, side)
            tn = self.tension[label](t, now_n)
            name = "장력 " + label
            # 하한은 그쪽 무게로만 정해진다. 장력 셀 값이 없어도(보정 전) 하한은 낸다
            need = mins[label] = (max(self.floor, self.per_kg * kg)
                                  if kg is not None and kg >= SIDE_MIN_KG else None)
            if kg is None:
                checks.append(Check(label, name, None, "%s 값 없음" % "+".join(cells)))
                continue
            if tn is None:
                checks.append(Check(label, name, None, "장력 셀 값 없음"))
                continue
            where = "%s %.1fkg" % ("+".join(cells), kg)
            if tn > self.max:
                lv, text = CAUTION, "과조임 %.1fN > 상한 %gN" % (tn, self.max)
            elif need is None:
                lv, text = OK, "%.1fN (화물 없음)" % tn
            elif tn < need / 2:
                lv, text = DANGER, "풀림 %.1fN < 하한 %.1fN 의 절반 (%s)" % (tn, need, where)
            elif tn < need:
                lv, text = CAUTION, "부족 %.1fN < 하한 %.1fN (%s)" % (tn, need, where)
            else:
                lv, text = OK, "%.1fN (하한 %.1fN)" % (tn, need)
            checks.append(Check(label, name, lv, text))
        return mins, checks
