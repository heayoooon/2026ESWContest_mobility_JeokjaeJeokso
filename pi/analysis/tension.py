"""벨트 장력 판정. N(앞)쪽 벨트와 S(뒤)쪽 벨트 2개를 따로 판정한다.

벨트 배치는 HX_TENSION 순서로 정한다 (Config.belt_sides()).
    첫 번째 장력 셀 = N쪽 벨트 -> 앞쪽 두 바닥 셀(COG_CORNERS 1 · 2번, NW+NE) 무게를 맡는다
    두 번째 장력 셀 = S쪽 벨트 -> 뒤쪽 두 바닥 셀(COG_CORNERS 3 · 4번, SW+SE) 무게를 맡는다
화물이 앞으로 쏠리면 N쪽 무게가 늘어 N쪽 벨트의 하한이 올라간다.

하한 = EN 12195-1:2010 식 10 (위로 감기 top-over lashing) 을 장력으로 푼 것.
화물이 미끄러지지 않으려면 바닥 마찰이 관성력보다 커야 한다. 벨트를 화물 위로 감아 당기면
양쪽 끈이 T sinα 씩 화물을 눌러 마찰을 늘린다. 표준은 여기에 안전계수 fs 를 곱한다.
    2 μ T sinα >= m g (A - μ) fs        ->        T >= m g (A - μ) fs / (2 μ sinα)
m = 그쪽 무게(kg), A = 버텨야 할 가감속(TENSION_A, G), μ = TENSION_MU, α = TENSION_ANGLE, fs = SAFETY_FACTOR.
마찰만으로 버티면(A <= μ) 식은 0 이 되지만, 그래도 진동으로 밀리지 않게 1kg 당 MIN_N_PER_KG 는 둔다
(EN 12195-1 4.1). 그리고 화물이 가벼워도 TENSION_FLOOR 밑으로는 안 내린다.

상한 = TENSION_MAX. 벨트 · 장력 셀이 견디는 힘이다 (EN 12195-1 의 LC, lashing capacity).
EN 12195-1 과 IRU 가이드는 조임 장력을 LC 의 50% 까지로 둔다.

위험도 4단계. 낮은 쪽(풀림)과 높은 쪽(과조임)을 따로 보고 나쁜 쪽을 낸다.
            장력 / 하한             장력 / 상한
    정상    >= 100%                 <= 50%
    주의    <  100%  EN 하한 미달    >  50%   과조임 (조임은 LC 의 50% 까지)
    경고    <   75%                  >  75%
    위험    <   50%  풀림            > 100%   상한 넘음 (벨트 · 셀 한계)
100% · 50% 는 근거가 있는 값이고, 75% 는 그 사이 가운데다 (total.py 의 125% 와 같은 방식).
판정 근거 정리: analysis/tension_EN12195-1.md

그쪽 셀 무게에는 이 벨트가 화물을 누른 몫(장력 x 2 sinα, Config.belt_push_g_per_n())이 들어 있으므로 빼고 쓴다.
안 빼면 세게 조일수록 그쪽이 무거워져 하한도 같이 올라간다 (2026-10-04: S쪽 c2 9N 이면 1.9kg 무겁게 잡힘).
가감속 중에는 앞뒤 셀 무게가 하중 이동으로 출렁이므로 그쪽 무게는 길게(SIDE_SMOOTH_S) 평활한다.
그쪽 무게가 SIDE_MIN_KG 보다 가벼우면 잡을 화물이 없으니 하한은 내지 않고 상한만 본다.
"""
import math

from analysis.risk import CAUTION, DANGER, OK, WARNING, Check, Ema, static_g

# --- 바꿔도 되는 기준값 ------------------------------------------------------
SAFETY_FACTOR = 1.1     # EN 12195-1 안전계수 fs. 측방 · 후방 1.1, 전방 1.25 (TENSION_A 를 전방 0.8 로 두면 1.25)
MIN_N_PER_KG = 1.0      # 하한의 최솟값(N/kg). IRU 가이드: 미끄럼 위험이 없어도 4t 당 STF 400daN (= 1kg 당 1N)
LOW_CAUTION_PCT = 100   # 장력이 하한의 이 % 보다 낮으면 주의 (EN 하한 미달)
LOW_WARNING_PCT = 75    # 이 % 보다 낮으면 경고
LOW_DANGER_PCT = 50     # 이 % 보다 낮으면 위험 (풀림)
HIGH_CAUTION_PCT = 50   # 장력이 상한(TENSION_MAX)의 이 % 를 넘으면 주의 (과조임)
HIGH_WARNING_PCT = 75   # 이 % 를 넘으면 경고
HIGH_DANGER_PCT = 100   # 이 % 를 넘으면 위험 (상한 넘음)
SIDE_SMOOTH_S = 3.0     # 그쪽 무게 평활 시정수(초). 가감속 때 하중 이동으로 출렁이는 걸 누른다
TENSION_SMOOTH_S = 0.3  # 장력 평활 시정수(초)
SIDE_MIN_KG = 0.2       # 그쪽 무게가 이보다 가벼우면 화물 없음

G = 9.80665             # m/s^2 (acquisition/config.py 와 같은 값)


def need_per_kg(cfg):
    """그쪽 화물 1kg 을 붙잡는 데 드는 벨트 장력(N/kg). 위 식을 m = 1kg 으로 푼 값, 최소 MIN_N_PER_KG."""
    sin_a = math.sin(math.radians(cfg.tension_angle))
    slide = G * (cfg.tension_a - cfg.tension_mu) * SAFETY_FACTOR / (2 * cfg.tension_mu * sin_a)
    return max(MIN_N_PER_KG, slide)


def check_name(side, label):
    """화면 · 엑셀에 나가는 판정 이름. 예) 장력 N(c1)"""
    return "장력 %s(%s)" % (side, label)


def low_level(percent):
    """장력이 하한의 percent % 일 때 (레벨, 넘은 기준 %). 정상이면 (OK, None)."""
    if percent < LOW_DANGER_PCT:
        return DANGER, LOW_DANGER_PCT
    if percent < LOW_WARNING_PCT:
        return WARNING, LOW_WARNING_PCT
    if percent < LOW_CAUTION_PCT:
        return CAUTION, LOW_CAUTION_PCT
    return OK, None


def high_level(percent):
    """장력이 상한의 percent % 일 때 (레벨, 넘은 기준 %). 정상이면 (OK, None)."""
    if percent > HIGH_DANGER_PCT:
        return DANGER, HIGH_DANGER_PCT
    if percent > HIGH_WARNING_PCT:
        return WARNING, HIGH_WARNING_PCT
    if percent > HIGH_CAUTION_PCT:
        return CAUTION, HIGH_CAUTION_PCT
    return OK, None


class TensionCheck:
    def __init__(self, cfg):
        # {장력 라벨: (쪽 이름 N / S, 그 벨트가 맡는 바닥 셀 라벨들)}
        self.sides = cfg.belt_sides()
        # 그쪽 화물 1kg 당 필요한 장력(N/kg)
        self.per_kg = need_per_kg(cfg)
        # 벨트 장력 1N 이 그쪽 바닥 셀 무게에 더하는 무게(g) = 2 sinα / g x 1000
        self.push_g_per_n = cfg.belt_push_g_per_n()
        # 하한의 최솟값(N), 상한(N)
        self.floor_n, self.max_n = cfg.tension_floor, cfg.tension_max
        # 벨트마다 평활기. 그쪽 무게(kg)용 / 장력(N)용
        self.smooth_side = {label: Ema(SIDE_SMOOTH_S) for label in self.sides}
        self.smooth_tension = {label: Ema(TENSION_SMOOTH_S) for label in self.sides}

    def __call__(self, t, frame, imu):
        """t = 경과(초), frame = 이번 프레임, imu = 차체 IMU 값 (없으면 None).
        -> ({장력 라벨: 하한 N 또는 None}, [Check] 벨트마다 N쪽, S쪽 순서)."""
        mins, checks = {}, []
        for label, (side, cells) in self.sides.items():
            name = check_name(side, label)

            # 그쪽 바닥 셀마다 잰 무게(g). 하나라도 없으면 그쪽 무게를 못 낸다
            cell_g = [frame.load_g.get(cell) for cell in cells]
            # 이번 프레임 장력(N). 보정 전이거나 응답이 없으면 None
            now_n = frame.tension_n.get(label)
            # 이 벨트가 화물을 눌러 그쪽 셀에 더해진 무게(g). 장력 값이 없으면 못 빼고 무겁게 둔다 (하한이 올라가 안전한 쪽)
            push_g = self.push_g_per_n * max(0.0, now_n) if now_n is not None else 0.0
            # 그쪽 무게(kg) = 셀 합 - 벨트 누름. az 로 나눠 위아래 출렁임을 빼고 길게 평활
            side_g = None if None in cell_g else static_g(sum(cell_g) - push_g, imu)
            side_kg = self.smooth_side[label](t, None if side_g is None else side_g / 1000)
            # 벨트 장력(N). 짧게 평활
            belt_n = self.smooth_tension[label](t, now_n)

            # 하한(N)은 그쪽 무게로만 정해진다. 장력 셀 값이 없어도(보정 전) 하한은 낸다
            if side_kg is not None and side_kg >= SIDE_MIN_KG:
                need_n = max(self.floor_n, self.per_kg * side_kg)
            else:
                need_n = None
            mins[label] = need_n

            if side_kg is None:
                checks.append(Check(label, name, None, "%s 값 없음" % "+".join(cells)))
                continue
            if belt_n is None:
                checks.append(Check(label, name, None, "장력 셀 값 없음"))
                continue
            where = "%s %.1fkg" % ("+".join(cells), side_kg)       # 그쪽 셀과 무게. 이유 끝에 붙인다

            # 낮은 쪽: 장력이 하한의 몇 % 인가. 그쪽에 화물이 없으면 안 본다
            low_pct = 100 * belt_n / need_n if need_n else None
            low, low_limit = low_level(low_pct) if low_pct is not None else (OK, None)
            # 높은 쪽: 장력이 상한의 몇 % 인가
            high_pct = 100 * belt_n / self.max_n
            high, high_limit = high_level(high_pct)

            # 레벨이 높은 쪽 이유부터. 둘 다 정상이 아니면(하한이 상한의 절반을 넘을 만큼 무거우면) 둘 다 적는다
            reasons = []
            if low != OK:
                reasons.append((low, "%s %.1fN = 하한 %.1fN 의 %.0f%% (%d%% 밑) · %s" % (
                    "풀림" if low == DANGER else "부족", belt_n, need_n, low_pct, low_limit, where)))
            if high != OK:
                reasons.append((high, "%s %.1fN = 상한 %gN 의 %.0f%% (%d%% 넘음)" % (
                    "상한 넘음" if high == DANGER else "과조임", belt_n, self.max_n, high_pct, high_limit)))
            reasons.sort(key=lambda r: -r[0])

            if reasons:
                level, text = reasons[0][0], " · ".join(r[1] for r in reasons)
            elif need_n is None:
                level, text = OK, "%.1fN (화물 없음)" % belt_n
            else:
                level, text = OK, "%.1fN = 하한 %.1fN 의 %.0f%% · %s" % (belt_n, need_n, low_pct, where)
            checks.append(Check(label, name, level, text))
        return mins, checks
