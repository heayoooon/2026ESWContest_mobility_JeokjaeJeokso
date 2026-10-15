"""벨트 장력 판정. N(앞)쪽 벨트와 S(뒤)쪽 벨트 2개를 따로 판정한다.

벨트 배치는 HX_TENSION 순서로 정한다 (Config.belt_sides()). 첫 번째 장력 셀 = N쪽 벨트, 두 번째 = S쪽 벨트.
벨트가 화물을 넘어가는 남북 위치는 BELT_N_Y / BELT_S_Y 다 (앞쪽 셀 줄에서 뒤로 잰 mm, 무게중심 Y 와 같은 좌표).

벨트마다 맡는 무게 = 화물 무게를 두 벨트가 지렛대처럼 나눈 몫.
    N 몫 = (y_S - y_c) / (y_S - y_N),   S 몫 = 1 - N 몫
y_c 는 화물 무게중심 Y, y_N · y_S 는 벨트 위치. 몫은 0 ~ 1 로 자른다 (무게중심이 벨트 바깥이면 가까운 벨트가 전부).
화물이 N쪽 벨트 가까이 있으면 N쪽 벨트가 더 맡아 N쪽 하한이 올라간다.

벨트는 판 아래(차체)에 걸려 있어서 화물을 2 T sinα 로 누르고(total.py), 그 힘은 벨트 위치에서 바닥 셀에 실린다.
그래서 바닥 셀로 잰 무게와 무게중심에는 벨트 누름이 섞여 있다. 누름을 빼고 화물만 다시 낸다.
    화물 = 셀 합 - Σ 누름,        y_c = 누름을 벨트 위치에서 뺀 무게중심 Y (cog.py, 무게중심 판정과 같은 값)
2026-10-15 전에는 벨트가 셀 줄 바로 위에 있다고 보고 앞쪽 두 셀 - N 누름, 뒤쪽 두 셀 - S 누름을 썼다.
S쪽 벨트가 판 가운데 쪽에 있으면 S 누름 일부가 앞쪽 셀에 실리는데 그걸 못 빼서, S 벨트를 조일수록 N 하한이 올라갔다.
BELT_N_Y = 0, BELT_S_Y = BED_W(기본값)이면 예전 계산과 같다.

하한 = EN 12195-1:2010 식 10 (위로 감기 top-over lashing) 을 장력으로 푼 것.
화물이 미끄러지지 않으려면 바닥 마찰이 관성력보다 커야 한다. 벨트를 화물 위로 감아 당기면
양쪽 끈이 T sinα 씩 화물을 눌러 마찰을 늘린다. 표준은 여기에 안전계수 fs 를 곱한다.
    2 μ T sinα >= m g (A - μ) fs        ->        T >= m g (A - μ) fs / (2 μ sinα)
m = 그 벨트가 맡은 무게(kg), A = 버텨야 할 가감속(TENSION_A, G), μ = TENSION_MU, α = TENSION_ANGLE, fs = SAFETY_FACTOR.
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

가감속 중에는 하중 이동으로 무게중심이 출렁이므로 화물 무게와 무게중심은 길게(SIDE_SMOOTH_S) 평활한다.
맡은 무게가 SIDE_MIN_KG 보다 가벼우면 잡을 화물이 없으니 하한은 내지 않고 상한만 본다.
"""
import math

from analysis.cog import belt_press
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
SIDE_SMOOTH_S = 3.0     # 화물 무게 · 무게중심 평활 시정수(초). 가감속 때 하중 이동으로 출렁이는 걸 누른다
TENSION_SMOOTH_S = 0.3  # 장력 평활 시정수(초)
SIDE_MIN_KG = 0.2       # 맡은 무게가 이보다 가벼우면 화물 없음

G = 9.80665             # m/s^2 (acquisition/config.py 와 같은 값)


def need_per_kg(cfg):
    """그 벨트가 맡은 화물 1kg 을 붙잡는 데 드는 벨트 장력(N/kg). 위 식을 m = 1kg 으로 푼 값, 최소 MIN_N_PER_KG."""
    sin_a = math.sin(math.radians(cfg.tension_angle))
    slide = G * (cfg.tension_a - cfg.tension_mu) * SAFETY_FACTOR / (2 * cfg.tension_mu * sin_a)
    return max(MIN_N_PER_KG, slide)


def lever_share(y_c, y_self, y_other):
    """무게중심 y_c 인 화물을 y_self · y_other 위치의 두 벨트가 지렛대처럼 나눠 맡을 때 y_self 벨트의 몫(0~1)."""
    return min(1.0, max(0.0, (y_other - y_c) / (y_other - y_self)))


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
        # {장력 라벨: (쪽 이름 N / S / 전체, 벨트 남북 위치. 전체면 None)}
        self.sides = cfg.belt_sides()
        weight, _, corners = cfg.roles()
        # 화물 무게를 낼 바닥 셀. 모서리를 모르면 바닥 셀 전부
        self.cells = list(corners) if corners else list(weight)
        # 위치를 아는 벨트(N · S)마다 (내 위치, 짝 벨트 위치)
        located = [(label, y) for label, (_, y) in self.sides.items() if y is not None]
        self.pair = {label: (y, located[1 - i][1]) for i, (label, y) in enumerate(located)}
        # 맡은 화물 1kg 당 필요한 장력(N/kg)
        self.per_kg = need_per_kg(cfg)
        # 벨트 장력 1N 이 바닥 셀 무게에 더하는 무게(g) = 2 sinα / g x 1000
        self.push_g_per_n = cfg.belt_push_g_per_n()
        # 하한의 최솟값(N), 상한(N)
        self.floor_n, self.max_n = cfg.tension_floor, cfg.tension_max
        # 평활기. 화물 무게(kg) · 화물 무게중심 Y 는 길게, 장력(N)은 벨트마다 짧게
        self.smooth_cargo = Ema(SIDE_SMOOTH_S)
        self.smooth_y = Ema(SIDE_SMOOTH_S)
        self.smooth_tension = {label: Ema(TENSION_SMOOTH_S) for label in self.sides}

    def cargo(self, t, frame, push_g, c, imu):
        """push_g = {장력 라벨: 그 벨트가 누른 무게 g}, c = 벨트 누름을 뺀 무게중심 (cog.py, 없으면 None).
        -> (화물 무게 kg, 화물 무게중심 Y). 바닥 셀 값이 없으면 (None, None).
        위치를 아는 벨트가 없거나 무게중심이 없으면(화물이 거의 없음) 무게중심은 None."""
        cell_g = [frame.load_g.get(label) for label in self.cells]
        if None in cell_g:
            return None, None
        # 화물 무게(kg) = 셀 합 - 벨트 누름 전부. az 로 나눠 위아래 출렁임을 빼고 길게 평활
        cargo_kg = self.smooth_cargo(t, static_g(sum(cell_g) - sum(push_g.values()), imu) / 1000)
        if not self.pair or c is None:
            return cargo_kg, None
        # 화물 몫이 셀처럼 az 배로 출렁여도 무게중심에서는 위아래가 같이 커져 지워진다. 하중 이동 출렁임만 길게 평활한다
        return cargo_kg, self.smooth_y(t, c.y)

    def __call__(self, t, frame, imu, c):
        """t = 경과(초), frame = 이번 프레임, imu = 차체 IMU 값 (없으면 None),
        c = 벨트 누름을 벨트 위치에서 뺀 무게중심 (cog.py 의 cog(), 없으면 None).
        -> ({장력 라벨: 하한 N 또는 None}, [Check] 벨트마다 N쪽, S쪽 순서)."""
        # 이번 프레임 장력(N). 보정 전이거나 응답이 없으면 None
        now_n = {label: frame.tension_n.get(label) for label in self.sides}
        # 벨트마다 화물을 눌러 바닥 셀에 더해진 무게(g). 장력 값이 없으면 못 빼고 0 (무겁게 나와 하한이 올라가는 안전한 쪽)
        push_g = belt_press(frame, self.sides, self.push_g_per_n)
        cargo_kg, y_c = self.cargo(t, frame, push_g, c, imu)

        mins, checks = {}, []
        for label, (side, y) in self.sides.items():
            name = check_name(side, label)

            # 이 벨트가 맡은 무게(kg). 전체 벨트는 화물 전부, N · S 벨트는 무게중심 위치로 나눈 몫
            if cargo_kg is None:
                side_kg = None
            elif label not in self.pair:
                side_kg = cargo_kg
            else:
                side_kg = cargo_kg * lever_share(y_c, *self.pair[label]) if y_c is not None else 0.0
            # 벨트 장력(N). 짧게 평활
            belt_n = self.smooth_tension[label](t, now_n[label])

            # 하한(N)은 맡은 무게로만 정해진다. 장력 셀 값이 없어도(보정 전) 하한은 낸다
            if side_kg is not None and side_kg >= SIDE_MIN_KG:
                need_n = max(self.floor_n, self.per_kg * side_kg)
            else:
                need_n = None
            mins[label] = need_n

            if side_kg is None:
                missing = [c for c in self.cells if frame.load_g.get(c) is None]
                checks.append(Check(label, name, None, "%s 값 없음" % ",".join(missing)))
                continue
            if belt_n is None:
                checks.append(Check(label, name, None, "장력 셀 값 없음"))
                continue
            # 맡은 무게. 이유 끝에 붙인다
            if label in self.pair and cargo_kg > 0:
                where = "맡은 무게 %.1fkg = 화물 %.1fkg 의 %.0f%%" % (side_kg, cargo_kg, 100 * side_kg / cargo_kg)
            else:
                where = "맡은 무게 %.1fkg" % side_kg

            # 낮은 쪽: 장력이 하한의 몇 % 인가. 맡은 화물이 없으면 안 본다
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
