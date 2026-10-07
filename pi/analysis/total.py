"""총하중 판정. 바닥 셀 합에서 벨트가 누르는 힘을 빼 실제 화물 무게를 내고, 적정 적재중량 RATED_KG 와 비교한다.

벨트는 화물 위로 감겨 판 아래(차체)에 걸려 있다. 장력 T 인 벨트 하나는 양쪽 끈으로 화물을 2 T sinα 만큼
바닥 셀 쪽으로 누르므로(tension.py 의 위로 감기와 같은 그림, α = TENSION_ANGLE) 셀 합이 그만큼 무겁게 나온다.
    셀 합 = 화물 x az + Σ 2 T sinα / g        ->        화물 = (셀 합 - Σ 2 T sinα / g) / az
벨트 누름은 장력 셀이 바로 재므로 먼저 빼고, 남은 화물 몫만 IMU az 로 나눠 위아래 출렁임을 뺀다.
2026-10-04 기록 (정차 · 정속 · 급출발급정거 · 우회전 · 화물추락 · 장력느슨, 화물 약 13.8kg):
셀 합 14.5 ~ 16.7kg -> 뺀 뒤 13.5 ~ 13.9kg. 데이터로 맞춘 계수도 1.96 으로 이론값 2 sin90° = 2 와 같다.
비교용으로 벨트 누름을 빼지 않은 측정 무게(셀 합 / az, 예전 load.py 값)도 같이 낸다.

위험도 4단계는 화물이 적정 적재중량의 몇 % 인지로 정한다.
    정상   <= 110%   도로교통법 시행령 제22조: 적재중량은 구조 · 성능에 따른 적재중량의 110% 이내
                      (도로법 시행령 제79조 과적 단속도 측정 오차로 1할 초과까지는 허용)
    주의   <= 125%
    경고   <= 140%
    위험   >  140%   도로법 과태료: 제한 중량을 40% 이상 넘기면 중대 위반 (법정 상한)
과적은 실을 때 정해지는 것이라 서두를 필요가 없다. 화물이 넘어지며 벨트를 당길 때처럼 1~2초 튀는 값에는
안 뜨게 analyzer 의 HOLD 로 2초 이어져야 알린다.
"""
from analysis.risk import CAUTION, DANGER, OK, WARNING, Check, Ema, static_g

# --- 바꿔도 되는 기준값 ------------------------------------------------------
RATED_KG = 10.0         # 적정 적재중량(kg). 회의에서 바뀌면 여기만 고친다
CAUTION_PCT = 110       # 화물이 적정 적재중량의 이 % 를 넘으면 주의
WARNING_PCT = 125       # 이 % 를 넘으면 경고
DANGER_PCT = 140        # 이 % 를 넘으면 위험
SMOOTH_S = 1.0          # 무게 평활 시정수(초). 클수록 출렁임에 덜 흔들리고 늦게 따라간다


class TotalLoadCheck:
    def __init__(self, cfg):
        # 바닥 하중 셀 라벨 (예: SW, SE, NW, NE), 벨트 장력 셀 라벨 (예: c1, c2)
        self.floor_labels, self.belt_labels, _ = cfg.roles()
        # 벨트 장력 1N 이 바닥 셀 합에 더하는 무게(g) = 2 sinα / g x 1000
        self.push_g_per_n = cfg.belt_push_g_per_n()
        # 무게(g) 지수 평활기. 화물 무게용 / 측정 무게용
        self.smooth_cargo = Ema(SMOOTH_S)
        self.smooth_measured = Ema(SMOOTH_S)

    def __call__(self, t, frame, imu):
        """t = 경과(초), frame = 이번 프레임, imu = 차체 IMU 값 (없으면 None).
        -> (화물 무게 kg, 측정 무게 kg, Check). 바닥 셀 값이 없으면 두 무게 다 None.
        화물 무게 = 벨트 누름을 뺀 값, 측정 무게 = 셀 합 그대로 (둘 다 az 로 나누고 평활).
        장력 셀 값이 없으면 그 벨트는 0N 으로 본다 (무겁게 나와 안전한 쪽)."""
        # 바닥 셀마다 잰 무게 {라벨: g}. 하나라도 없으면 합이 틀리므로 판정하지 않는다
        floor_g = {label: frame.load_g.get(label) for label in self.floor_labels}
        if not floor_g or None in floor_g.values():
            return None, None, Check("load", "적재", None, "바닥 셀 값 없음")
        floor_total_g = sum(floor_g.values())

        # 벨트마다 잰 장력(N). 값이 없으면 None
        belt_n = {label: frame.tension_n.get(label) for label in self.belt_labels}
        # 벨트들이 화물을 눌러 바닥 셀 합에 더해진 무게(g). 느슨하면 장력이 음수로 읽히니 0 으로 친다
        belt_push_g = self.push_g_per_n * sum(max(0.0, n) for n in belt_n.values() if n is not None)

        # 측정 무게(kg) = 셀 합 그대로, 실제 화물 무게(kg) = 셀 합 - 벨트 누름. 둘 다 az 로 나눠 출렁임을 빼고 평활
        measured_kg = self.smooth_measured(t, static_g(floor_total_g, imu)) / 1000
        cargo_kg = self.smooth_cargo(t, static_g(floor_total_g - belt_push_g, imu)) / 1000
        percent = 100 * cargo_kg / RATED_KG        # 적정 적재중량의 몇 % 인가

        if percent > DANGER_PCT:
            level, limit_pct = DANGER, DANGER_PCT
        elif percent > WARNING_PCT:
            level, limit_pct = WARNING, WARNING_PCT
        elif percent > CAUTION_PCT:
            level, limit_pct = CAUTION, CAUTION_PCT
        else:
            level, limit_pct = OK, None

        if level == OK:
            text = "%.1fkg (적정 %gkg 의 %.0f%%)" % (cargo_kg, RATED_KG, percent)
        else:
            text = "과적 %.1fkg = 적정 %gkg 의 %.0f%% (%d%% 넘음)" % (cargo_kg, RATED_KG, percent, limit_pct)
        missing_belts = [label for label, n in belt_n.items() if n is None]    # 장력 값이 없는 벨트
        if missing_belts:
            text += " · 장력 보정 못 함 (%s 값 없음)" % ",".join(missing_belts)
        return cargo_kg, measured_kg, Check("load", "적재", level, text)
