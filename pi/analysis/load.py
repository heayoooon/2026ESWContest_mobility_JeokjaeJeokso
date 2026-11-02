"""최대 적재 판정. 바닥 셀 합(g) -> 적재 무게(kg) -> LOAD_MAX_KG 와 비교.

차가 위아래로 흔들리면 셀 합도 같이 출렁이므로 차체 IMU 의 az 로 나눠 정지 무게로 되돌리고
TAU_S 초로 평활한다. 넘으면 위험, WARN_RATIO 를 넘으면 주의.
과적은 실을 때 정해지는 것이라 서두를 필요가 없다. 화물이 넘어지며 벨트를 당길 때처럼 셀 합이 1~2초 튀는
것(2026-10-03 화물떨어짐: 13.5 -> 21kg, 1.5초)에 과적이 뜨지 않게 analyzer 의 HOLD 로 2초 이어져야 알린다.
"""
from analysis.risk import CAUTION, DANGER, OK, Check, Ema, static_g

TAU_S = 1.0         # 무게 평활 시정수(초)
WARN_RATIO = 0.9    # 최대 적재의 이 비율을 넘으면 주의


class LoadCheck:
    def __init__(self, max_kg):
        self.max_kg = max_kg
        self.ema = Ema(TAU_S)

    def __call__(self, t, total_g, imu):
        """-> (적재 kg 또는 None, Check)"""
        g = self.ema(t, static_g(total_g, imu))
        if g is None:
            return None, Check("load", "적재", None, "바닥 셀 값 없음")
        kg = g / 1000
        if kg > self.max_kg:
            return kg, Check("load", "적재", DANGER, "과적 %.1fkg > 최대 %gkg" % (kg, self.max_kg))
        if kg > WARN_RATIO * self.max_kg:
            return kg, Check("load", "적재", CAUTION, "%.1fkg (최대 %gkg 의 %.0f%%)"
                             % (kg, self.max_kg, 100 * kg / self.max_kg))
        return kg, Check("load", "적재", OK, "%.1fkg" % kg)
