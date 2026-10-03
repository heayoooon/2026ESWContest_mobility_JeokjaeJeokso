"""파이프라인의 계약: 한 프레임 = HX711 샘플 한 번(약 10Hz) + 그 구간의 IMU.

FrameSource(acquisition/source.py) 가 만들고, 판정(analysis/analyzer.py)과 기록(acquisition/log_xlsx.py)은
이것만 본다. 드라이버나 핀은 모른다.

dict 의 키는 설정(HX_LABELS, IMU_LABELS, HX_TENSION)으로 고정된다. 응답 없는 모듈 · 못 연 IMU ·
보정값이 없는 채널은 키는 그대로 있고 값이 None 이다. 값은 반올림하지 않은 그대로다.
"""
from dataclasses import dataclass


@dataclass
class ImuReading:
    """IMU 하나의 프레임 구간 값. 장착 자세 보정 후라 +Z 가 위다."""
    ax: float           # G, 구간 평균
    ay: float
    az: float
    gx: float           # °/s, 구간 평균
    gy: float
    gz: float
    roll: float         # °, 상보필터 현재값
    pitch: float
    ax_peak: float      # G, 구간 안에서 |ax| 가 가장 컸던 값 (부호 유지). 급제동 · 급가속
    ay_peak: float      # G, 같은 방식으로 |ay|. 급선회
    n: int              # 평균 낸 샘플 수


@dataclass
class Frame:
    seq: int
    t_ns: int           # HX711 변환이 끝난 시각 (CLOCK_MONOTONIC)
    t_s: float          # 수집 시작부터 초
    wall: float         # 벽시계 (unix 초). 다른 프로세스(twin 등) 기록과 맞출 때
    load_g: dict        # 바닥 하중 셀 {라벨: g | None}
    tension_n: dict     # 벨트 장력 셀 {라벨: N | None}
    raw: dict           # HX711 전 채널 {라벨: count | None}
    imu: dict           # {라벨: ImuReading | None}
    flags: list         # sck_high, stuck_hi:<라벨>, imu_miss:<라벨> 등
