"""핀 배치와 수집 파라미터를 전부 환경변수로 받는다.

코드 어디에도 핀 번호를 박지 않는다. 배선이 바뀌면 환경변수만 고치면 된다.
값을 주지 않으면 아래 기본값(공유 SCK 배선 제안안)이 쓰인다.

HX711
  HX_SCK       SCK 핀 (BCM). 값 하나면 전 모듈이 공유하는 배선,
               쉼표로 여러 개면 모듈별 SCK 배선(HX_DT 와 개수·순서를 맞춘다).
                                                         기본 16   (물리 36)
  HX_DT        모듈별 DT 핀, 쉼표 구분 (BCM)             기본 5,6,13,19,26,20
  HX_LABELS    DT 순서대로 붙일 이름                     기본 c1,c2,...
  HX_GAIN      128 | 64 | 32                             기본 128
  HX_PULL      up | down | none   (DT 내부 풀)           기본 up
  HX_RATE      10 | 80  (모듈 RATE 핀 설정과 맞출 것)    기본 10
  HX_TIMEOUT   DRDY 기다리는 한계(초)                    기본 1.0
  HX_CHIP      gpiochip 번호. 비우면 4 -> 0 순으로 시도
  HX_INVERT    누르면 값이 줄어드는 채널 라벨, 쉼표 구분.
               드라이버가 부호를 뒤집는다 (A+/A- 를 바꿔 꽂은 것과 같다).
               바꾸면 calibrate.py tare 부터 다시           기본 없음

채널 역할 / 무게중심 (pipeline.py, calibrate.py)
  HX_TENSION   HX_LABELS 중 벨트 장력(인장형) 셀.
               나머지는 전부 바닥 하중 셀로 본다        기본 T1,T2
  COG_CORNERS  무게중심 계산에 쓸 바닥 셀 4개,
               앞왼,앞오,뒤왼,뒤오 순서                  기본 FL,FR,RL,RR
  BED_L        앞왼-앞오 셀 사이 거리(mm)  } 둘 다 주면 무게중심을 mm 로,
  BED_W        앞왼-뒤왼 셀 사이 거리(mm)  } 안 주면 % 로 적는다 (50 = 가운데)
               HX_TENSION 순서가 곧 벨트 위치다: 첫 번째 = N(앞)쪽 벨트, 두 번째 = S(뒤)쪽 벨트.
               화물 무게를 무게중심 위치에 따라 두 벨트가 지렛대처럼 나눠 맡고, 그 몫으로 장력 하한을 정한다.
               벨트가 하나뿐이면 화물 전부
  BELT_N_Y     N쪽 벨트가 화물을 넘어가는 남북 위치(mm). 앞쪽 셀 줄(앞왼-앞오)에서 뒤쪽으로 잰 거리,
               무게중심 Y 와 같은 좌표다. 셀 줄 밖이면 음수 / BED_W 보다 큰 값      기본 0 (앞쪽 셀 줄 위)
  BELT_S_Y     S쪽 벨트 위치(mm), 같은 기준                    기본 BED_W (뒤쪽 셀 줄 위)
               벨트가 누르는 힘을 그 위치에서 빼서 화물 무게중심을 내고(무게중심 · 화물 이동 판정도 이 값),
               두 벨트 사이 어디에 무게중심이 있는지로 몫을 나눈다.
               둘 다 기본값이면 몫은 예전 계산(앞쪽 두 셀 무게 / 뒤쪽 두 셀 무게)과 같다. BED_L/BED_W 가 있어야 쓴다

판정 · 위험도 (analysis/, pipeline.py). 정상 / 주의 / 경고 / 위험 4단계 (판정 4개 다 쓴다)
  적재 기준(적정 적재중량 kg, 주의 · 경고 · 위험 %)은 환경변수가 아니라 analysis/total.py 맨 위 상수다
  장력 단계 기준(하한 · 상한의 몇 %, 안전계수 fs)도 analysis/tension.py 맨 위 상수다
  종합 위험도는 적재 · 무게중심 · 장력 단계를 합친 것이다 (analysis/overall.py, 화물 이동은 안 넣는다)
  COG_CAUTION  무게중심이 판 가장자리에서 셀 간격의 이 % 안으로 들어오면 주의  기본 25
  COG_WARNING  같은 방식으로 경고                                기본 15
  COG_DANGER   같은 방식으로 위험                                기본 7
               (기본값이면 가운데 50% = 정상, 그 바깥 10% = 주의, 8% = 경고, 가장자리 7% = 위험)
  COG_RESID    IMU 로 예측한 무게중심과 실측의 차 허용(셀 간격 %), 가속이 없을 때.
               가감속으로 설명 안 되는 변화 = 화물 이동. 허용의 1 · 1.4 · 1.8배부터 주의 · 경고 · 위험  기본 10
  COG_RESID_A  가속 1G 당 늘어나는 허용(셀 간격 %). 허용 = COG_RESID + COG_RESID_A x |a|.
               보상을 해도 급가감속 중에는 h 오차가 가속에 비례해 남는다      기본 20
  MOVE_WIN_S   화물 이동을 볼 최근 구간(초)                       기본 1.0
  MOVE_ALPHA   그 구간에서 이 비율 이상이 그 단계여야 판정 (단발성 충격을 거른다)  기본 0.6
  MOVE_CONFIRM_S  이동이 이만큼(초) 이어지면 최소 경고 (그 전은 의심)  기본 1.5
  MOVE_DANGER_S   이동이 이만큼(초) 이어지고 잔차가 위험 수준이면 위험  기본 3.0
  COG_LAG     로드셀이 IMU 보다 늦는 시간(초)                  기본 0.2
  COG_H        무게중심 높이 h 의 시작값(mm). 주행 중 가감속 데이터로 고쳐 나간다.
               BED_L/BED_W 가 없으면(% 단위) 안 쓰고 데이터로만 추정  기본 190
               (2026-10-04 실측에서 맞춘 h: 앞뒤 155~204mm, 좌우 132~190mm).
               화물 이동이 위험으로 확정되거나 화물을 내리면 학습한 h 를 버리고 이 값으로 돌아간다
  JUDGE_IMU    하중 이동 예측 · 무게 보정에 쓸 IMU 라벨 (차체에 붙은 것)  기본 IMU 라벨 첫 번째
  TENSION_A    벨트가 버텨야 할 가감속(G)                       기본 0.5
  TENSION_MU   화물-바닥 마찰계수                                기본 0.5
  TENSION_ANGLE 벨트가 바닥과 이루는 각(°)                       기본 90
               장력 하한(N) = 그 벨트가 맡은 무게(kg) x g(A - MU) fs / (2 MU sin ANGLE)   (EN 12195-1 식 10, 위로 감기)
               하한의 100% 밑 주의, 75% 밑 경고, 50% 밑 위험 (analysis/tension.py)
               벨트가 화물을 누르는 힘(장력 x 2 sin ANGLE)은 바닥 셀에 더해지므로 적재 무게 ·
               장력 하한을 낼 때 빼는 데도 이 각을 쓴다
  TENSION_FLOOR 장력 하한의 최솟값(N). 화물이 가벼워도 벨트는 이만큼은 팽팽해야 한다  기본 2
  TENSION_MAX  장력 상한(N) = 벨트 · 장력 셀이 견디는 힘 (EN 12195-1 의 LC).
               50% 넘으면 주의(과조임), 75% 경고, 100% 위험         기본 80

MPU-6050
  IMU_BUS      I2C 버스 번호                             기본 1
  IMU_ADDRS    파이에 직접 물린 센서 주소, 쉼표 구분     기본 0x68
  IMU_LABELS   주소 순서대로 붙일 이름                   기본 imu1
  IMU_UDP_PORT ESP32 무선 센서를 받을 UDP 포트 (펌웨어 PI_PORT 와 맞출 것).
               0 이면 안 받는다. CAM_PORT 와 달라야 한다   기본 5005
  IMU_UDP_LABEL ESP32 센서 이름 (CSV 열 imu2_*)          기본 imu2
  IMU_UDP_ADDR  ESP32 센서의 MPU 주소 (보정 키 esp32:<주소>) 기본 0x68
  IMU_HZ       목표 샘플레이트                           기본 100
  IMU_CAL      imu_cal.json 경로 (calibration/imu_calibrate.py 결과).
               비우면 calibration/imu_cal.json

카메라 (D435i 트윈 노드 twin/pi_twin_node.py 가 보내는 UDP 패킷, pipeline.py)
  CAM_PORT     받을 UDP 포트. 패킷이 오면 <LOG_NAME>_camera.xlsx 를
               센서 엑셀과 같이 만든다. 0 이면 안 받는다   기본 5006
               (컬러 · depth 원본은 트윈 노드가 .db3 로 녹화한다. twin/pi_twin_node.py 머리말)

로그
  LOG_DIR     로그가 쌓일 상위 폴더                     기본 ./logs
  LOG_SECONDS  기록 길이(초). 0 이면 Ctrl+C 까지         기본 0
  LOG_NAME     이번 런 이름(폴더/파일 이름). 비우면 시각으로 자동

예)
  HX_DT=5,6,13 HX_LABELS=FL,FR,RL python3 acquisition/log_sync.py
  HX_SCK=17 LOG_SECONDS=60 python3 acquisition/log_sync.py
  # 모듈별 SCK (4모서리)
  HX_SCK=21,26,17,13 HX_DT=20,16,18,19 HX_LABELS=SW,SE,NW,NE python3 acquisition/log_sync.py
"""
import math
import os
import time

G = 9.80665     # m/s^2

def _str(name, default):
    v = os.environ.get(name)
    return default if v is None or v.strip() == "" else v.strip()


def _int(name, default):
    return int(_str(name, str(default)), 0)


def _float(name, default):
    return float(_str(name, str(default)))


def _int_list(name, default):
    raw = _str(name, default).replace(",", " ").split()
    return [int(x, 0) for x in raw]


def _str_list(name, default):
    return [x for x in _str(name, default).replace(",", " ").split()]


class Config:
    def __init__(self):
        # --- HX711 -------------------------------------------------------
        self.hx_sck = _int_list("HX_SCK", "16")
        self.hx_dt = _int_list("HX_DT", "5,6,13,19,26,20")
        self.hx_shared_sck = len(self.hx_sck) == 1
        self.hx_gain = _int("HX_GAIN", 128)
        self.hx_pull = _str("HX_PULL", "up").lower()
        self.hx_rate = _int("HX_RATE", 10)
        self.hx_timeout = _float("HX_TIMEOUT", 1.0)
        chip = _str("HX_CHIP", "")
        self.hx_chip = int(chip, 0) if chip else None

        n = len(self.hx_dt)
        self.hx_labels = _str_list("HX_LABELS", ",".join("c%d" % (i + 1) for i in range(n)))
        self.hx_invert = _str_list("HX_INVERT", "")

        # --- 채널 역할 / 무게중심 ----------------------------------------
        self.hx_tension = _str_list("HX_TENSION", "T1,T2")
        self.cog_corners = _str_list("COG_CORNERS", "FL,FR,RL,RR")
        self.bed_l = _float("BED_L", 0) or None
        self.bed_w = _float("BED_W", 0) or None
        self.belt_n_y = _float("BELT_N_Y", 0)
        self.belt_s_y = _float("BELT_S_Y", self.bed_w or 0)

        # --- MPU-6050 ----------------------------------------------------
        self.imu_bus = _int("IMU_BUS", 1)
        self.imu_addrs = _int_list("IMU_ADDRS", "0x68")
        m = len(self.imu_addrs)
        self.imu_i2c_labels = _str_list("IMU_LABELS", ",".join("imu%d" % (i + 1) for i in range(m)))
        self.imu_udp_port = _int("IMU_UDP_PORT", 5005)
        self.imu_udp_label = _str("IMU_UDP_LABEL", "imu%d" % (m + 1))
        self.imu_udp_addr = _int("IMU_UDP_ADDR", 0x68)
        # 프레임/엑셀 열에 쓰는 전체 IMU 라벨: 파이에 직접 물린 것 + ESP32 무선
        self.imu_labels = self.imu_i2c_labels + ([self.imu_udp_label] if self.imu_udp_port else [])
        self.imu_hz = _int("IMU_HZ", 100)
        self.imu_cal = _str("IMU_CAL", "")

        # --- 판정 · 위험도 -----------------------------------------------
        self.cog_caution = _float("COG_CAUTION", 25)
        self.cog_warning = _float("COG_WARNING", 15)
        self.cog_danger = _float("COG_DANGER", 7)
        self.cog_resid = _float("COG_RESID", 10)
        self.cog_resid_a = _float("COG_RESID_A", 20)
        self.move_win_s = _float("MOVE_WIN_S", 1.0)
        self.move_alpha = _float("MOVE_ALPHA", 0.6)
        self.move_confirm_s = _float("MOVE_CONFIRM_S", 1.5)
        self.move_danger_s = _float("MOVE_DANGER_S", 3.0)
        self.cog_lag = _float("COG_LAG", 0.2)
        self.cog_h = _float("COG_H", 190) or None
        self.judge_imu = _str("JUDGE_IMU", self.imu_labels[0] if self.imu_labels else "")
        self.tension_a = _float("TENSION_A", 0.5)
        self.tension_mu = _float("TENSION_MU", 0.5)
        self.tension_angle = _float("TENSION_ANGLE", 90)
        self.tension_floor = _float("TENSION_FLOOR", 2)
        self.tension_max = _float("TENSION_MAX", 80)

        # --- 카메라 ------------------------------------------------------
        self.cam_port = _int("CAM_PORT", 5006)

        # --- 로그 --------------------------------------------------------
        self.log_dir = _str("LOG_DIR", os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs"))
        self.log_seconds = _float("LOG_SECONDS", 0)
        self.log_name = _str("LOG_NAME", time.strftime("%Y%m%d_%H%M%S"))

        self._validate()

    def _validate(self):
        if len(self.hx_labels) != len(self.hx_dt):
            raise SystemExit("HX_LABELS %d개 != HX_DT %d개" % (len(self.hx_labels), len(self.hx_dt)))
        if len(set(self.hx_labels)) != len(self.hx_labels):
            raise SystemExit("HX_LABELS 에 같은 이름이 두 번 들어있다: %r" % (self.hx_labels,))
        unknown = [l for l in self.hx_invert if l not in self.hx_labels]
        if unknown:
            raise SystemExit("HX_INVERT 에 HX_LABELS 에 없는 라벨: %s" % ", ".join(unknown))
        if len(set(self.hx_dt)) != len(self.hx_dt):
            raise SystemExit("HX_DT 에 같은 핀이 두 번 들어있다: %r" % (self.hx_dt,))
        if len(set(self.hx_sck)) != len(self.hx_sck):
            raise SystemExit("HX_SCK 에 같은 핀이 두 번 들어있다: %r" % (self.hx_sck,))
        clash = set(self.hx_sck) & set(self.hx_dt)
        if clash:
            raise SystemExit("HX_SCK 와 HX_DT 에 같은 핀: %s"
                             % ", ".join("GPIO%d" % p for p in sorted(clash)))
        if not self.hx_shared_sck and len(self.hx_sck) != len(self.hx_dt):
            raise SystemExit("모듈별 SCK 배선이면 HX_SCK 와 HX_DT 개수가 같아야 한다 "
                             "(SCK %d개, DT %d개)" % (len(self.hx_sck), len(self.hx_dt)))
        if {2, 3} & (set(self.hx_dt) | set(self.hx_sck)):
            raise SystemExit("GPIO2/3 은 IMU I2C 전용이다. HX_SCK/HX_DT 에서 빼라")
        if self.hx_gain not in (128, 64, 32):
            raise SystemExit("HX_GAIN 은 128/64/32 중 하나")
        if self.hx_pull not in ("up", "down", "none"):
            raise SystemExit("HX_PULL 은 up/down/none 중 하나")
        if len(self.imu_i2c_labels) != len(self.imu_addrs):
            raise SystemExit("IMU_LABELS %d개 != IMU_ADDRS %d개" % (len(self.imu_i2c_labels), len(self.imu_addrs)))
        if len(set(self.imu_labels)) != len(self.imu_labels):
            raise SystemExit("IMU 라벨이 겹친다: %r (IMU_LABELS / IMU_UDP_LABEL)" % (self.imu_labels,))
        if not 0 <= self.imu_udp_port <= 65535:
            raise SystemExit("IMU_UDP_PORT 는 0(안 받음) 또는 1~65535")
        if self.imu_udp_port and self.imu_udp_port == self.cam_port:
            raise SystemExit("IMU_UDP_PORT 와 CAM_PORT 가 같다 (%d). 하나를 바꿔라" % self.cam_port)
        if len(self.cog_corners) != 4 or len(set(self.cog_corners)) != 4:
            raise SystemExit("COG_CORNERS 는 서로 다른 라벨 4개 (앞왼,앞오,뒤왼,뒤오): %r"
                             % (self.cog_corners,))
        both = set(self.cog_corners) & set(self.hx_tension)
        if both:
            raise SystemExit("COG_CORNERS 와 HX_TENSION 에 같은 라벨: %s" % ", ".join(sorted(both)))
        if not 0 <= self.cam_port <= 65535:
            raise SystemExit("CAM_PORT 는 0(안 받음) 또는 1~65535")
        if (self.bed_l is None) != (self.bed_w is None):
            raise SystemExit("BED_L 과 BED_W 는 둘 다 주거나 둘 다 비운다")
        if self.bed_l is not None and (self.bed_l < 0 or self.bed_w < 0):
            raise SystemExit("BED_L / BED_W 는 양수(mm)")
        if self.bed_w is None and (_str("BELT_N_Y", "") or _str("BELT_S_Y", "")):
            raise SystemExit("BELT_N_Y / BELT_S_Y 는 mm 라 BED_L / BED_W 도 줘야 한다")
        if self.bed_w is not None and not self.belt_n_y < self.belt_s_y:
            raise SystemExit("BELT_N_Y < BELT_S_Y (mm, 앞쪽 셀 줄에서 뒤로 잰 거리): %g, %g"
                             % (self.belt_n_y, self.belt_s_y))
        if not 0 <= self.cog_danger < self.cog_warning < self.cog_caution < 50:
            raise SystemExit("0 <= COG_DANGER < COG_WARNING < COG_CAUTION < 50 (가장자리에서 셀 간격 %%): %g, %g, %g"
                             % (self.cog_danger, self.cog_warning, self.cog_caution))
        if self.cog_resid <= 0 or self.cog_lag < 0:
            raise SystemExit("COG_RESID 는 양수(%), COG_LAG 는 0 이상(초)")
        if self.cog_resid_a < 0:
            raise SystemExit("COG_RESID_A 는 0 이상(셀 간격 %/G)")
        if self.move_win_s <= 0 or not 0 < self.move_alpha <= 1:
            raise SystemExit("MOVE_WIN_S 는 양수(초), MOVE_ALPHA 는 0 초과 1 이하")
        if not 0 < self.move_confirm_s < self.move_danger_s:
            raise SystemExit("0 < MOVE_CONFIRM_S < MOVE_DANGER_S (초): %g, %g"
                             % (self.move_confirm_s, self.move_danger_s))
        if self.cog_h is not None and self.cog_h < 0:
            raise SystemExit("COG_H 는 양수(mm). 0 이면 시작값 없이 데이터로만 추정")
        if self.judge_imu and self.judge_imu not in self.imu_labels:
            raise SystemExit("JUDGE_IMU=%s 가 IMU 라벨 %r 에 없다" % (self.judge_imu, self.imu_labels))
        if self.tension_mu <= 0 or self.tension_a < 0:
            raise SystemExit("TENSION_MU 는 양수, TENSION_A 는 0 이상(G)")
        if not 0 < self.tension_angle <= 90:
            raise SystemExit("TENSION_ANGLE 은 0 초과 90 이하(°)")
        if not 0 <= self.tension_floor < self.tension_max:
            raise SystemExit("0 <= TENSION_FLOOR < TENSION_MAX (N)")

    def roles(self, labels=None):
        """(하중 라벨, 장력 라벨, 무게중심 모서리 4개 또는 None).
        labels 를 주면 그 부분집합 기준 (안 보이는 모듈을 빼고 연 경우)."""
        labels = self.hx_labels if labels is None else labels
        tension = [l for l in labels if l in self.hx_tension]
        weight = [l for l in labels if l not in self.hx_tension]
        corners = self.cog_corners if all(c in weight for c in self.cog_corners) else None
        return weight, tension, corners

    def belt_sides(self):
        """{장력 라벨: (쪽 이름, 벨트 남북 위치)}. HX_TENSION 순서대로 첫 번째 = ("N", BELT_N_Y), 두 번째 = ("S", BELT_S_Y).
        위치는 무게중심 Y 와 같은 좌표다 (앞쪽 셀 줄 0 -> 뒤쪽 셀 줄 BED_W mm. BED_W 가 없으면 셀 줄 위로 보고 0 / 100 %).
        벨트가 하나뿐이거나 세 번째부터는 ("전체", None) = 화물 전부를 맡는다. 모서리를 몰라도 ("전체", None)."""
        _, tension, corners = self.roles()
        if not corners or len(tension) < 2:
            return {t: ("전체", None) for t in tension}
        rows = (self.belt_n_y, self.belt_s_y) if self.bed_w else (0.0, 100.0)
        sides = [("N", rows[0]), ("S", rows[1])]
        return {t: sides[i] if i < len(sides) else ("전체", None) for i, t in enumerate(tension)}

    def belt_push_g_per_n(self):
        """벨트 장력 1N 이 바닥 셀 합에 더하는 무게(g). 화물 위로 감은 벨트는 양쪽 끈이
        2 T sin ANGLE 로 화물을 바닥 쪽으로 누른다 (2026-10-04 기록에서 맞춘 계수 1.96 ~ 이론 2)."""
        return 2 * math.sin(math.radians(self.tension_angle)) / G * 1000

    def as_dict(self):
        return {
            "hx": {
                "sck": self.hx_sck[0] if self.hx_shared_sck else self.hx_sck,
                "shared_sck": self.hx_shared_sck,
                "dt": self.hx_dt, "labels": self.hx_labels,
                "gain": self.hx_gain, "pull": self.hx_pull, "rate_sps": self.hx_rate,
                "chip": self.hx_chip, "invert": self.hx_invert,
            },
            "imu": {
                "bus": self.imu_bus, "addrs": [hex(a) for a in self.imu_addrs],
                "labels": self.imu_labels, "target_hz": self.imu_hz,
                "udp": ({"label": self.imu_udp_label, "port": self.imu_udp_port,
                         "addr": hex(self.imu_udp_addr)} if self.imu_udp_port else None),
            },
            "roles": {
                "tension": self.hx_tension, "cog_corners": self.cog_corners,
                "bed_mm": [self.bed_l, self.bed_w] if self.bed_l else None,
            },
            "cam": {"udp_port": self.cam_port or None},
            "log": {"dir": self.log_dir, "seconds": self.log_seconds, "name": self.log_name},
        }

    def describe(self):
        if self.hx_shared_sck:
            head = "SCK=GPIO%d (공유)" % self.hx_sck[0]
            pairs = ", ".join("%s=GPIO%d" % (l, d)
                              for l, d in zip(self.hx_labels, self.hx_dt))
        else:
            head = "SCK 모듈별 (동시 토글)"
            pairs = ", ".join("%s=SCK GPIO%d/DT GPIO%d" % (l, s, d)
                              for l, s, d in zip(self.hx_labels, self.hx_sck, self.hx_dt))
        imus = ", ".join("%s=%s" % (l, hex(a)) for l, a in zip(self.imu_i2c_labels, self.imu_addrs))
        if self.imu_udp_port:
            imus += (", " if imus else "") + "%s=ESP32 UDP %d" % (self.imu_udp_label, self.imu_udp_port)
        weight, tension, corners = self.roles()
        if corners:
            cog = "무게중심 %s (%s)" % ("/".join(corners), "%gx%gmm" % (self.bed_l, self.bed_w)
                                       if self.bed_l else "% 단위")
        else:
            cog = "무게중심 안 함 (COG_CORNERS=%s 가 하중 라벨에 없음)" % ",".join(self.cog_corners)
        inv = "  부호반전 [%s]" % ",".join(self.hx_invert) if self.hx_invert else ""
        return (
            "HX711  %s  %d채널  [%s]  gain=%d pull=%s rate=%dSPS%s\n"
            "역할   하중 [%s]  장력 [%s]  %s\n"
            "IMU    bus%d  [%s]  target=%dHz\n"
            "CAM    %s\n"
            "LOG    %s/%s  %s"
            % (head, len(self.hx_dt), pairs, self.hx_gain, self.hx_pull, self.hx_rate, inv,
               ",".join(weight), ",".join(tension), cog,
               self.imu_bus, imus, self.imu_hz,
               ("UDP %d (트윈 노드 패킷 -> pipeline.py 가 _camera 엑셀로)" % self.cam_port
                if self.cam_port else "안 받음 (CAM_PORT=0)"),
               self.log_dir, self.log_name,
               ("%.0f초" % self.log_seconds) if self.log_seconds else "Ctrl+C 까지")
        )

    def describe_judge(self):
        """pipeline.py 판정 기준 한눈에."""
        from analysis import tension, total     # 적재 · 장력 단계 기준은 환경변수가 아니라 각 파일 맨 위 상수
        _, _, corners = self.roles()
        lines = ["판정   적재  적정 %gkg 의 %d%% 넘으면 주의, %d%% 경고, %d%% 위험 (셀 합에서 장력 1N 당 %.0fg 뺌)" % (
            total.RATED_KG, total.CAUTION_PCT, total.WARNING_PCT, total.DANGER_PCT, self.belt_push_g_per_n())]
        if corners:
            lines.append("       무게중심  가장자리에서 %g%% 안 = 주의, %g%% 안 = 경고, %g%% 안 = 위험" % (
                self.cog_caution, self.cog_warning, self.cog_danger))
            from analysis import transfer           # 허용 배수는 환경변수가 아니라 transfer.py 상수
            lines.append("       화물 이동  IMU(%s) 예측과 차가 허용(%g%% + %g%%/G x |a|)의 %g · %g · %g배부터 주의 · 경고 · 위험, "
                         "최근 %gs 의 %g%% 이상일 때" % (
                             self.judge_imu or "없음", self.cog_resid, self.cog_resid_a, transfer.E_CAUTION,
                             transfer.E_WARNING, transfer.E_DANGER, self.move_win_s, 100 * self.move_alpha))
            lines.append("                  %gs 이어지면 경고, %gs 이어지면 위험 (지연 %.1fs, h 시작 %s)" % (
                self.move_confirm_s, self.move_danger_s, self.cog_lag,
                "%gmm" % self.cog_h if self.cog_h and self.bed_l else "없음 -> 데이터로만"))
        sides = self.belt_sides()
        if sides:
            unit = "mm" if self.bed_w else "%"
            lines.append("       장력  하한 = 맡은 무게(벨트 누름 뺀 화물을 두 벨트가 지렛대로 나눔) x %.2f N/kg (fs %g, 최소 %gN)  %s" % (
                tension.need_per_kg(self), tension.SAFETY_FACTOR, self.tension_floor,
                "  ".join("%s쪽 %s%s" % (side, t, "" if y is None else " Y %g%s" % (y, unit))
                          for t, (side, y) in sides.items())))
            lines.append("             하한의 %d%% 밑 주의, %d%% 경고, %d%% 위험 / 상한 %gN 의 %d%% 넘으면 주의, %d%% 경고, %d%% 위험" % (
                tension.LOW_CAUTION_PCT, tension.LOW_WARNING_PCT, tension.LOW_DANGER_PCT, self.tension_max,
                tension.HIGH_CAUTION_PCT, tension.HIGH_WARNING_PCT, tension.HIGH_DANGER_PCT))
        lines.append("       종합  적재 · 무게중심 · 장력(나쁜 벨트) 단계를 안전 여유 곱으로 합침: 하나라도 위험이면 위험, "
                     "경고 둘 -> 위험, 주의 셋 -> 경고. 화물 이동은 안 넣고 따로 알림")
        return "\n".join(lines)


if __name__ == "__main__":
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # pi/ (analysis 를 찾게)
    cfg = Config()
    print(cfg.describe())
    print(cfg.describe_judge())
