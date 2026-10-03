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
import os
import time


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

    def roles(self, labels=None):
        """(하중 라벨, 장력 라벨, 무게중심 모서리 4개 또는 None).
        labels 를 주면 그 부분집합 기준 (안 보이는 모듈을 빼고 연 경우)."""
        labels = self.hx_labels if labels is None else labels
        tension = [l for l in labels if l in self.hx_tension]
        weight = [l for l in labels if l not in self.hx_tension]
        corners = self.cog_corners if all(c in weight for c in self.cog_corners) else None
        return weight, tension, corners

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


if __name__ == "__main__":
    print(Config().describe())
