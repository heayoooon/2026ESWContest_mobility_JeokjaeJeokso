"""MPU-6050 최소 드라이버 (smbus2).

로거는 raw LSB 를 그대로 남긴다. 변환 계수(ACC_SCALE/GYR_SCALE)는 meta.json 에
적히므로 나중에 g / deg/s 로 바꾸면 된다. 원본을 잃지 않기 위해서다.

센서 구분은 핀이 아니라 I2C 주소다. 두 센서 모두 SDA=GPIO2 / SCL=GPIO3 한 줄에
물리고, AD0 를 3.3V 로 올린 쪽이 0x69, 비운 쪽이 0x68 이 된다. 버스 하나에
주소가 둘뿐이라 MPU-6050 은 버스당 최대 2개다.

물리 단위가 필요하면(acquisition/source.py) to_units() 로 바꾼다. calibration/imu_calibrate.py 가 만든
calibration/imu_cal.json 이 있으면 그 바이어스를 빼고, 장착 자세의 up 축이 +Z 가 되도록
축을 돌린다. ESP32 로 받는 센서(mpu6050_udp.py)도 같은 규칙을 쓴다.
"""
import json
import math
import os
import time

try:
    from smbus2 import SMBus
except ImportError:         # PC 에서도 변환 계수/함수는 import 할 수 있게 (sim.py)
    SMBus = None

PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
WHO_AM_I = 0x75

ACC_SCALE = 16384.0   # LSB per g      (+-2 g)
GYR_SCALE = 131.0     # LSB per deg/s  (+-250 dps)
TEMP_SCALE = 340.0    # degC = raw/340 + 36.53
TEMP_OFFSET = 36.53
WHO_OK = (0x68, 0x70, 0x71, 0x72, 0x73, 0x69)   # MPU6050/6500/9250 계열

BUSES = (1, 3, 4, 5, 6)     # find_all() 이 뒤지는 I2C 버스. 하드웨어 I2C 는 1 (GPIO2/3)
CAL_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "calibration", "imu_cal.json")

# imu_cal.json 의 up 축을 +Z 로 가져오는 회전. (x, y, z) -> (x', y', z')
REMAP = {
    "+z": lambda x, y, z: (x, y, z),
    "-z": lambda x, y, z: (x, -y, -z),
    "+x": lambda x, y, z: (-z, y, x),
    "-x": lambda x, y, z: (z, y, -x),
    "+y": lambda x, y, z: (x, -z, y),
    "-y": lambda x, y, z: (x, z, -y),
}


class MPU6050:
    def __init__(self, addr, bus=1, hz=100, label=None):
        if SMBus is None:
            raise RuntimeError("smbus2 가 없다. pip install smbus2")
        self.addr = addr
        self.busno = bus
        self.label = label or ("bus%d_%s" % (bus, hex(addr)))
        self.bus = SMBus(bus)

        self.bus.write_byte_data(addr, PWR_MGMT_1, 0x01)      # 깨우고 클럭은 자이로 X
        time.sleep(0.01)
        self.bus.write_byte_data(addr, CONFIG, 0x03)          # DLPF ~44Hz -> 출력 1kHz 기준
        div = max(0, min(255, int(round(1000.0 / max(1, hz))) - 1))
        self.bus.write_byte_data(addr, SMPLRT_DIV, div)
        self.bus.write_byte_data(addr, GYRO_CONFIG, 0x00)     # +-250 dps
        self.bus.write_byte_data(addr, ACCEL_CONFIG, 0x00)    # +-2 g
        time.sleep(0.05)
        self.actual_hz = 1000.0 / (1 + div)
        self.smplrt_div = div

    def who_am_i(self):
        return self.bus.read_byte_data(self.addr, WHO_AM_I)

    def check(self):
        w = self.who_am_i()
        if w not in WHO_OK:
            raise RuntimeError("%s WHO_AM_I=%s - MPU-6050 이 아닌 것 같다" % (self.label, hex(w)))
        return w

    def read_raw(self):
        """(t_ns, ax, ay, az, gx, gy, gz, temp) raw LSB.
        14바이트를 한 번에 읽으므로 6축이 같은 순간의 값이다.
        타임스탬프는 읽기 직전에 찍는다 (I2C 전송 ~1ms)."""
        t_ns = time.monotonic_ns()
        d = self.bus.read_i2c_block_data(self.addr, ACCEL_XOUT_H, 14)

        def s(i):
            v = (d[i] << 8) | d[i + 1]
            return v - 65536 if v & 0x8000 else v

        return (t_ns, s(0), s(2), s(4), s(8), s(10), s(12), s(6))

    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass


def cal_key(bus, addr):
    """imu_cal.json 의 키. 예) bus1:0x68"""
    return "bus%d:%s" % (bus, hex(addr))


def find_cal(path=""):
    """imu_cal.json 경로. path 를 안 주면 calibration/imu_cal.json. 파일이 없으면 None."""
    p = path or CAL_PATH
    return os.path.abspath(p) if os.path.isfile(p) else None


def load_cal(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def detect_up(ax, ay, az):
    """중력이 실린 칩 축 (성분 절댓값이 가장 큰 축). 예) "+z" 칩이 위, "+x" 모듈이 세워짐."""
    comps = {"x": ax, "y": ay, "z": az}
    axis = max(comps, key=lambda k: abs(comps[k]))
    return ("+" if comps[axis] >= 0 else "-") + axis


def find_all():
    """WHO_AM_I 에 응답하는 MPU-6050 전부 [(bus, addr), ...]."""
    if SMBus is None:
        raise RuntimeError("smbus2 가 없다. pip install smbus2")
    found = []
    for bus in BUSES:
        if not os.path.exists("/dev/i2c-%d" % bus):
            continue
        try:
            with SMBus(bus) as b:
                for a in (0x68, 0x69):
                    try:
                        if b.read_byte_data(a, WHO_AM_I) in WHO_OK:
                            found.append((bus, a))
                    except OSError:
                        pass
        except OSError:
            pass
    return found


def to_units(raw, cal=None):
    """raw LSB (ax, ay, az, gx, gy, gz) -> (G, G, G, deg/s, deg/s, deg/s).
    cal 은 imu_cal.json 의 센서 항목. 바이어스를 빼고 up 축을 +Z 로 돌린다."""
    c = cal or {}
    off = [c.get(k, 0.0) for k in ("ax", "ay", "az", "gx", "gy", "gz")]
    rot = REMAP[c.get("up", "+z")]
    acc = rot(*[(raw[i] - off[i]) / ACC_SCALE for i in range(3)])
    gyr = rot(*[(raw[i] - off[i]) / GYR_SCALE for i in range(3, 6)])
    return acc + gyr


def tilt(ax, ay, az):
    """가속도(중력 방향)로 본 roll / pitch (도)."""
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
    return roll, pitch


def scales():
    """meta.json 에 넣을 변환 계수."""
    return {
        "acc_lsb_per_g": ACC_SCALE,
        "gyro_lsb_per_dps": GYR_SCALE,
        "temp_degC": "raw/%.0f + %.2f" % (TEMP_SCALE, TEMP_OFFSET),
        "accel_range_g": 2,
        "gyro_range_dps": 250,
        "dlpf_hz": 44,
    }
