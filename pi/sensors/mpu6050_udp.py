"""ESP32 에 물린 MPU-6050 을 WiFi(UDP) 로 받는 드라이버.

짝: hardware/esp32_mpu6050_udp/esp32_mpu6050_udp.ino. ESP32 가 레지스터 raw 값
(ax, ay, az, gx, gy, gz, temp, seq)을 JSON 한 줄로 UDP 에 보낸다.

read_raw() 는 mpu6050.MPU6050.read_raw() 와 같은 모양
(t_ns, ax, ay, az, gx, gy, gz, temp) 을 돌려주므로, 물리 단위는 같은 to_units() 로 바꾼다.
보정 키는 "esp32:<주소>" (예: esp32:0x68). calibration/imu_calibrate.py 는 파이에 직접
물린 센서만 찾으므로 이 항목은 imu_cal.json 에 손으로 넣어야 한다. 없으면 보정 없이 쓴다.

실시간 확인: python3 tools/imu_udp_receiver.py
"""
import json
import socket
import time


def udp_key(addr):
    """imu_cal.json 의 키. 예) esp32:0x68"""
    return "esp32:%s" % hex(addr)


class MPU6050UDP:
    def __init__(self, addr=0x68, port=5005, timeout=1.0, label=None):
        self.addr = addr
        self.port = port
        self.key = udp_key(addr)
        self.label = label or self.key
        self.last_seq = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(timeout)

    def read_raw(self):
        """다음 패킷 하나. (t_ns, ax, ay, az, gx, gy, gz, temp) raw LSB.

        패킷에 시각이 없어서 t_ns 는 파이가 **받은** 시각이다. WiFi 전송 지연만큼
        실제 측정보다 늦다. timeout 안에 패킷이 없으면 socket.timeout (ESP32 끊김/꺼짐)."""
        data, _ = self.sock.recvfrom(256)
        t_ns = time.monotonic_ns()
        d = json.loads(data.decode("utf-8"))
        self.last_seq = d.get("seq")
        return (t_ns, d["ax"], d["ay"], d["az"], d["gx"], d["gy"], d["gz"], d["temp"])

    def close(self):
        self.sock.close()
