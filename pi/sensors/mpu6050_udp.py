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
import select
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
        self.dropped = 0
        self._last_t = None
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

    def drain(self):
        """기다리지 않고, 지금까지 쌓인 패킷을 전부 읽는다. [(t_ns, ax..temp), ...] (없으면 []).

        파이프라인(acquisition/source.py)이 HX711 클럭 사이에서 부르므로 절대 블로킹하면 안 된다.
        한꺼번에 여러 개가 쌓였으면 패킷마다 받은 시각이 같아져 자이로 적분의 dt 가 0 이 되므로,
        지난 호출 시각부터 지금까지를 패킷 수로 나눠 시각을 펼친다."""
        out = []
        while select.select([self.sock], [], [], 0)[0]:
            data, _ = self.sock.recvfrom(256)
            try:
                d = json.loads(data.decode("utf-8"))
                row = (d["ax"], d["ay"], d["az"], d["gx"], d["gy"], d["gz"], d["temp"])
            except (ValueError, KeyError):          # 깨진/다른 패킷은 버린다
                continue
            seq = d.get("seq")
            if seq is not None and self.last_seq is not None and seq - self.last_seq > 1:
                self.dropped += seq - self.last_seq - 1
            self.last_seq = seq if seq is not None else self.last_seq
            out.append(row)
        if not out:
            return []
        now = time.monotonic_ns()
        last = self._last_t if self._last_t is not None else now
        self._last_t = now
        n = len(out)
        return [(last + (now - last) * (i + 1) // n,) + r for i, r in enumerate(out)]

    def close(self):
        self.sock.close()
