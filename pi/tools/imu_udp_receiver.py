#!/usr/bin/env python3
"""ESP32 무선 MPU-6050 실시간 확인 (UDP 수신).

    python3 tools/imu_udp_receiver.py                # 보정 적용 roll / pitch
    python3 tools/imu_udp_receiver.py --port 5005    # .ino 의 PI_PORT 와 맞출 것
    python3 tools/imu_udp_receiver.py --raw          # 보정/스케일 없이 raw
"""
import argparse
import os
import socket
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # pi/

from sensors.mpu6050 import TEMP_OFFSET, TEMP_SCALE, find_cal, load_cal, tilt, to_units  # noqa: E402
from sensors.mpu6050_udp import MPU6050UDP  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=5005, help="ESP32 .ino 의 PI_PORT 와 일치해야 함")
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=0x68, help="MPU6050 주소 (0x68/0x69)")
    ap.add_argument("--raw", action="store_true", help="보정/스케일 없이 raw 값만 출력")
    args = ap.parse_args()

    m = MPU6050UDP(addr=args.addr, port=args.port)
    cal_path = find_cal()
    cal = (load_cal(cal_path) if cal_path else {}).get(m.key)
    print(">> UDP %d번 포트에서 ESP32 패킷 대기 중... (key=%s, 보정 %s)"
          % (args.port, m.key, "있음" if cal else "없음"))
    last_seq = None
    try:
        while True:
            try:
                r = m.read_raw()
            except socket.timeout:
                print("⚠️  1초 넘게 패킷 없음 (ESP32 WiFi 끊김?)")
                continue

            dropped = ""
            if last_seq is not None and m.last_seq is not None:
                gap = m.last_seq - last_seq
                if gap > 1:
                    dropped = "  (패킷 %d개 유실)" % (gap - 1)
            last_seq = m.last_seq

            temp = r[7] / TEMP_SCALE + TEMP_OFFSET
            if args.raw:
                print("ax=%6d ay=%6d az=%6d  gx=%6d gy=%6d gz=%6d  temp=%.1fC%s"
                      % (r[1], r[2], r[3], r[4], r[5], r[6], temp, dropped),
                      end="\r", flush=True)
            else:
                ax, ay, az = to_units(r[1:7], cal)[:3]
                roll, pitch = tilt(ax, ay, az)
                print("roll=%+6.1f pitch=%+6.1f  ax=%+.2f ay=%+.2f az=%+.2f  temp=%.1fC%s   "
                      % (roll, pitch, ax, ay, az, temp, dropped), end="\r", flush=True)
    except KeyboardInterrupt:
        print()
    finally:
        m.close()


if __name__ == "__main__":
    main()
