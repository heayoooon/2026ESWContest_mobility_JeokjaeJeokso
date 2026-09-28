#!/usr/bin/env python3
"""UDP receiver for the wireless MPU-6050 (wired to an ESP32, sent over WiFi).

Pairs with hardware/esp32_mpu6050_udp/esp32_mpu6050_udp.ino. The ESP32 sends
raw register values (same 14-byte layout as mpu6050_lib.MPU6050.read_raw())
as JSON over UDP; this applies the exact same scale/bias/remap logic as the
wired sensors so it can be dropped in next to them.

Calibration key: "esp32:<addr hex>" (e.g. "esp32:0x68") in imu_cal.json.
imu_calibrate.py only knows about locally-wired sensors from find_all(), so
for now this reads uncalibrated (off=0, up=+z) unless you hand-add an
"esp32:0x68" entry to imu_cal.json yourself.

Usage:
    python3 imu_udp_receiver.py                  # print live values
    python3 imu_udp_receiver.py --port 5005       # custom port (must match .ino)
"""
import argparse
import json
import math
import socket
import time

from mpu6050_lib import ACC_SCALE, GYR_SCALE, REMAP, CAL
import json as _json
import os


class MPU6050UDP:
    """Same read()/tilt()/key/close() interface as mpu6050_lib.MPU6050,
    but fed by UDP packets from an ESP32 instead of a local I2C bus."""

    def __init__(self, addr=0x68, port=5005, timeout=1.0):
        self.addr = addr
        self.key = f"esp32:{hex(addr)}"  # e.g. "esp32:0x68" -- distinct key in imu_cal.json
        self.port = port
        self.timeout = timeout
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(timeout)

        self.off = {"ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0}
        self.up = "+z"
        if os.path.exists(CAL):
            cal = _json.load(open(CAL))
            c = cal.get(self.key)
            if c:
                self.off = {k: c.get(k, 0) for k in ("ax", "ay", "az", "gx", "gy", "gz")}
                self.up = c.get("up", "+z")

        self._last_seq = None
        self._last_raw = None

    def read_raw(self):
        """Blocks up to self.timeout for the next packet. Raises socket.timeout
        if the ESP32 stopped sending (WiFi drop, powered off, etc.)."""
        data, _addr = self.sock.recvfrom(256)
        d = json.loads(data.decode("utf-8"))
        self._last_seq = d.get("seq")
        raw = {"ax": d["ax"], "ay": d["ay"], "az": d["az"],
               "temp": d["temp"] / 340.0 + 36.53,
               "gx": d["gx"], "gy": d["gy"], "gz": d["gz"]}
        self._last_raw = raw
        return raw

    def read(self):
        """Bias-corrected, scaled, and rotated -- identical math to
        mpu6050_lib.MPU6050.read()."""
        r = self.read_raw()
        o = self.off
        rot = REMAP[self.up]
        ax, ay, az = rot((r["ax"] - o["ax"]) / ACC_SCALE,
                         (r["ay"] - o["ay"]) / ACC_SCALE,
                         (r["az"] - o["az"]) / ACC_SCALE)
        gx, gy, gz = rot((r["gx"] - o["gx"]) / GYR_SCALE,
                         (r["gy"] - o["gy"]) / GYR_SCALE,
                         (r["gz"] - o["gz"]) / GYR_SCALE)
        return {"ax": ax, "ay": ay, "az": az, "gx": gx, "gy": gy, "gz": gz, "temp": r["temp"]}

    @staticmethod
    def tilt(a):
        roll = math.degrees(math.atan2(a["ay"], a["az"]))
        pitch = math.degrees(math.atan2(-a["ax"], math.sqrt(a["ay"] ** 2 + a["az"] ** 2)))
        return roll, pitch

    def close(self):
        self.sock.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=5005, help="ESP32 .ino 의 PI_PORT 와 일치해야 함")
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=0x68, help="MPU6050 주소 (0x68/0x69)")
    ap.add_argument("--raw", action="store_true", help="보정/스케일 없이 raw 값만 출력")
    args = ap.parse_args()

    m = MPU6050UDP(addr=args.addr, port=args.port)
    print(f">> UDP {args.port}번 포트에서 ESP32 패킷 대기 중... (key={m.key})")
    last_seq = None
    try:
        while True:
            try:
                d = m.read_raw() if args.raw else m.read()
            except socket.timeout:
                print("⚠️  1초 넘게 패킷 없음 (ESP32 WiFi 끊김?)")
                continue

            dropped = ""
            if last_seq is not None and m._last_seq is not None:
                gap = m._last_seq - last_seq
                if gap > 1:
                    dropped = f"  (패킷 {gap - 1}개 유실)"
            last_seq = m._last_seq

            if args.raw:
                print(f"ax={d['ax']:6d} ay={d['ay']:6d} az={d['az']:6d}  "
                      f"gx={d['gx']:6d} gy={d['gy']:6d} gz={d['gz']:6d}  "
                      f"temp={d['temp']:.1f}C{dropped}", end="\r", flush=True)
            else:
                roll, pitch = m.tilt(d)
                print(f"roll={roll:+6.1f} pitch={pitch:+6.1f}  "
                      f"ax={d['ax']:+.2f} ay={d['ay']:+.2f} az={d['az']:+.2f}  "
                      f"temp={d['temp']:.1f}C{dropped}   ", end="\r", flush=True)
    except KeyboardInterrupt:
        print()
    finally:
        m.close()


if __name__ == "__main__":
    main()
