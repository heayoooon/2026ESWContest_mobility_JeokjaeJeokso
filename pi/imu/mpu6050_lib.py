"""Minimal MPU-6050 driver over smbus2, with mounting-orientation support.

Sensors are identified by (bus, addr). Hardware I2C is bus 1 (GPIO2/3).
Addresses: 0x68 (AD0 low) / 0x69 (AD0 high).

Calibration file imu_cal.json, per "bus<N>:<addr>":
  ax..gz : true sensor biases in raw LSB (small numbers)
  up     : which chip axis points UP in the rest pose ("+z" flat chip-up,
           "+x" when the module stands on its side, ...). read() rotates
           the axes so that this axis becomes +Z, so roll/pitch are always
           measured relative to the rest pose with correct geometry.
"""
import json
import math
import os
import time

from smbus2 import SMBus

PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
WHO_AM_I = 0x75
ACC_SCALE = 16384.0  # LSB per g  at +-2 g
GYR_SCALE = 131.0    # LSB per deg/s at +-250 dps
WHO_OK = (0x68, 0x69, 0x70, 0x71, 0x72, 0x73)

BUSES = (1, 3, 4, 5, 6)
D = os.path.dirname(os.path.abspath(__file__))
CAL = os.path.join(D, "imu_cal.json")

# rotation that brings the given chip axis to +Z: returns (x', y', z') from (x, y, z)
REMAP = {
    "+z": lambda x, y, z: (x, y, z),
    "-z": lambda x, y, z: (x, -y, -z),
    "+x": lambda x, y, z: (-z, y, x),
    "-x": lambda x, y, z: (z, y, -x),
    "+y": lambda x, y, z: (x, -z, y),
    "-y": lambda x, y, z: (x, z, -y),
}


def key(bus, addr):
    return f"bus{bus}:{hex(addr)}"


def detect_up(ax, ay, az):
    """Which chip axis carries gravity (largest |component|)."""
    comps = {"x": ax, "y": ay, "z": az}
    axis = max(comps, key=lambda k: abs(comps[k]))
    return ("+" if comps[axis] >= 0 else "-") + axis


class MPU6050:
    def __init__(self, addr, bus=1):
        self.addr = addr
        self.busno = bus
        self.key = key(bus, addr)
        self.bus = SMBus(bus)
        self.bus.write_byte_data(addr, PWR_MGMT_1, 0x01)    # wake, clock = gyro X
        self.bus.write_byte_data(addr, SMPLRT_DIV, 0x07)    # 1 kHz / 8 = 125 Hz
        self.bus.write_byte_data(addr, CONFIG, 0x03)        # DLPF ~44 Hz
        self.bus.write_byte_data(addr, GYRO_CONFIG, 0x00)   # +-250 dps
        self.bus.write_byte_data(addr, ACCEL_CONFIG, 0x00)  # +-2 g
        time.sleep(0.05)
        self.off = {"ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0}
        self.up = "+z"
        cal = json.load(open(CAL)) if os.path.exists(CAL) else {}
        c = cal.get(self.key)
        if c:
            self.off = {k: c.get(k, 0) for k in ("ax", "ay", "az", "gx", "gy", "gz")}
            self.up = c.get("up", "+z")

    def who_am_i(self):
        return self.bus.read_byte_data(self.addr, WHO_AM_I)

    def read_raw(self):
        d = self.bus.read_i2c_block_data(self.addr, ACCEL_XOUT_H, 14)

        def s(i):
            v = (d[i] << 8) | d[i + 1]
            return v - 65536 if v & 0x8000 else v

        return {"ax": s(0), "ay": s(2), "az": s(4), "temp": s(6) / 340.0 + 36.53,
                "gx": s(8), "gy": s(10), "gz": s(12)}

    def read(self):
        """Bias-corrected, scaled, and rotated so the rest-pose 'up' axis is +Z."""
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
        """roll / pitch in degrees from the accelerometer (gravity direction)."""
        roll = math.degrees(math.atan2(a["ay"], a["az"]))
        pitch = math.degrees(math.atan2(-a["ax"], math.sqrt(a["ay"] ** 2 + a["az"] ** 2)))
        return roll, pitch

    def close(self):
        self.bus.close()


def find_all():
    """Return [(bus, addr), ...] for every MPU-6050 that answers WHO_AM_I."""
    found = []
    for bus in BUSES:
        if not os.path.exists(f"/dev/i2c-{bus}"):
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
