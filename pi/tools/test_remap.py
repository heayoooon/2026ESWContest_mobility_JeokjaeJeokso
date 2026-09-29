#!/usr/bin/env python3
"""Unit test for the axis remap in sensors/mpu6050: gravity maps to +Z for every 'up',
rotations are right-handed (det = +1), and a physical tilt reads the same angle
whatever way the module is mounted. Runs on a PC too (no sensor needed)."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # pi/

from sensors.mpu6050 import REMAP, detect_up, tilt  # noqa: E402

ok = True
for up, (x, y, z) in {"+z": (0, 0, 1), "-z": (0, 0, -1), "+x": (1, 0, 0),
                      "-x": (-1, 0, 0), "+y": (0, 1, 0), "-y": (0, -1, 0)}.items():
    assert detect_up(x * 16000, y * 16000, z * 16000) == up, up
    r = tuple(round(v, 6) for v in REMAP[up](x, y, z))
    cols = [REMAP[up](*e) for e in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    M = [[cols[j][i] for j in range(3)] for i in range(3)]
    det = (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
           - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
           + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]))
    good = r == (0, 0, 1) and abs(det - 1) < 1e-9
    ok &= good
    print(f"up={up}: gravity->{r} det={det:+.0f} {'OK' if good else 'BAD'}")


def inverse(up):
    """Inverse rotation: from remapped frame back to chip frame."""
    cols = [REMAP[up](*e) for e in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    # rotation matrix R with columns = images of basis vectors; inverse = transpose
    return lambda X, Y, Z: tuple(cols[i][0] * X + cols[i][1] * Y + cols[i][2] * Z for i in range(3))


# physical +30 deg pitch and +20 deg roll, expressed in a flat (+z) frame
for up in REMAP:
    p, rl = math.radians(30), math.radians(20)
    g = (-math.sin(p), math.cos(p) * math.sin(rl), math.cos(p) * math.cos(rl))  # gravity in remapped frame
    chip = inverse(up)(*g)                       # what the chip would measure if mounted with this 'up'
    back = REMAP[up](*chip)                      # what read() would return
    roll, pitch = tilt(back[0], back[1], back[2])
    good = abs(pitch - 30) < 1e-6 and abs(roll - 20) < 1e-6
    ok &= good
    print(f"mount up={up}: physical pitch 30 / roll 20 -> reads pitch={pitch:+.3f} roll={roll:+.3f} {'OK' if good else 'BAD'}")

print("\nALL OK" if ok else "\nPROBLEM")
