#!/usr/bin/env python3
"""Read every detected HX711: raw stats, bridge health (gain 128 vs 64), and
grams using any calibration file that applies. Run after scan_hx711.py."""
import json
import os
import statistics
import time

from hx711_lib import HX711

D = os.path.dirname(os.path.abspath(__file__))
cells = json.load(open(os.path.join(D, "cells.json")))

cals = {}
for name in ("calibration_4cell_backup.json", "calibration.json", "calibration_single_cell_backup.json"):
    p = os.path.join(D, name)
    if os.path.exists(p):
        cals[name] = json.load(open(p))

for c in cells:
    dt, sck = c["dt"], c["sck"]
    print(f"=== HX711 DT=GPIO{dt} SCK=GPIO{sck} ===")
    res = {}
    for g in (128, 64, 128):
        hx = HX711(dt, sck, gain=g)
        hx._read_raw_once()
        v = [hx._read_raw_once() for _ in range(40)]
        hx.close()
        res.setdefault(g, []).append((statistics.median(v), statistics.pstdev(v)))
        time.sleep(0.1)
    m128 = statistics.mean(x[0] for x in res[128])
    sd128 = statistics.mean(x[1] for x in res[128])
    m64 = res[64][0][0]
    ratio = m128 / m64 if m64 else float("nan")
    print(f"  gain128 median={m128:9.0f} sd={sd128:6.1f}   gain64 median={m64:9.0f}   ratio={ratio:.2f} (bridge ~2.0 unless near zero)")
    for name, cal in cals.items():
        if cal.get("dt_pin") == dt:
            grams = (m128 - cal["offset"]) / cal["scale"]
            print(f"  by {name}: offset={cal['offset']:.0f} scale={cal['scale']:.2f} -> {grams:8.1f} g")
    if not any(cal.get("dt_pin") == dt for cal in cals.values()):
        print("  (no calibration on file for this pin pair)")
