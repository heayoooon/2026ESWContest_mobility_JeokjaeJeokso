#!/usr/bin/env python3
"""
Step 2 - calibration.

  1. Remove all weight, press Enter  -> measures the zero (tare).
  2. Put a KNOWN weight on, type its grams, press Enter.
  3. Saves offset + scale to calibration.json for weigh.py to use.

  DT_PIN / SCK_PIN default to GPIO5 / GPIO6.
"""
import os
import json
from hx711_lib import HX711

DT = int(os.environ.get("DT_PIN", 5))
SCK = int(os.environ.get("SCK_PIN", 6))
OUT = os.path.join(os.path.dirname(__file__), "calibration.json")

hx = HX711(DT, SCK)
try:
    input("Remove ALL weight from the load cell, then press Enter...")
    offset = hx.tare(times=30)
    print(f"  zero (offset) = {offset:.0f}")

    grams = float(input("\nPlace a known weight, enter its grams (e.g. 100): "))
    input(f"Make sure the {grams} g is resting on it, then press Enter...")
    raw = hx.read_raw(times=30)

    scale = (raw - offset) / grams
    print(f"  raw with weight = {raw:.0f}")
    print(f"  scale = {scale:.3f} raw units per gram")

    with open(OUT, "w") as f:
        json.dump({"offset": offset, "scale": scale,
                   "dt_pin": DT, "sck_pin": SCK}, f, indent=2)
    print(f"\nSaved -> {OUT}")
    print("Now run:  python3 weigh.py")
finally:
    hx.close()
