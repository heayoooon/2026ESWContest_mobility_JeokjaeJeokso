#!/usr/bin/env python3
"""
Step 3 - live weighing.

Reads calibration.json (made by calibrate.py) and prints live grams.
Press 't' + Enter is not needed; it re-tares automatically on start only if
--tare is passed.

  python3 weigh.py            # use saved zero
  python3 weigh.py --tare     # re-zero now, then weigh
"""
import os
import sys
import json
import time
from hx711_lib import HX711

CAL = os.path.join(os.path.dirname(__file__), "calibration.json")
if not os.path.exists(CAL):
    sys.exit("No calibration.json found. Run:  python3 calibrate.py  first.")

with open(CAL) as f:
    cal = json.load(f)

DT = int(os.environ.get("DT_PIN", cal.get("dt_pin", 5)))
SCK = int(os.environ.get("SCK_PIN", cal.get("sck_pin", 6)))

hx = HX711(DT, SCK)
hx.offset = cal["offset"]
hx.scale = cal["scale"]

if "--tare" in sys.argv:
    print("Re-zeroing... remove weight.")
    time.sleep(1)
    hx.tare(times=30)

print(f"Live weight  (DT=GPIO{DT}, SCK=GPIO{SCK})   Ctrl+C to stop.\n")
try:
    while True:
        grams = hx.get_units(times=20)
        bar = "#" * min(40, max(0, int(abs(grams) / 10)))
        print(f"{grams:8.1f} g  {bar}", end="\r", flush=True)
        time.sleep(0.15)
except KeyboardInterrupt:
    print("\nstopped.")
finally:
    hx.close()
