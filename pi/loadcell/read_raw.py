#!/usr/bin/env python3
"""
Step 1 - wiring check.

Prints raw HX711 values continuously. If wiring is correct you should see
stable-ish numbers that jump clearly when you press on the load cell.

  DT_PIN / SCK_PIN default to GPIO5 / GPIO6 (BCM numbering).
  Override:  DT_PIN=5 SCK_PIN=6 python3 read_raw.py
"""
import os
import time
from hx711_lib import HX711

DT = int(os.environ.get("DT_PIN", 5))
SCK = int(os.environ.get("SCK_PIN", 6))

print(f"HX711 raw read  (DT=GPIO{DT}, SCK=GPIO{SCK})")
print("Press the load cell to see the value change. Ctrl+C to stop.\n")

hx = HX711(DT, SCK)
try:
    while True:
        raw = hx.read_raw(times=5)
        print(f"raw = {raw:>12.0f}")
        time.sleep(0.3)
except KeyboardInterrupt:
    print("\nstopped.")
finally:
    hx.close()
