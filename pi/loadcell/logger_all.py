#!/usr/bin/env python3
"""Log raw values of every HX711 in cells.json to log_all.csv (~10 Hz).
  python3 logger_all.py [seconds]   (default 600)
Columns: t, then one raw column per module named dt<N>."""
import json
import os
import sys
import time

from hx711_lib import HX711

D = os.path.dirname(os.path.abspath(__file__))
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 600
cells = json.load(open(os.path.join(D, "cells.json")))
hx = [(c["dt"], HX711(c["dt"], c["sck"])) for c in cells]
t0 = time.time()
with open(os.path.join(D, "log_all.csv"), "w") as f:
    f.write("t," + ",".join(f"dt{d}" for d, _ in hx) + "\n")
    while time.time() - t0 < dur:
        vals = [h._read_raw_once() for _, h in hx]
        f.write(f"{time.time() - t0:.2f}," + ",".join(str(v) for v in vals) + "\n")
        f.flush()
        time.sleep(0.08)
for _, h in hx:
    h.close()
