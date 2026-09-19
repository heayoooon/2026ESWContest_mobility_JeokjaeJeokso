#!/usr/bin/env python3
import time, sys
from hx711_lib import HX711
dur=float(sys.argv[1]) if len(sys.argv)>1 else 180
hx=HX711(5,6); t0=time.time()
with open("log.csv","w") as f:
    f.write("t,raw\n")
    while time.time()-t0<dur:
        f.write(f"{time.time()-t0:.2f},{hx._read_raw_once()}\n"); f.flush()
        time.sleep(0.08)
hx.close()
