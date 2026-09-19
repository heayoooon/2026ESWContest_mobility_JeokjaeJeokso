#!/usr/bin/env python3
"""Live raw viewer for wiring checks. Shows raw value, change from start, and a bar.
Pull / load the cell and watch the bar. Ctrl+C to stop.
  DT_PIN / SCK_PIN default 5 / 6."""
import os, time, statistics
from hx711_lib import HX711
DT=int(os.environ.get("DT_PIN",5)); SCK=int(os.environ.get("SCK_PIN",6))
hx=HX711(DT,SCK)
print(f"DT=GPIO{DT} SCK=GPIO{SCK}  measuring zero... keep the cell unloaded 2s")
zero=statistics.median([hx._read_raw_once() for _ in range(20)])
print(f"zero = {zero:.0f}   now pull / hang weight.  (Ctrl+C to stop)\n")
try:
    while True:
        r=statistics.median([hx._read_raw_once() for _ in range(3)])
        d=r-zero; n=min(50,int(abs(d)/200))
        bar=("+" if d>0 else "-")*n
        print(f"raw={r:8.0f}  delta={d:+8.0f}  {bar}", flush=True)
        time.sleep(0.1)
except KeyboardInterrupt: print("\nstopped")
finally: hx.close()
