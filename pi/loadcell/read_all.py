#!/usr/bin/env python3
"""Live raw values of every detected HX711 side by side (run scan_hx711.py first). Ctrl+C to stop."""
import time
from cells_lib import load_cells
cells=load_cells()
print("  ".join(f"cell{i} DT={k}/SCK=GPIO{c[sck]}" for i,k,c,_ in cells)); print()
try:
    while True:
        print("  ".join(f"cell{i}: {hx.read_raw(3):>10.0f}" for i,k,c,hx in cells), flush=True); time.sleep(0.15)
except KeyboardInterrupt: pass
finally:
    for *_,hx in cells: hx.close()
