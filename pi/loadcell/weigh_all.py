#!/usr/bin/env python3
"""Live grams for every calibrated cell. --tare re-zeros all cells at start (unloaded)."""
import sys, time
from cells_lib import load_cells
cells=load_cells()
if "--tare" in sys.argv:
    for *_,hx in cells: hx.tare(30)
try:
    while True:
        print("   ".join(f"cell{i}: {hx.get_units(15):8.1f} g" for i,k,c,hx in cells), end="\r", flush=True); time.sleep(0.15)
except KeyboardInterrupt: print()
finally:
    for *_,hx in cells: hx.close()
