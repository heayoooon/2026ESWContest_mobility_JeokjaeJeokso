#!/usr/bin/env python3
"""Calibrate each detected HX711 in turn: tare, then a known weight. Saves cal_cells.json."""
import json, os
from cells_lib import load_cells, D
cells=load_cells(); cal={}
for i,key,c,hx in cells:
    print(f"\n=== cell{i} (DT {key}) ===")
    input("  remove all load from THIS cell, press Enter...")
    off=hx.tare(30); print(f"  zero={off:.0f}")
    g=float(input("  hang a known weight on THIS cell, enter grams: "))
    input(f"  {g} g hanging? press Enter...")
    raw=hx.read_raw(30); sc=(raw-off)/g
    print(f"  raw={raw:.0f}  scale={sc:.2f} counts/g")
    cal[key]={"offset":off,"scale":sc,"grams":g,"sck":c["sck"]}
json.dump(cal,open(f"{D}/cal_cells.json","w"),indent=2)
print(f"\nsaved {D}/cal_cells.json   ->  python3 weigh_all.py")
for *_,hx in cells: hx.close()
