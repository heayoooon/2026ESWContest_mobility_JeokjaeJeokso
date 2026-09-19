#!/usr/bin/env python3
"""Run with NO weight on the cell. Uses loaded_snapshot.json (162g reading) + fresh tare -> calibration.json"""
import json, os, statistics
from hx711_lib import HX711
d=os.path.dirname(os.path.abspath(__file__))
s=json.load(open(f"{d}/loaded_snapshot.json"))
hx=HX711(s["dt_pin"],s["sck_pin"])
v=[hx._read_raw_once() for _ in range(60)]; hx.close()
tare=statistics.median(v); sd=statistics.pstdev(v)
scale=(s["raw_with_weight"]-tare)/s["known_grams"]
json.dump({"offset":tare,"scale":scale,"dt_pin":s["dt_pin"],"sck_pin":s["sck_pin"],
           "calibrated_with_grams":s["known_grams"]},open(f"{d}/calibration.json","w"),indent=2)
print(f"tare={tare:.0f} (std {sd:.1f})  loaded={s["raw_with_weight"]}  -> scale={scale:.2f} counts/g")
print("saved calibration.json. Put the 162g back and run: python3 weigh.py")
