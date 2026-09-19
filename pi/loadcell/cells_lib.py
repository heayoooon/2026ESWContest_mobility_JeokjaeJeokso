"""Multi-HX711 helper: loads cells.json (from scan_hx711.py) and cal_cells.json."""
import json, os
from hx711_lib import HX711
D=os.path.dirname(os.path.abspath(__file__))
def load_cells():
    cells=json.load(open(f"{D}/cells.json"))
    cal={}
    if os.path.exists(f"{D}/cal_cells.json"): cal=json.load(open(f"{D}/cal_cells.json"))
    out=[]
    for i,c in enumerate(cells, start=1):
        hx=HX711(c["dt"],c["sck"]); key="GPIO%d" % c["dt"]
        if key in cal: hx.offset=cal[key]["offset"]; hx.scale=cal[key]["scale"]
        out.append((i,key,c,hx))
    return out
