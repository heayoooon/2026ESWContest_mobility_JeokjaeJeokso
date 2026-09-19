#!/usr/bin/env python3
"""Find every HX711 on the 40-pin header: which GPIO is DT and which is SCK."""
import time, lgpio, json, sys
PINS=[p for p in range(4,28)]  # skip GPIO2/3 (I2C for the IMUs)
h=lgpio.gpiochip_open(0)

def read_with_pull(pin, pull):
    lgpio.gpio_free(h,pin) if False else None
    lgpio.gpio_claim_input(h,pin,pull); time.sleep(0.005)
    v=lgpio.gpio_read(h,pin); lgpio.gpio_free(h,pin); return v

# 1) DT candidates = actively driven pins (same value regardless of pull)
driven=[]
for p in PINS:
    try:
        up=read_with_pull(p,lgpio.SET_PULL_UP); dn=read_with_pull(p,lgpio.SET_PULL_DOWN)
        if up==dn: driven.append(p)
    except Exception as e: print("skip",p,e)
print("driven (DT candidates):",driven)

# 2) pair: for each other pin, drive low (wake HX711), pulse 25x, see which DT goes LOW->HIGH
for p in driven: lgpio.gpio_claim_input(h,p)
pairs={}
for sck in PINS:
    if sck in driven: continue
    try: lgpio.gpio_claim_output(h,sck,0)
    except Exception as e: print("skip out",sck,e); continue
    time.sleep(0.5)                      # power-up + first conversion
    before={d:lgpio.gpio_read(h,d) for d in driven}
    for _ in range(25):
        lgpio.gpio_write(h,sck,1); lgpio.gpio_write(h,sck,0)
    after={d:lgpio.gpio_read(h,d) for d in driven}
    hit=[d for d in driven if before[d]==0 and after[d]==1]
    if hit: print(f"SCK GPIO{sck} -> DT {hit}"); 
    for d in hit: pairs[d]=sck
    lgpio.gpio_free(h,sck)
lgpio.gpiochip_close(h)
print("PAIRS", json.dumps({f"GPIO{d}":f"GPIO{s}" for d,s in pairs.items()}))
json.dump([{"dt":d,"sck":s} for d,s in sorted(pairs.items())], open("/home/abcd/loadcell/cells.json","w"), indent=1)
