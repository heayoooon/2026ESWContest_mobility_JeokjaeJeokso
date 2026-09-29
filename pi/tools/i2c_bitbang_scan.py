#!/usr/bin/env python3
"""Software (bit-banged) I2C scan on GPIO2 (SDA) / GPIO3 (SCL) using lgpio.

Bypasses the hardware I2C controller entirely, so it tells whether the sensors
are really on the wires. Open-drain is emulated: drive low = output 0,
release = input (the modules' pull-ups pull the line high).
Reads WHO_AM_I (reg 0x75) from 0x68 and 0x69.

NOTE: this re-muxes GPIO2/3 to plain GPIO. Reboot (or dtparam i2c_arm off/on)
afterwards to give the pins back to the hardware controller.
"""
import sys
import time

import lgpio

SDA, SCL = 2, 3
H = lgpio.gpiochip_open(0)
D = 0.00005  # half clock period (~10 kHz)


def rel(pin):
    try:
        lgpio.gpio_free(H, pin)
    except Exception:
        pass
    lgpio.gpio_claim_input(H, pin)


def low(pin):
    try:
        lgpio.gpio_free(H, pin)
    except Exception:
        pass
    lgpio.gpio_claim_output(H, pin, 0)


def rd(pin):
    return lgpio.gpio_read(H, pin)


def start():
    rel(SDA); rel(SCL); time.sleep(D)
    low(SDA); time.sleep(D)
    low(SCL); time.sleep(D)


def stop():
    low(SDA); time.sleep(D)
    rel(SCL); time.sleep(D)
    rel(SDA); time.sleep(D)


def wbit(b):
    (rel if b else low)(SDA); time.sleep(D)
    rel(SCL); time.sleep(D)
    low(SCL); time.sleep(D)


def rbit():
    rel(SDA); time.sleep(D)
    rel(SCL); time.sleep(D)
    b = rd(SDA)
    low(SCL); time.sleep(D)
    return b


def wbyte(v):
    for i in range(7, -1, -1):
        wbit((v >> i) & 1)
    return rbit() == 0  # ACK = slave pulled SDA low


def rbyte(ack):
    v = 0
    for _ in range(8):
        v = (v << 1) | rbit()
    wbit(0 if ack else 1)
    return v


def read_reg(addr, reg):
    start()
    if not wbyte(addr << 1):
        stop(); return None
    wbyte(reg)
    start()  # repeated start
    if not wbyte((addr << 1) | 1):
        stop(); return None
    v = rbyte(False)
    stop()
    return v


rel(SDA); rel(SCL); time.sleep(0.01)
print(f"idle levels: SDA={rd(SDA)} SCL={rd(SCL)}  (both should be 1)")
if rd(SDA) == 0 or rd(SCL) == 0:
    print("-> a line is held LOW even with the controller bypassed: wiring short / a sensor pin in the wrong row")

print("--- ACK scan 0x08..0x77 (software I2C) ---")
acked = []
for a in range(0x08, 0x78):
    start()
    if wbyte(a << 1):
        acked.append(hex(a))
    stop()
print("  devices that ACK:", acked or "none")

for a in (0x68, 0x69):
    w = read_reg(a, 0x75)
    print(f"  {hex(a)} WHO_AM_I = {'no response' if w is None else hex(w)}")

def read_block(addr, reg, n):
    start()
    if not wbyte(addr << 1):
        stop(); return None
    wbyte(reg)
    start()
    if not wbyte((addr << 1) | 1):
        stop(); return None
    out = [rbyte(i < n - 1) for i in range(n)]
    stop()
    return out


def s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v & 0x8000 else v


if "--data" in sys.argv:
    import statistics
    for a in (0x68, 0x69):
        # wake the chip first (PWR_MGMT_1 = 1) so accel/gyro run
        start()
        if wbyte(a << 1):
            wbyte(0x6B); wbyte(0x01)
        stop()
        time.sleep(0.05)
        S = {k: [] for k in ("ax", "ay", "az", "gx", "gy", "gz")}
        for _ in range(25):
            d = read_block(a, 0x3B, 14)
            if d is None:
                break
            S["ax"].append(s16(d[0], d[1])); S["ay"].append(s16(d[2], d[3])); S["az"].append(s16(d[4], d[5]))
            S["gx"].append(s16(d[8], d[9])); S["gy"].append(s16(d[10], d[11])); S["gz"].append(s16(d[12], d[13]))
        if not S["ax"]:
            print(f"  {hex(a)}: no data"); continue
        g = (statistics.mean(S["ax"])**2 + statistics.mean(S["ay"])**2 + statistics.mean(S["az"])**2) ** 0.5 / 16384
        sd = {k: round(statistics.pstdev(v), 1) for k, v in S.items()}
        mean = {k: round(statistics.mean(v)) for k, v in S.items()}
        print(f"  {hex(a)} data x{len(S['ax'])}: mean={mean} sd={sd} |accel|={g:.3f} g")
        if 0.9 < g < 1.1 and max(sd["gx"], sd["gy"], sd["gz"]) < 40:
            print("     -> clean single sensor (one chip at this address)")
        else:
            print("     -> garbled/unstable -> likely TWO chips at this address (AD0 of sensor 2 not at 3.3V) or sensor moving")

rel(SDA); rel(SCL)
lgpio.gpiochip_close(H)
print("\nverdict:")
if any(x in acked for x in ("0x68", "0x69")):
    print("  sensors ARE on the wires -> the hardware I2C controller / pin mux is the problem -> reboot the pi")
else:
    print("  nothing answers even by software -> wiring problem on GPIO2/3 rows (SDA/SCL crossed, AD0 in a data row, or a wire not in the same 5-hole row)")
