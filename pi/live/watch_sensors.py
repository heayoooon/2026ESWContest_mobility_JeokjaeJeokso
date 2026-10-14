#!/usr/bin/env python3
"""센서 값 동시에 보기 (센서 파이 · 같은 LAN 의 다른 파이 어디서든, 몇 개든).

sensor_feed.py 가 내보내는 멀티캐스트 239.255.42.21:5021 을 받아 화면에 찍는다. HX711 GPIO 를 건드리지 않으므로
트윈 송신 · 다른 보기 프로그램과 동시에 돌려도 된다. (보정 calibrate.py 처럼 원시 값이 필요한 일만 stop_live.sh 로 끄고 한다)

usage: python3 watch_sensors.py [--json]
"""
import json, socket, struct, sys

GROUP, PORT = '239.255.42.21', 5021
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('', PORT))
s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, struct.pack('4sl', socket.inet_aton(GROUP), socket.INADDR_ANY))
raw = '--json' in sys.argv
LV = ('정상', '주의', '경고', '위험')
f = lambda v: '   -  ' if v is None else '%6.1f' % v  # noqa: E731
while True:
    d = json.loads(s.recv(65536))
    if raw:
        print(json.dumps(d, ensure_ascii=False), flush=True)
        continue
    lv = d.get('risk')
    print('%7.1fs  NW %s NE %s SW %s SE %s g | c1 %s c2 %s N | 적재 %s kg | 종합 %s  %s' % (
        d.get('t_s', 0), f(d.get('NW_g')), f(d.get('NE_g')), f(d.get('SW_g')), f(d.get('SE_g')),
        f(d.get('c1_N')), f(d.get('c2_N')), f(d.get('load_kg')), LV[lv] if lv is not None else '-',
        d.get('flags', '')), flush=True)
