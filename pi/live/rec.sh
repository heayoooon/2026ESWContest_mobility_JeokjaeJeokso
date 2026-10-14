#!/bin/bash
# 실측 구간 저장 (센서 파이). run_live.sh 로 송신이 켜져 있어야 한다.
#   ./rec.sh start [이름]   지금부터 저장 시작 -> pi/live/runs/<시각>_<이름>/
#   ./rec.sh stop           저장 끝 (행 수 · 시간 표시)
#   ./rec.sh status         저장 중인지
# 저장되는 것: sensors.csv (센서 · 판정 0.1 초마다), twin.tws (카메라 깊이 · 컬러 + 센서, 디스플레이 재생용), summary.json
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
ask() {   # $1 = 센서 공급기에 보낼 명령. 답(JSON)을 받아 찍는다
    python3 - "$1" <<'PY'
import json, socket, sys
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(3)
s.sendto(sys.argv[1].encode(), ('127.0.0.1', 5022))
try:
    d = json.loads(s.recv(4096))
except socket.timeout:
    print('! 센서 공급기가 답이 없다. 먼저 ./run_live.sh 로 송신을 켠다'); sys.exit(1)
if d.get('recording'):
    print('저장 중: %s  (%d 행, %.1f 초)' % (d['dir'], d['rows'], d['seconds'] or 0))
elif d.get('last'):
    l = d['last']
    print('저장 안 하는 중. 마지막 저장: %s  (%d 행, %.1f 초, %s ~ %s)' % (l['dir'], l['rows'], l['seconds'], l['started'], l['ended']))
else:
    print('저장 안 하는 중')
PY
}
twin() { python3 -c "import socket,sys; socket.socket(socket.AF_INET,socket.SOCK_DGRAM).sendto(sys.argv[1].encode(),('127.0.0.1',5011))" "$1"; }

case "${1:-status}" in
    start)
        NAME="${2:-run}"
        DIR="$HERE/runs/$(date +%Y%m%d_%H%M%S)_${NAME// /_}"
        mkdir -p "$DIR"
        twin "rec_start $DIR"
        ask "start $DIR" || exit 1
        ;;
    stop)
        twin "rec_stop"
        ask "stop" || exit 1
        ;;
    status)
        ask "status"
        ;;
    *)
        echo "usage: $0 start [이름] | stop | status"
        exit 1
        ;;
esac
