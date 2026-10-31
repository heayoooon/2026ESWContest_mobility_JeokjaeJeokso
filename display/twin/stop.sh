#!/bin/bash
# SCSS 디지털 트윈 전부 끄기: 실행기(launch.sh), 키오스크 크로미움, twin_server.py.
set -u
PROFILE="$HOME/.cache/cargo-twin-kiosk"
CHROME="^/usr/lib/chromium/chromium .*--user-data-dir=$PROFILE( |$)"
PY='^python3( -u)? \S*(twin_server|stream_play)\.py'
LAUNCH='^(\S*/)?bash \S*(twin_web|display/twin|scss_twin)/launch\.sh'

pkill -TERM -f "$LAUNCH" 2>/dev/null     # 실행기가 스스로 정리한다
pkill -TERM -f "$CHROME" 2>/dev/null
pkill -TERM -f "$PY" 2>/dev/null
for i in $(seq 1 25); do
    pgrep -f "$CHROME" >/dev/null || pgrep -f "$PY" >/dev/null || pgrep -f "$LAUNCH" >/dev/null || break
    sleep 0.2
done
pkill -KILL -f "$CHROME" 2>/dev/null
pkill -KILL -f "$PY" 2>/dev/null
pkill -KILL -f "$LAUNCH" 2>/dev/null
rm -f "$(cd "$(dirname "$0")" && pwd)/run/"*.pid
echo "stopped"
