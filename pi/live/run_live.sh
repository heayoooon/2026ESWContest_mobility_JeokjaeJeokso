#!/bin/bash
# SCSS 실시간 송신 시작 (센서 파이): 센서 공급기(sensor_feed.py) + 카메라 송신기(live_twin.py).
# 디스플레이 파이에서는 display/twin/launch.sh (바탕화면 아이콘) 를 켜 둔다.
# usage: ./run_live.sh [디스플레이 주소, 기본 192.168.10.2] [미러 주소[:포트], 예: Isaac Sim 트윈 PC 100.109.1.106:5010]
#   카메라 쪽 파이썬은 pyrealsense2 가 있는 것: 기본 ~/d435i/.venv/bin/python (CAM_PY 로 바꾼다)
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
PI="$(dirname "$HERE")"
DEST="${1:-192.168.10.2}"
MIRROR="${2:-}"
CAM_PY="${CAM_PY:-$HOME/d435i/.venv/bin/python}"
mkdir -p "$HERE/logs"
"$HERE/stop_live.sh" >/dev/null 2>&1
# pipeline.py 와 HX711 을 같이 쓸 수 없다
pkill -INT -f '^python3( -u)? \S*pipeline\.py' 2>/dev/null && sleep 2
# 센서 배선 · 역할 (pipeline.py 와 같은 값)
source "$PI/acquisition/env.sh"
# IMU1 (MPU-6050, I2C 0x68): 다른 도구가 GPIO2/3 을 일반 GPIO 로 바꿔 두면 I2C 가 끊긴다 -> 켜기 전에 I2C 기능으로 되돌린다
pinctrl set 2 a3 pu 2>/dev/null; pinctrl set 3 a3 pu 2>/dev/null
cd "$HERE"
nohup python3 -u "$HERE/sensor_feed.py" >>"$HERE/logs/sensor_feed.log" 2>&1 </dev/null &
echo "sensor_feed pid $!"
nohup "$CAM_PY" -u "$HERE/live_twin.py" "$DEST" ${MIRROR:+--mirror "$MIRROR"} >>"$HERE/logs/live_twin.log" 2>&1 </dev/null &
echo "live_twin pid $! -> $DEST:5010${MIRROR:+ (미러 $MIRROR)}"
