#!/bin/bash
# 녹화 재생 (디스플레이 파이): runs/ 의 녹화를 디지털 트윈으로 계속 돌려 보여 준다.
#   ./replay.sh              전부 차례로 (끝나면 처음부터)
#   ./replay.sh 좌회전 충돌   이름에 그 글자가 들어간 녹화만
# 키오스크(launch.sh)를 같이 띄운다. 홈 버튼으로 키오스크를 닫으면 재생기도 끝난다.
# 끄기만: pkill -f replay_runs.py
# 센서 파이 실시간 송신(run_live.sh)과 같이 쓰지 않는다 (서버가 받는 연결은 하나).
set -u
APP="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$APP/logs"
pkill -f 'replay_runs\.py' 2>/dev/null
nohup python3 -u "$APP/server/replay_runs.py" "$@" >>"$APP/logs/replay.log" 2>&1 </dev/null &
echo "replay pid $! (logs/replay.log). 키오스크를 닫으면 재생기도 스스로 끝난다"
# 키오스크가 이미 떠 있으면 launch.sh 는 그냥 나가고, 재생기는 떠 있는 서버로 보낸다
"$APP/launch.sh"
