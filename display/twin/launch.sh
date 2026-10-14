#!/bin/bash
# SCSS 화물 디지털 트윈 키오스크 실행기 (디스플레이 파이. 바탕화면 아이콘이 이것을 실행한다).
# 센서 파이 실시간 입력 전용: 센서 파이가 TCP 5010 으로 보내는 프레임만 그린다 (녹화 재생 없음).
#   1) 이전에 떠 있던 것(서버, 키오스크 크로미움)을 끈다
#   2) server/twin_server.py (http://localhost:8090, TCP 5010 수신) 를 띄우고 응답할 때까지 기다린다.
#      끝내 응답이 없으면 키오스크 대신 안내 창(zenity)
#   3) 크로미움 키오스크로 연다. 크로미움이 닫히면(홈 버튼 = POST /api/exit) 서버도 끈다.
#      쓰는 중에 서버가 죽으면 키오스크를 닫고 안내 창을 띄운다 (터치만으로는 오류 페이지를 닫을 수 없으므로)
# 두 번 눌려도 하나만 뜬다 (잠금 파일). 로그: display/twin/logs/
set -u
APP="$(cd "$(dirname "$0")" && pwd)"
LOGS="$APP/logs"
RUN="$APP/run"
PROFILE="$HOME/.cache/cargo-twin-kiosk"
URL="http://localhost:8090/"
mkdir -p "$LOGS" "$RUN" "$PROFILE"

# 화면 · 소리 환경 (바탕화면에서 실행하든 ssh 로 실행하든 같게)
export XDG_RUNTIME_DIR=/run/user/1000
export WAYLAND_DISPLAY=wayland-0
export DBUS_SESSION_BUS_ADDRESS="unix:path=$XDG_RUNTIME_DIR/bus"
export XDG_SESSION_TYPE=wayland
export PYGAME_HIDE_SUPPORT_PROMPT=1

# 로그는 너무 커지면 한 번 돌린다
for f in launch chromium server.out; do
    [ -f "$LOGS/$f.log" ] && [ "$(stat -c %s "$LOGS/$f.log")" -gt 2000000 ] && mv -f "$LOGS/$f.log" "$LOGS/$f.log.1"
done
exec >>"$LOGS/launch.log" 2>&1
echo "=== $(date '+%F %T') launch.sh (pid $$)"

# 한 번에 하나만: 이미 실행기가 돌고 있으면 그냥 나간다 (아이콘을 두 번 눌렀을 때)
exec 9>"$RUN/launch.lock"
if ! flock -n 9; then
    echo "이미 실행 중 -> 종료"
    exit 0
fi

PY='^python3( -u)? \S*(twin_server|stream_play)\.py'
CHROME="^/usr/lib/chromium/chromium .*--user-data-dir=$PROFILE( |$)"

stop_all() {
    # 키오스크 크로미움 (본체만 TERM: 자식 프로세스는 따라서 닫힌다) · 서버 (예전 녹화 재생기가 떠 있으면 그것도)
    pkill -TERM -f "$CHROME" 2>/dev/null
    pkill -TERM -f "$PY" 2>/dev/null
}

wait_gone() {   # 최대 5 s 기다리고 남은 것은 KILL
    local i
    for i in $(seq 1 25); do
        pgrep -f "$CHROME" >/dev/null || pgrep -f "$PY" >/dev/null || return 0
        sleep 0.2
    done
    echo "남은 프로세스 강제 종료"
    pkill -KILL -f "$CHROME" 2>/dev/null
    pkill -KILL -f "$PY" 2>/dev/null
    sleep 0.3
}

# 살아 있나 (끝났는데 아직 거두지 않은 좀비는 죽은 것으로 본다)
alive() {
    local s
    s=$(ps -o stat= -p "$1" 2>/dev/null) || return 1
    case "$s" in Z*) return 1 ;; esac
    return 0
}

# 화면에 짧은 안내 창 (터치로 '확인', 30 s 뒤 저절로 닫힘). 키오스크 대신 이것만 띄울 때 쓴다
notify() {
    command -v zenity >/dev/null || return 0
    timeout 40 zenity --error --title='SCSS 디지털 트윈' --width=380 --timeout=30 --ok-label='확인' \
        --text="$1" </dev/null >/dev/null 2>&1 9>&- &
}

CHROME_PID=""
WATCH_PID=""
cleanup() {
    trap - EXIT INT TERM HUP
    echo "$(date '+%T') 정리: 크로미움 · 서버 끄기"
    [ -n "$WATCH_PID" ] && kill "$WATCH_PID" 2>/dev/null
    [ -n "$CHROME_PID" ] && kill -TERM "$CHROME_PID" 2>/dev/null
    stop_all
    wait_gone
    rm -f "$RUN/chromium.pid" "$RUN/server.pid"
    echo "$(date '+%T') 끝"
}

stop_all
wait_gone

trap cleanup EXIT
trap 'exit 0' INT TERM HUP

# 서버 (TCP 5010 을 열고 센서 파이 실시간 프레임을 기다린다). 9>&- : 잠금을 자식에게 물려주지 않는다
cd "$APP/server" || exit 1
setsid python3 -u "$APP/server/twin_server.py" </dev/null >>"$LOGS/server.out.log" 2>&1 9>&- &
echo $! >"$RUN/server.pid"
SERVER_PID=$(cat "$RUN/server.pid")
echo "server pid $SERVER_PID — 센서 파이 실시간 입력을 기다린다 (TCP 5010)"

ok=0
for i in $(seq 1 60); do
    if curl -sf -m 1 -o /dev/null "${URL}api/state"; then ok=1; break; fi
    alive "$SERVER_PID" || { echo "서버가 죽었다 (logs/server.out.log)"; break; }
    sleep 0.25
done
# 화면(페이지)도 이 서버가 준다. 서버 없이 키오스크를 열면 '연결할 수 없음' 페이지가 전체 화면으로 떠
# 터치만으로는 닫을 수 없으므로, 열지 않고 안내 창만 띄운다
if [ $ok != 1 ]; then
    echo "서버 응답 없음 -> 키오스크를 열지 않는다"
    notify "서버가 시작되지 않아 화면을 열지 않았습니다.\n\n잠시 뒤 바탕화면 아이콘을 다시 눌러 주세요.\n(기록: display/twin/logs/server.out.log)"
    exit 1
fi
echo "서버 응답 OK ($i)"

# 키오스크 프로필 설정: 지난번에 강제로 닫혔어도 '페이지 복원' 풍선 없이, 한국어 페이지에 '번역' 풍선이 뜨지 않게
# (--disable-features=Translate 만으로는 Chromium 149 에서 번역 풍선이 떴다)
python3 - "$PROFILE" <<'PY' || echo "Preferences 수정 실패"
import json, os, sys
d = os.path.join(sys.argv[1], 'Default')
os.makedirs(d, exist_ok=True)
p = os.path.join(d, 'Preferences')
try:
    with open(p, encoding='utf-8') as f:
        pr = json.load(f)
except Exception:
    pr = {}
pr.setdefault('profile', {}).update(exit_type='Normal', exited_cleanly=True)
pr['translate'] = dict(pr.get('translate') or {}, enabled=False)
pr['translate_blocked_languages'] = sorted(set(pr.get('translate_blocked_languages') or []) | {'ko'})
pr.setdefault('intl', {}).update(accept_languages='ko-KR,ko,en-US,en', selected_languages='ko-KR,ko,en-US,en')
with open(p, 'w', encoding='utf-8') as f:
    json.dump(pr, f)
PY

chromium \
    --kiosk "--user-data-dir=$PROFILE" \
    --noerrdialogs --disable-infobars --no-first-run --password-store=basic \
    --autoplay-policy=no-user-gesture-required \
    --check-for-update-interval=31536000 \
    --disable-features=Translate,TranslateUI \
    --overscroll-history-navigation=0 --disable-pinch \
    --ozone-platform=wayland --ignore-gpu-blocklist --enable-gpu-rasterization \
    --touch-events=enabled \
    --hide-crash-restore-bubble --disable-session-crashed-bubble --disable-renderer-accessibility \
    --disable-background-networking --disable-component-update \
    --remote-debugging-port=9222 --remote-debugging-address=127.0.0.1 \
    "$URL" </dev/null >>"$LOGS/chromium.log" 2>&1 9>&- &
CHROME_PID=$!
echo "$CHROME_PID" >"$RUN/chromium.pid"
echo "chromium pid $CHROME_PID"

# 지키미: 쓰는 중에 서버가 죽으면 페이지도 끊기고 홈 버튼도 듣지 않으므로 키오스크를 닫는다
rm -f "$RUN/server_died"
(
    while alive "$CHROME_PID"; do
        if ! alive "$SERVER_PID"; then
            echo "$(date '+%T') 서버가 멈췄다 -> 키오스크를 닫는다"
            : >"$RUN/server_died"
            kill -TERM "$CHROME_PID" 2>/dev/null
            break
        fi
        sleep 1
    done
) 9>&- &
WATCH_PID=$!

wait "$CHROME_PID"
code=$?
echo "$(date '+%T') 크로미움 종료 (code $code)"
if [ -f "$RUN/server_died" ]; then
    rm -f "$RUN/server_died"
    notify "서버가 멈춰서 화면을 닫았습니다.\n\n바탕화면 아이콘을 다시 눌러 주세요.\n(기록: display/twin/logs/server.out.log)"
fi
exit 0
