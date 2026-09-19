#!/usr/bin/env bash
# realsense-viewer를 Pi의 DSI 화면에 띄운다.
# SSH로 들어와서 실행해도 로컬 모니터에 창이 뜬다.
#
#   ~/rs_viewer.sh          # 포그라운드
#   ~/rs_viewer.sh &        # 백그라운드
set -u

BUILD="${LIBREALSENSE_DIR:-$HOME/librealsense}/build/Release"
BIN="$BUILD/realsense-viewer"

if [ ! -x "$BIN" ]; then
    echo "realsense-viewer가 없다: $BIN" >&2
    exit 1
fi

export XDG_RUNTIME_DIR="/run/user/$(id -u)"
export DISPLAY=:0
export XAUTHORITY="$HOME/.Xauthority"
export LD_LIBRARY_PATH="$BUILD${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# GLFW를 X11(Xwayland) 백엔드로 강제한다.
#
# WAYLAND_DISPLAY을 그냥 unset하면 안 된다. libwayland의 wl_display_connect(NULL)이
# XDG_RUNTIME_DIR의 기본 소켓명 "wayland-0"으로 폴백해서 결국 Wayland에 붙어버린다.
# 실제로 그렇게 붙었더니 GLFW가 창 위치·커서 설정을 못 해서 초당 200줄씩 에러를
# 뱉었다. 존재하지 않는 소켓명을 넣어 Wayland 연결을 실패시키면 GLFW가 DISPLAY로
# 폴백한다.
export WAYLAND_DISPLAY=force-x11-no-such-socket

echo "DSI 화면(800x480)에 realsense-viewer를 띄운다 (X11 백엔드)..."
exec "$BIN" "$@"
