#!/usr/bin/env bash
# 빌드 트리의 pyrealsense2를 쓰도록 환경을 잡고 파이썬 스크립트를 실행한다.
# (sudo make install 없이 쓰기 위한 래퍼)
#
#   ~/rs_py.sh ~/test_d435i.py --headless          # 숫자만
#   ~/rs_py.sh ~/test_d435i.py --max-width 800 --fullscreen --seconds 600
set -u

BUILD="${LIBREALSENSE_DIR:-$HOME/librealsense}/build/Release"

if ! ls "$BUILD"/pyrealsense2*.so >/dev/null 2>&1; then
    echo "pyrealsense2 .so가 없다: $BUILD" >&2
    exit 1
fi

export PYTHONPATH="$BUILD${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$BUILD${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# GUI 인자가 있으면 DSI 화면으로 보낸다 (--headless면 불필요하지만 둬도 무해)
export XDG_RUNTIME_DIR="/run/user/$(id -u)"
export DISPLAY=:0
export XAUTHORITY="$HOME/.Xauthority"
# X11(Xwayland)로 강제. unset만으로는 libwayland가 기본 소켓 "wayland-0"으로
# 폴백해 붙어버린다. 자세한 이유는 rs_viewer.sh 주석 참고.
export WAYLAND_DISPLAY=force-x11-no-such-socket

exec python3 "$@"
