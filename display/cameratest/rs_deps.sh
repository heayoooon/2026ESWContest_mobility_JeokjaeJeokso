#!/usr/bin/env bash
# librealsense 빌드 의존성 + RealSense udev 권한 설정 (sudo 필요, 1회성)
# 실행: sudo bash ~/rs_deps.sh
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "root로 실행해야 한다:  sudo bash ~/rs_deps.sh" >&2
    exit 1
fi

# sudo로 실행되므로 $HOME은 /root다. 실제 사용자의 홈을 찾는다.
TARGET_USER="${SUDO_USER:-$(logname 2>/dev/null || echo root)}"
TARGET_HOME=$(getent passwd "$TARGET_USER" | cut -d: -f6)
REPO="${LIBREALSENSE_DIR:-$TARGET_HOME/librealsense}"
RULES="$REPO/config/99-realsense-libusb.rules"

echo "=== [1/3] apt 패키지 목록 갱신 ==="
apt-get update

echo
echo "=== [2/3] 빌드 의존성 설치 ==="
apt-get install -y --no-install-recommends \
    git cmake build-essential pkg-config \
    libssl-dev libusb-1.0-0-dev libudev-dev \
    libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
    python3-dev v4l-utils

echo
echo "=== [3/3] udev 권한 규칙 설치 ==="
# 공식 setup_udev_rules.sh는 대화형 프롬프트가 있어 rules 파일만 직접 넣는다.
if [ ! -f "$RULES" ]; then
    echo "rules 파일이 없다: $RULES" >&2
    exit 1
fi
cp "$RULES" /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger
echo "  -> /etc/udev/rules.d/99-realsense-libusb.rules 설치 완료"

echo
echo "============================================"
echo " 완료. 카메라 USB를 한 번 뽑았다 다시 꽂으면"
echo " 새 권한 규칙이 적용된다."
echo "============================================"
