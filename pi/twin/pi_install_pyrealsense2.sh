#!/usr/bin/env bash
# 빌드된 librealsense(v2.58.4, RSUSB) 를 ~/d435i/.venv 에 pyrealsense2 패키지로 설치 (sudo 불필요).
# pip wheel 과 같은 구조: site-packages/pyrealsense2/{__init__.py, pyrealsense2*.so, librealsense2.so*}
# 빌드 시 RPATH=$ORIGIN 이라 같은 폴더의 librealsense2.so 를 찾는다.
set -euo pipefail
B=~/librealsense/build
PY=~/d435i/.venv/bin/python
SP=$($PY -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')
DST=$SP/pyrealsense2

rm -rf "$DST"; mkdir -p "$DST"
R=$B/Release                                    # 빌드 산출물 위치
cp -av "$R"/pyrealsense2*.so* "$R"/pyrsutils*.so* "$DST"/
cp -av "$R"/librealsense2.so* "$DST"/
cat > "$DST/__init__.py" <<'EOF'
# source-built librealsense v2.58.4 (RSUSB backend) for aarch64 / Python 3.13
from .pyrealsense2 import *
from .pyrealsense2 import __version__
EOF

# CLI 도구 (rs-enumerate-devices 등): lib 경로를 잡아주는 래퍼를 ~/.local/bin 에
mkdir -p ~/.local/bin
for t in $(find "$R" -maxdepth 1 -type f -name 'rs-*' -perm -u+x); do
  n=$(basename "$t")
  printf '#!/bin/sh\nLD_LIBRARY_PATH=%s${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH} exec %s "$@"\n' "$DST" "$t" > ~/.local/bin/$n
  chmod +x ~/.local/bin/$n; echo "tool: $n"
done
$PY -c 'import pyrealsense2 as rs; print("pyrealsense2", rs.__version__, "OK; devices:", len(rs.context().query_devices()))'
