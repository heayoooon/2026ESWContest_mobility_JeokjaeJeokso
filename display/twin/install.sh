#!/bin/bash
# 디스플레이 파이에 바탕화면 · 메뉴 아이콘을 만든다 (한 번만). 아이콘을 누르면 launch.sh 가 실행된다.
set -eu
APP="$(cd "$(dirname "$0")" && pwd)"
chmod +x "$APP/launch.sh" "$APP/stop.sh"
ENTRY="[Desktop Entry]
Type=Application
Version=1.0
Name=Smart Cargo Stability System
Name[ko]=Smart Cargo Stability System
GenericName=화물 적재 디지털 트윈
Comment=SCSS · 화물 적재 디지털 트윈 (센서 파이 실시간)
Exec=$APP/launch.sh
Icon=$APP/icon.png
Terminal=false
StartupNotify=false
Categories=Utility;
Keywords=SCSS;smart;stability;cargo;twin;digital;화물;트윈;"
mkdir -p "$HOME/.local/share/applications" "$HOME/Desktop"
for f in "$HOME/.local/share/applications/scss-twin.desktop" "$HOME/Desktop/scss-twin.desktop"; do
    printf '%s\n' "$ENTRY" >"$f"
    chmod +x "$f"
done
gio set "$HOME/Desktop/scss-twin.desktop" metadata::trusted true 2>/dev/null || true
echo "아이콘을 만들었다: 바탕화면 · 메뉴의 'Smart Cargo Stability System'"
