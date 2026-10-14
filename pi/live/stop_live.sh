#!/bin/bash
# SCSS 실시간 송신 끄기 (센서 파이). 어떤 경로 · 인터프리터로 띄웠든 sensor_feed.py / live_twin.py 를 끈다
PAT='(sensor_feed|live_twin)\.py'
pkill -INT -f "$PAT" 2>/dev/null
for i in $(seq 1 30); do
    pgrep -f "$PAT" >/dev/null || break
    sleep 0.2
done
pkill -KILL -f "$PAT" 2>/dev/null
echo "stopped"
