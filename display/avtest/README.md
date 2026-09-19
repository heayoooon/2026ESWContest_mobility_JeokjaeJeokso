# 모니터 / 스피커 테스트 (display pi)
스피커 (USB, Jieli UACDemoV1.0, PipeWire 기본 싱크):
  pw-play ~/avtest/speakertest.wav        # 왼쪽 삐, 오른쪽 삐, 양쪽 삐, 상승음 4개 (3.8초)
  wpctl get-volume @DEFAULT_AUDIO_SINK@   # 볼륨 확인 / set-volume 0.7 로 변경
모니터 (7인치 DSI 800x480, Wayland labwc):
  XDG_RUNTIME_DIR=/run/user/1000 WAYLAND_DISPLAY=wayland-0 SDL_VIDEODRIVER=wayland python3 ~/avtest/disptest.py
  grim ~/avtest/shot.png                  # 화면 캡처
