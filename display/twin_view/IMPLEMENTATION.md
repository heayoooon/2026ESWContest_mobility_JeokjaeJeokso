# 실측 표면 기반 디지털 트윈

2026-10-03. 두 Raspberry Pi에서 녹화 RGB-D와 센서 CSV를 동기화해 재생하는 데모다. 커밋하지 않았다.

## 달라진 점

- 기존 높이맵은 상자의 수직 앞면을 잃고, 주변 바닥과 화물을 긴 삼각형으로 연결했다. 기본 표현을 **카메라 깊이 격자의 실제 3D 표면**으로 바꿨다. 상자 앞면과 인쇄 글자가 유지된다.
- 실제 3D 변 길이가 25 mm를 넘는 삼각형은 제외한다. 수직 면 자체는 금지하지 않는다. 빈 영역이나 뒤쪽 면을 임의로 메우지 않는다.
- 카메라 설치 보정은 이전처럼 정속주행에서 고정한다. 원본 RGB/깊이를 확인한 관측 범위 `x=[-0.24,0.16], y=[0.035,0.45] m`만 남긴다. 이것은 자동 검출한 적재함 평면이나 실측 카트 치수가 아니다. `--roi X0 X1 Y0 Y1`로 변경할 수 있다.
- 보조 높이맵은 1 cm, 셀 최고점 아래 15 mm 이내 점의 색만 사용한다. 통계는 이 높이맵에서 산출되며, 밀폐된 물체의 실측 부피/무게중심으로 해석하면 안 된다.
- 작은 깊이 떨림(8 mm 미만)만 현재 65%/이전 35%로 완화한다. 큰 변화와 깊이 0은 즉시 반영하고, 시나리오 변경/반복에서 초기화한다. 가려진 형상을 몇 초간 유지하지 않는다.
- 고정 감마/밝기 LUT로 어두운 색을 보정한다. 색상이 카메라 시야 밖인 유효 깊이 표면은 회색이다.
- 기본 시점 고정, 4개 시점 버튼, 드래그 회전, 컬러 표면/포인트/높이색 전환. 우측 하중·무게중심·장력·IMU HUD는 유지한다.
- 관측 ROI와 바닥 기준 격자는 참고용이다. 실측 카트 모델과 로드셀-카메라 좌표 정합이 없으므로 실제 적재함 모델이나 3D 로드셀 위치를 지어내지 않았다.

## 전송

기본은 TCP 5010의 영구 연결이다. 4바이트 big-endian 프레임 길이 다음에 기존 JSON 길이/헤더/바이너리 구조를 보낸다. `session`으로 송신기 재시작을 구분한다. 네트워크 장애 시 재접속을 시도한다.

v2 본문은 순서대로 높이맵 zlib, 셀 RGB zlib, 실제 깊이 delta-zlib, 정렬 RGB JPEG(quality 88, 4:4:4)다. 깊이는 mm 정수의 가로 차분을 무손실 압축한다. `surface.intr`, `R`, `lens`, `depth_codec`, `color_codec`로 복원한다. RGB는 5-bit 양자화 후 JPEG이므로 색상은 손실 압축이다. 기하는 JPEG 영향을 받지 않는다.

센서 venv에 Pillow가 없으면 이미 설치된 Debian `/usr/lib/python3/dist-packages`의 Pillow를 사용한다. apt/pip 설치나 시스템 설정 변경은 하지 않았다.

`--transport udp` / 뷰어 `--udp`도 지원한다. UDP는 1200바이트 데이터 조각에 8개당 XOR 복구 조각 1개를 더한다. Tailscale에서 그룹 내 다중 조각 누락이 남아 최종 기본값은 TCP다. UDP 손실과 TCP 지연을 숨기지 않는다. 인터넷/Tailscale 장애까지 무손실·고정 지연을 보장하는 것은 아니다.

## 실행

디스플레이 파이:

```sh
cd ~/twin_view
XDG_RUNTIME_DIR=/run/user/1000 WAYLAND_DISPLAY=wayland-0 SDL_VIDEODRIVER=wayland \
  python3 -u twin_view.py 5010 --capture-dir /tmp/twin_screens
```

센서 파이:

```sh
cd /home/abcd/2026ESWContest_mobility_JeokjaeJeokso/pi
~/d435i/.venv/bin/python -u twin/replay_twin.py 100.90.251.42:5010 \
  --hz 10 --capture-dir /tmp/twin_capture
```

뷰어를 먼저 실행한다. 실행 로그는 `~/twin_view/view.log`, `/tmp/replay.log`다. 각 녹화의 전환에는 파일 열기/선행 프레임 디코드로 실제 데이터 공백이 생기며, 그 시간까지 10 Hz로 가짜 프레임을 만들지 않는다. `RUN ... active ... Hz`는 시나리오 재생 구간의 수신률이다. HUD는 최근 2초의 수신률이므로 전환에서 낮아질 수 있다.

## 파일 및 검증 자료

최종 저장 로그 기준: 누적 수신 **1,687 프레임, 손실 0**, 렌더 **27.5–30.3 fps**(5초 구간 측정). 송신기 재시작을 포함한 로그이므로 중간에 17프레임만 재생한 drop 항목이 한 번 있다. 완주한 각 시나리오의 재생 중 수신은 약 10 Hz다. 파일 전환 공백은 별도다.

| 시나리오 | 송신 프레임 | 처리 중앙 / p95 | 프레임 평균 | 전송량 | 재생 지연 누적 |
|---|---:|---:|---:|---:|---:|
| 정속주행 | 149 | 30.7 / 37.1 ms | 36.6 KB | 2.99 Mbps | 0 |
| 급출발·급정거 | 63 | 31.1 / 33.3 ms | 39.4 KB | 3.23 Mbps | 0 |
| 우회전 | 109 | 31.3 / 32.1 ms | 38.7 KB | 3.17 Mbps | 0 |
| 화물 떨어짐 | 87 | 31.3 / 37.0 ms | 38.7 KB | 3.17 Mbps | 0 |

위 처리 시간은 기존 로거처럼 프레임 필터링부터 전송까지이며, rosbag 프레임셋 디코드는 앞에서 수행한다. 최초 TCP 연결은 재생 시계 시작 전에 준비한다. Mbps는 응용 프레임 기준으로 TCP/IP/Tailscale 헤더는 제외한다. 12개 회귀 테스트는 PC와 센서 Pi에서 모두 통과했다. 상세 로그와 추출 수치는 `final_replay.log`, `final_view.log`, `performance.json`에 보관했다.

- `replay_twin.py`: ROI/표면 전송, 동기화 재생, 원본/패킷 캡처.
- `twin_view.py`: 표면 메시, 수신 스레드, HUD, 터치 시점/모드, 화면 캡처.
- `surface_core.py`: 순수 numpy 표면 인덱스/경계 제거/깊이 평활.
- `transport_core.py`: 길이 기반 TCP, 선택적 UDP 조립/복구.
- `test_surface_core.py`, `test_transport_core.py`: 수직 면 보존, 커튼 제거, 미관측 비움, 즉각적인 변화 반영, 조각 손실/재정렬/만료, TCP 프레이밍 등 12개 회귀 테스트.
- `comparison_montage.png`: 네 시나리오 원본 RGB / 이전 / 개선 화면.
- `comparison_checks.json`: 원본·이전·개선 프레임의 타임스탬프 일치 검증.
- `twin_capture/`: 실제 RGB, 전송 패킷. `twin_screens/`: 장비 화면/프레임 메타데이터.
- `twin_baseline/`: 기존 송신기와 기존 뷰어로 재현한 동일 시점 비교 자료. 당시 라이브 성능 로그가 아니라 오프라인 재렌더이므로 이미지 하단의 0 Hz/0 fps는 성능 측정값이 아니다.
- `capture_baseline.py`, `render_baseline.py`, `make_comparison.py`: 비교 자료 재생성 도구.
- `before.png`, `after_first.png`: 초기 실기 화면.

기존 코드는 센서 `/tmp/replay_twin_before.py`, 디스플레이 `~/twin_view/twin_view_before.py`로 백업했다. `/tmp` 자료는 재부팅 시 없어질 수 있으므로 로컬 비교 자료를 보존한다. 실제 카메라용 `pi_twin_node.py`와 다른 사용자의 수정 파일은 변경하지 않았다.
