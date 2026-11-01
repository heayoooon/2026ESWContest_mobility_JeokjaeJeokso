# SCSS 디지털 트윈 (디스플레이 파이)

센서 파이가 **실시간으로** 보내는 카메라(D435i 깊이 · 컬러) + 센서 값(로드셀 4 · 장력 2 · IMU) + 위험 판정을
받아 7인치 터치 화면에 2.5D 디지털 트윈으로 그리는 키오스크 앱. 센서 파이가 보내지 않으면
"센서 신호를 기다리는 중" 화면에서 기다린다. 녹화 재생은 `replay.sh` 로 따로 (아래).

```
display/twin/
├─ launch.sh          키오스크 실행 (서버 + 크로미움 전체 화면). 바탕화면 아이콘이 이것을 실행
├─ stop.sh            전부 끄기
├─ install.sh         바탕화면 · 메뉴 아이콘 만들기 (한 번만)
├─ replay.sh          녹화 재생 (runs/ 의 녹화를 키오스크로 계속 돌린다)
├─ icon.png
├─ server/            Python 백엔드 (http://localhost:8090, 보기 전용 :8091, TCP 5010 수신)
│   ├─ twin_server.py   HTTP · SSE · 음량 · 경보음 · 음성, 위험 판정을 화면 형식으로
│   ├─ replay_runs.py   runs/<녹화>/twin.tws + sensors.csv 를 녹화 속도대로 127.0.0.1:5010 에 보낸다
│   ├─ stack_layers.py  쌓인 상자를 층별로 나누기 · 옆 짐 · 판 높이 찾기
│   ├─ twin_view.py, stable_twin.py, cargo_model.py, surface_core.py, transport_core.py
│   │                   수신 · 깊이 안정화 · 2.5D 상자 맞추기
│   ├─ hud_v2.py, hud_v3.py (경보음 · 음성), scene_v2.py, scene_v3.py
│   └─ sounds/          caution.mp3 '주의 주의' · warning.mp3 '경고 경고' · danger.mp3 '위험 위험',
│                       reason/*.mp3 이유 음성 (ElevenLabs, Julia - Confident and Friendly, Eleven v4)
├─ www/               빌드된 화면 (서버가 그대로 내보낸다. 파이에는 node 가 필요 없다)
└─ web/               화면 소스 (React 19 + three.js + Vite)
```

## 실행 (디스플레이 파이)

```bash
cd display/twin
./install.sh        # 처음 한 번: 바탕화면에 'Smart Cargo Stability System' 아이콘
./launch.sh         # 또는 아이콘을 누른다
./stop.sh           # 끄기
```

필요: Raspberry Pi OS (Wayland, labwc), chromium, python3 + numpy · pygame · Pillow · PyOpenGL (`apt install python3-numpy python3-pygame python3-pil python3-opengl`), PipeWire (`wpctl`, 음량).
로그: `display/twin/logs/` (server.log, server.out.log, launch.log, chromium.log)

센서 파이와는 LAN 선으로 연결한다. 센서 파이 쪽 송신기가 `<디스플레이 파이 IP>:5010` 으로 TCP 연결해서 보낸다.

## 받는 형식 (센서 파이 → 디스플레이 파이, TCP 5010)

프레임마다 `[4바이트 길이 (big-endian)][본문]`, 약 10 Hz.
본문 = `[4바이트 헤더 길이][헤더 JSON][zlib 높이맵 int16][zlib 높이맵 색 uint8 RGB][zlib 깊이 (가로 차분 int16)][컬러 JPEG]`

헤더 JSON 에서 쓰는 것:

| 키 | 뜻 |
|---|---|
| `session`, `seq`, `send_t` | 보내는 쪽 세션 id, 일련번호, 보낸 시각 (unix s) |
| `run`, `t_run` | 실행 이름 · 시작부터 초 (같은 세션 안에서 늘어나야 한다) |
| `grid` | 높이맵 `{nx, ny, cell, x0, y0, unit:"mm", nodata:-32768}` (m) |
| `len` | `[높이맵 바이트, 색 바이트]` |
| `calib.ref_h` | 바닥(카메라 아래 기준면) 높이 (m, 카메라 좌표) |
| `cam_accel` | 카메라 IMU 가속도 `[x, y, z]` (흔들림 판정) |
| `surface` | `{intr:{width,height,fx,fy,ppx,ppy}, R:3x3 월드 회전, lens:[깊이 바이트, JPEG 바이트], depth_codec:"delta-zlib", color_codec:"jpeg"}` |
| `sensors` | `pi/pipeline.py` CSV 한 행과 같은 이름 (아래) |
| `sensor_age` | 센서 값이 카메라 프레임보다 얼마나 오래됐나 (s) |

`sensors` (없는 값은 `null`):
- 로드셀: `NW_g NE_g SW_g SE_g total_g`, 장력: `c1_N c2_N` (c1 = 앞(N) 벨트, c2 = 뒤(S) 벨트)
- IMU: `imu1_ax_G imu1_ay_G imu1_az_G imu1_gz_dps imu1_roll_deg imu1_pitch_deg`
- 판정 (`pi/analysis`, 0 정상 · 1 주의 · 2 경고 · 3 위험): `risk lv_load lv_cog lv_move lv_c1 lv_c2 risk_text`,
  `load_kg measured_kg cog_x_mm cog_y_mm cog_pred_x_mm cog_pred_y_mm c1_min_N c2_min_N`
- 있으면 더 정확: `cog_static_x_mm cog_static_y_mm` (하중 이동을 뺀 무게중심 — 카메라 · 판 위치 맞춤에 씀), `rated_kg load_pct risk_overall`
- 이유 문구: `check_text` (`{load, cog, move, c1, c2: Check.text}`). 없으면 `risk_text` 를 나눠 쓴다. 이유 음성을 고르는 데 쓴다
- 모드: `mode` (`load` 적재 · `drive` 주행). 적재 모드면 경고음 · 주의음을 내지 않는다. 없으면(옛 송신기 · 녹화 재생)
  디스플레이 서버가 모드를 들고(처음은 주행) 적재 · 주행 단추로 소리만 끄고 켠다 (`TwinState.mode_by = 'display'`)

헤더의 `src` 에 `replay` 가 있으면 녹화 재생(화면에 '녹화 재생'), `script.boxes` 가 있으면 깊이 추정 대신 그 상자(영상용 장면, `kind: 'bottle'` = 눕힌 페트병)를 그대로 그린다.

## 보내는 신호 (디스플레이 파이 → 센서 파이, UDP)

| 포트 | 내용 |
|---|---|
| 5011 | `pause` / `resume` (화면 멈춤, live_twin.py) |
| 5023 | `mode load` / `mode drive` (적재 · 주행 단추), `rebase` (기준 재설정 단추, 실시간일 때만). 둘 다 sensor_feed.py 가 받는다. `POST /api/mode {mode}` · `POST /api/control {cmd: rebase}` 가 보낸다 |

## 화면

- 위 띠: 종합 위험도(팀 판정 안전 여유 곱) + 가장 중요한 이유, 화물 이동은 따로 알림, 색 뜻(정상 · 주의 · 경고 · 위험), 받는 속도(Hz)
- 가운데 3D: 적재판(네 구역 색) · 쌓인 상자(층별) · 벨트 · 무게중심, 문제 있는 곳에 이름표. 위험 단계 색이 화물 · 판에 비친다
- 왼쪽 막대: 홈, 보기 모드(기본 · 투시 · 분해 · 센서), 무게중심 지도(판을 위에서, 팀 판정 구역), 기울기
- 오른쪽 막대: 시점, 화면 멈춤, 적재 · 주행 (지금 모드를 글자로, 누르면 바뀐다), 기준 재설정, 음량
- 아래 다섯 칸: 적재 · 무게중심 · 화물 이동 · 앞 벨트 · 뒤 벨트 (누르면 자세히)
- 소리 (USB 스피커): 단계 소리 + 이유 음성. 알림이 새로 뜨거나 단계가 오를 때 한 번
  - 위험 = 빠른 '삐삐삐' x4 + '위험 위험', 경고 = '삐빅' x3 + '경고 경고', 주의 = '띵-동' + '주의 주의'
  - 이어서 이유 (`twin_server.reason_clip`): 과적입니다 · 화물이 앞쪽/뒤쪽/왼쪽/오른쪽으로 쏠렸습니다 · 화물이 이동했습니다 ·
    앞/뒷 벨트에 장력이 부족합니다 (부족 · 풀림) · 앞/뒷 벨트에 장력이 과조임 되었습니다 (과조임 · 상한 넘음)
  - 여러 개가 같이 뜨면 높은 단계부터 차례로, 더 높은 단계가 뜨면 재생 중인 낮은 단계를 끊는다. 이미 풀린 알림은 말하지 않는다
  - 적재 모드에서는 소리만 안 내고 화면은 주행 모드와 같다. 주행으로 바꾸는 순간 이미 떠 있는 경고 · 주의가 울린다
- 보기 전용 주소 `http://<디스플레이 IP>:8091`: 같은 화면을 다른 사람이 본다 (멈춤 · 음량 · 끝내기 · 모드는 막힘, 화면을 한 번 누르면 같은 음성이 그 브라우저에서도 난다)

## 녹화 재생

```bash
./replay.sh                 # runs/ 의 녹화를 index.json 순서로 계속 (키오스크도 띄운다)
./replay.sh 급커브 물건빼기   # 이름에 그 글자가 들어간 녹화만
```

`runs/<녹화>/twin.tws` + `sensors.csv` 와 `runs/index.json` 이 필요하다 (용량이 커서 레포에는 없다).
서버가 받는 연결은 하나라서 센서 파이 실시간 송신이 붙어 있으면 재생이 들어가지 않는다. 같이 쓰려면 재생 전용 서버를 따로 띄운다:
`python3 server/twin_server.py --http 8092 --view 8093 --tcp 5012` + `python3 server/replay_runs.py --port 5012`

카메라와 적재판의 위치 관계는 잰 적이 없어서, 서버가 "가감속 없을 때 카메라로 본 화물 중심 = 로드셀 무게중심" 이 되게
스스로 맞춘다 (`logs/../run/reg.json` 에 저장). 카메라를 옮기면 `run/reg.json` 을 지운다.

## 화면 고치기 (PC)

```bash
cd display/twin/web
npm ci
npm run dev                     # http://localhost:5173  (?mock=1 이면 가짜 데이터, 개발 서버에서만)
npx vite build --outDir ../www --emptyOutDir   # 고친 뒤 www 를 다시 만든다
```
