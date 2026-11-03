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
│   ├─ hud_v2.py, hud_v3.py (경보음 · 음성만 쓴다. 안의 pygame HUD 평면도는 안 쓰는 옛 코드로 센서 파이 이름 그대로 그린다), scene_v2.py, scene_v3.py
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
- **errors.log** (디버깅용, 1 MB x 5): 경고 · 오류 · traceback + 들어오는 상태가 **나빠지고 풀릴 때만** 한 줄
  (`twin_server.Health`: 센서 값 없음 · 늦음, IMU1 값 없음 · 3 초째 똑같음, 로드셀 · 장력 값 없음, 프레임 끊김, 5 초 연결 끊김,
  송신기 다시 켜짐). 센서 파이 쪽 원인은 센서 파이 `pi/live/logs/errors_*.log` 에 같은 시각으로 남는다.

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
- 로드셀: `NW_g NE_g SW_g SE_g total_g`, 장력: `c1_N c2_N` (c1 = 앞(N) 벨트 = 카메라 쪽, c2 = 뒤(S) 벨트 = 먼 쪽)
- IMU: `imu1_ax_G imu1_ay_G imu1_az_G imu1_gz_dps imu1_roll_deg imu1_pitch_deg`
- 판정 (`pi/analysis`, 0 정상 · 1 주의 · 2 경고 · 3 위험): `risk lv_load lv_cog lv_move lv_c1 lv_c2 risk_text`,
  `load_kg measured_kg cog_x_mm cog_y_mm cog_pred_x_mm cog_pred_y_mm c1_min_N c2_min_N`
- 벨트 누름 (`pi/analysis/belt_fit.py`): `c1_k c1_px_mm c1_py_mm c2_k c2_px_mm c2_py_mm`. 벨트 칸의 장력 아래에 `K x y` 로 보인다 (자리를 아직 모르면 `—`). `twin_server.py` 가 `sensors.belts[].k · x_mm · y_mm` 로 넘긴다
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
- 화물 모양 (`stack_layers.ObjectMemory`): 물건마다 번호 · 크기를 고정하고 위치와 윗면 높이는 프레임마다 높이맵에서 찾는다. 다른 물건에 덮인 상자는 안 움직인다.
  기억에 없는 평평한 네모는 상자. 손을 뗀 상태로 무게가 1.2 ~ 2.3 kg 늘고 받침 위에 병 크기 자국이 있으면 물통 (장력이 같이 변한 무게는 벨트라 안 친다, 눕힌 2 L, 지름 11 cm x 길이 31 cm 고정, 방향은 세로 · 가로). 물건이 생기고 옮겨지고 없어질 때마다 `logs/errors.log` 에 한 줄
- 소리 (USB 스피커): 단계 소리 + 이유 음성. 알림이 새로 뜨거나 단계가 오를 때 한 번
  - 위험 = 빠른 '삐삐삐' x4 + '위험 위험', 경고 = '삐빅' x3 + '경고 경고', 주의 = '띵-동' + '주의 주의'
  - 이어서 이유 (`twin_server.reason_clip`): 과적입니다 · 화물이 앞쪽/뒤쪽/왼쪽/오른쪽으로 쏠렸습니다 · 화물이 이동했습니다 ·
    앞/뒷 벨트에 장력이 부족합니다 (부족 · 풀림) · 앞/뒷 벨트에 장력이 과조임 되었습니다 (과조임 · 상한 넘음)
  - 여러 개가 같이 뜨면 높은 단계부터 차례로, 더 높은 단계가 뜨면 재생 중인 낮은 단계를 끊는다. 이미 풀린 알림은 말하지 않는다
  - 적재 모드에서는 소리를 안 낸다. 화면도 주행 모드와 같지만 화물 이동 칸만 등급 없이 값만 보인다 (기준 무게중심에서 떨어진 거리. 올리고 내리는 중이라 잠깐 커졌다가 약 4 초 뒤 새 자리를 기준으로 0 쪽으로 돌아온다, 화물이 없으면 0 mm '화물 없음'). 주행으로 바꾸는 순간 이미 떠 있는 경고 · 주의가 울린다
  - 카메라 격자는 y .75 m 까지 본다 (판 끝 실측 y .63, x -.245 ~ +.215). 그 너머 판 뒤 바닥에 선 물건은 화물로 안 친다
- 앞뒤 기준: **카메라 가까운 쪽이 트럭 앞** (앞 벨트), 먼 쪽이 뒤 (뒤 벨트). 센서 파이 이름과 같다. 카메라를 보고 선 셈이라 화물을 그릴 때 판 X = -(카메라 x - cx), 판 Z(뒤 +) = 카메라 y - cy
- 적재판은 네 구역을 따로 칠한다: 모서리 무게 몫 (과적일 때) · 무게중심이 있는 구역 · 화물이 밀린 구역 중 가장 나쁜 단계. 벨트는 벨트 줄 · 이름표 · 아래 칸이 알린다
- 보기 전용 주소 `http://<디스플레이 IP>:8091`: 같은 화면을 다른 사람이 본다 (멈춤 · 음량 · 끝내기 · 모드는 막힘, 화면을 한 번 누르면 같은 음성이 그 브라우저에서도 난다)

## 녹화 재생

```bash
./replay.sh                 # runs/ 의 녹화를 index.json 순서로 계속 (키오스크도 띄운다)
./replay.sh 급커브 물건빼기   # 이름에 그 글자가 들어간 녹화만
```

`runs/<녹화>/twin.tws` + `sensors.csv` 와 `runs/index.json` 이 필요하다 (용량이 커서 레포에는 없다).
서버가 받는 연결은 하나라서 센서 파이 실시간 송신이 붙어 있으면 재생이 들어가지 않는다. 같이 쓰려면 재생 전용 서버를 따로 띄운다:
`python3 server/twin_server.py --http 8092 --view 8093 --tcp 5012` + `python3 server/replay_runs.py --port 5012`

카메라와 적재판의 위치 관계는 `run/reg.json` 에 고정돼 있다 (지금 값 = 합판 실측 가운데 cx -.015, cy .35). 카메라를 옮기면 이 파일을 지운다:
서버가 "가감속 없을 때 카메라로 본 화물 중심 = 로드셀 무게중심" 이 되게 다시 배워 저장한다.
합판 · 로드셀 자리는 이것과 따로 박혀 있어서 다시 재서 같이 고친다: `server/stack_layers.py` 의 `CELL_CX CELL_CY PLATE_*`, `web/.../twin-scene.ts` 의 `PLATE_CAM IMU_CAM`, `isaac_sim/scripts` 의 판 끝 `.63`.

## 화면 고치기 (PC)

```bash
cd display/twin/web
npm ci
npm run dev                     # http://localhost:5173  (?mock=1 이면 가짜 데이터, 개발 서버에서만)
npx vite build --outDir ../www --emptyOutDir   # 고친 뒤 www 를 다시 만든다
```
