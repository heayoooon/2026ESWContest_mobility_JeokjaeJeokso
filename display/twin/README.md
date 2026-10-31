# SCSS 디지털 트윈 (디스플레이 파이)

센서 파이가 **실시간으로** 보내는 카메라(D435i 깊이 · 컬러) + 센서 값(로드셀 4 · 장력 2 · IMU) + 위험 판정을
받아 7인치 터치 화면에 2.5D 디지털 트윈으로 그리는 키오스크 앱. **녹화 재생은 없다** — 센서 파이가 보내지 않으면
"센서 신호를 기다리는 중" 화면에서 기다린다.

```
display/twin/
├─ launch.sh          키오스크 실행 (서버 + 크로미움 전체 화면). 바탕화면 아이콘이 이것을 실행
├─ stop.sh            전부 끄기
├─ install.sh         바탕화면 · 메뉴 아이콘 만들기 (한 번만)
├─ icon.png
├─ server/            Python 백엔드 (http://localhost:8090, TCP 5010 수신)
│   ├─ twin_server.py   HTTP · SSE · 음량 · 경보음, 위험 판정을 화면 형식으로
│   ├─ stack_layers.py  쌓인 상자를 층별로 나누기 · 옆 짐 · 판 높이 찾기
│   ├─ twin_view.py, stable_twin.py, cargo_model.py, surface_core.py, transport_core.py
│   │                   수신 · 깊이 안정화 · 2.5D 상자 맞추기
│   ├─ hud_v2.py, hud_v3.py (경보음 · 음성), scene_v2.py, scene_v3.py
│   └─ sounds/          warning.mp3 (경고 · 위험), caution.mp3 (주의)
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
- 모드: `mode` (`load` 적재 · `drive` 주행). 적재 모드면 경고음 · 주의음을 내지 않는다. 없으면(옛 송신기) 주행처럼 소리를 낸다

## 보내는 신호 (디스플레이 파이 → 센서 파이, UDP)

| 포트 | 내용 |
|---|---|
| 5011 | `pause` / `resume` (화면 멈춤, live_twin.py) |
| 5023 | `mode load` / `mode drive` (적재 · 주행 단추, sensor_feed.py). `POST /api/mode {mode}` 가 보낸다 |

## 화면

- 위 띠: 종합 위험도(팀 판정 안전 여유 곱) + 가장 중요한 이유, 화물 이동은 따로 알림, 색 뜻(정상 · 주의 · 경고 · 위험), 받는 속도(Hz)
- 가운데 3D: 적재판(네 구역 색) · 쌓인 상자(층별) · 벨트 · 무게중심, 문제 있는 곳에 이름표. 위험 단계 색이 화물 · 판에 비친다
- 왼쪽 막대: 홈, 보기 모드(기본 · 투시 · 분해 · 센서), 무게중심 지도(판을 위에서, 팀 판정 구역), 기울기
- 오른쪽 막대: 시점, 화면 멈춤, 적재 · 주행 (지금 모드를 글자로, 누르면 바뀐다), 음량
- 아래 다섯 칸: 적재 · 무게중심 · 화물 이동 · 앞 벨트 · 뒤 벨트 (누르면 자세히)
- 경고 · 위험은 경고음, 주의는 주의음 (USB 스피커). 적재 모드에서는 소리만 안 내고 화면은 주행 모드와 같다.
  주행으로 바꾸는 순간 이미 떠 있는 경고 · 주의가 울린다

카메라와 적재판의 위치 관계는 잰 적이 없어서, 서버가 "가감속 없을 때 카메라로 본 화물 중심 = 로드셀 무게중심" 이 되게
스스로 맞춘다 (`logs/../run/reg.json` 에 저장). 카메라를 옮기면 `run/reg.json` 을 지운다.

## 화면 고치기 (PC)

```bash
cd display/twin/web
npm ci
npm run dev                     # http://localhost:5173  (?mock=1 이면 가짜 데이터, 개발 서버에서만)
npx vite build --outDir ../www --emptyOutDir   # 고친 뒤 www 를 다시 만든다
```
