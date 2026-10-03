# 적재적소 (JeokjaeJeokso)

**2026 임베디드 소프트웨어 경진대회 · 모빌리티 부문**

운반 대차에 실린 화물의 **무게 분포**와 **차체 자세**를 실시간으로 재고, 지금 이
적재 상태가 위험한지 판정해서 3D로 보여주는 시스템이다.

총 하중이 정격 이내여도 무게가 한쪽으로 몰리면 문턱을 넘거나 코너를 돌 때 넘어진다.
"얼마나 실었나"가 아니라 **"어디에 실렸나"** 를 보는 것이 이 프로젝트의 초점이고,
그래서 이름이 적재적소다.

---

## 시스템 구성

```
  센서 (pi / display 노드)                엣지                      서버 / 시각화
  ─────────────────────────              ───────────               ───────────────
  하중  로드셀 + HX711  ──┐
  자세  MPU-6050 x2 (I2C) ─┼──▶  센서 퓨전 / 모션 보상  ──▶  FastAPI + WebSocket
  형상  RealSense D435i ──┘        무게중심 · 위험도 판정        2.5D 렌더링 · 디지털 트윈
```

역할 구분은 [이슈 템플릿](.github/ISSUE_TEMPLATE/bug_report.yml)의 파트 분류와 같다.

| 파트 | 하는 일 | 폴더 |
|---|---|---|
| hardware | 센서 · 배선 · 펌웨어 | `pi/`, `display/`, `hardware/` |
| edge-computing | 센서 퓨전 · 모션 보상 · 위험도 판정 | `edge/` |
| server-visualization | FastAPI · WebSocket · 2.5D 렌더링 | `server/`, `viz/` |

---

## 저장소 구조

| 경로 | 내용 |
|---|---|
| `pi/sensors/` | 센서 드라이버: HX711 다채널(하중 · 장력), MPU-6050(I2C 직결 · ESP32 UDP), PC용 가짜 센서 |
| `pi/pipeline.py` | 센서 실행 파일: 수집 → 판정 → 기록. 단계 사이는 `Frame`(센서값) · `Result`(판정 결과)로 주고받는다. 판정은 pi 에서 한다 |
| `pi/acquisition/` | 공통 시계(`CLOCK_MONOTONIC`) 기반 수집 · 정렬 · 기록 (`source.py` → `Frame`, `log_xlsx.py` 엑셀, `log_camera.py` 카메라 엑셀, `log_sync.py` raw CSV) |
| `pi/analysis/` | 판정: `analyzer.py`(`Frame` → `Result`), 총하중 · 무게중심 (`cog.py`) |
| `pi/calibration/` | 로드셀 영점 · 배율(`calib.json`), IMU 바이어스 · 장착 방향(`imu_cal.json`) |
| `pi/tools/` | HX711 배선 점검, IMU 스캔 · 실시간 각도, I2C 진단 |
| `display/avtest/` | 7인치 DSI 화면 · USB 스피커 동작 확인 |
| `display/cameratest/` | RealSense D435i 진단 (`test_d435i.py`, `ir_probe.py`) + Pi 5용 librealsense 소스 빌드 절차 |
| `viz/digital-twin/` | 영상 → 포토그래메트리 → Blender 정리 → Unity 씬 생성 파이프라인 |
| `hardware/` `edge/` `server/` | 아직 비어 있음 (`.gitkeep`) |

---

## 하드웨어

| 노드 | 보드 / OS | 달린 것 |
|---|---|---|
| `pi` | Raspberry Pi 5 Model B · Debian 13 (trixie) · Python 3.13 | 로드셀 + HX711, MPU-6050 2개 |
| `display` | Raspberry Pi 5 · Debian 13 · labwc(Wayland) | 공식 7" DSI 터치스크린 800x480, USB 스피커, RealSense D435i |

- **로드셀**: 4개를 HX711 1개에 합친 플랫폼 구성으로 보정을 끝냈고(offset 130740, 6.713 counts/g,
  162 g 기준, 분해능 약 ±6 g), 이후 **SparkFun SEN-21669 인장형(10 kg bar + hook) 2개 + HX711 2개**
  구성으로 재배선 중이다. 셀 선색은 빨강 E+, 검정 E-, 초록 A+, 흰색 A-.
- **IMU**: MPU-6050 2개를 하드웨어 I2C(bus 1, GPIO2/3)에 물린다. `AD0 → 3.3V` 쪽이 `0x69`, 비운 쪽이 `0x68`.
- **뎁스카메라**: D435I, 시리얼 261822073176, 펌웨어 5.15.1.55, librealsense v2.58.4를 **RSUSB 백엔드로
  소스 빌드**해서 쓴다 (Debian 13 apt에 패키지 없음, aarch64 pip 휠 없음).

---

## 진행 상황

| 항목 | 상태 |
|---|---|
| HX711 4셀 플랫폼 보정 | ✅ 완료 · 162 g 실측 152~162 g |
| HX711 인장형 2셀 재배선 | ⚠️ 헤더 스캔에서 2개만 응답(GPIO5/6, GPIO26/21). 모듈마다 새 영점 + 162 g 보정 필요 |
| MPU-6050 2개 각도 보정 | ✅ 완료 · 장착 방향(up 축) 인식까지, `test_remap.py` 6방향 전부 통과 |
| D435i depth | ✅ 완료 · 유효 depth 73.6 %, 0.18~4.69 m, 24.9 fps |
| 7" DSI 화면 | ❌ 패널이 안 켜짐 (백라이트 0x45 · 터치 0x38 무응답 → 디스플레이 보드 전원/리본 의심). 렌더링 · 스크린샷은 됨 |
| USB 스피커 | ✅ 완료 |
| 디지털 트윈 스캔 | ⚠️ 카트 + 화물 텍스처 메시 1차 완료(79장 중 46장 정합, 재투영 오차 0.82 px). **스케일 · 질량 미보정** |
| 센서 퓨전 / 위험도 판정 (`edge/`) | ⬜ 미착수 |
| FastAPI · WebSocket · 2.5D (`server/`) | ⬜ 미착수 |

---

## 개발 환경 — 장비 접속

라즈베리파이는 **Tailscale**로 붙는다 (MagicDNS 켜져 있음).

```bash
ssh <계정>@pi         # 로드셀 · IMU
ssh <계정>@display    # 화면 · 스피커 · D435i
```

- 계정명과 tailnet 주소는 팀 내부 공유 문서를 본다. **기본 계정 `pi`가 아니다.**
- 비대화형 ssh를 쓰려면 각자 공개키를 두 노드의 `authorized_keys`에 등록해야 한다.
- `sudo`는 비밀번호를 묻는다. `config.txt` 수정, `dtparam`, `apt` 같은 루트 작업은 사람이 직접 해야 한다.
- 두 노드는 공유 핫스팟 LAN으로 서로 직접 통신한다 (~11 ms).

---

## 모듈별 실행법

### 로드셀 · IMU — `pi/`

명령은 전부 `pi/` 폴더에서 실행한다. 폴더 구성은 [`pi/README.md`](pi/README.md),
환경변수 · 배선 · 로그 형식은 [`pi/acquisition/README.md`](pi/acquisition/README.md).

```bash
source acquisition/env.sh                       # 핀 / 라벨 / 역할 (환경변수)
python3 tools/check_wiring.py                   # HX711 배선 확인 (live 로 모서리 라벨 확정)
python3 calibration/calibrate.py tare           # 로드셀 영점 -> span / multi / cell 로 배율
python3 calibration/imu_calibrate.py --auto     # IMU rest pose 보정 -> imu_cal.json
python3 tools/imu_angle.py                      # IMU 실시간 roll/pitch (상보필터 α=0.98)
LOG_SECONDS=60 python3 pipeline.py              # 전 센서 한 시계로 -> 무게중심 -> 엑셀 (+ 트윈 노드가 보내면 _camera 엑셀)
python pipeline.py --sim                        # PC 에서 가짜 센서로
```

IMU 보정은 **"0도로 삼고 싶은 자세(rest pose)"** 로 두고 돌린다. 스크립트가 중력이 실린 축을 자동
감지해 `imu_cal.json`에 `up`으로 저장하고, 읽을 때 축을 돌려서 모듈이 세워져 있어도 각도가 맞는다.

### 화면 · 스피커 — `display:~/avtest`

```bash
pw-play ~/avtest/speakertest.wav
XDG_RUNTIME_DIR=/run/user/1000 WAYLAND_DISPLAY=wayland-0 SDL_VIDEODRIVER=wayland \
  python3 ~/avtest/disptest.py
grim ~/avtest/shot.png                       # 화면 캡처
```

### 뎁스카메라 — `display:~/cameratest`

```bash
sudo bash rs_deps.sh                         # 의존성 + udev 규칙 (1회)
./rs_py.sh test_d435i.py --headless --seconds 10   # 숫자 진단
./rs_py.sh ir_probe.py --out /tmp            # depth가 안 나올 때 IR 원본 확인
./rs_viewer.sh                               # GUI 뷰어 (DSI 화면으로 뜬다)
```

정상 기준: USB 3.2, 센서 목록에 Motion 포함(없으면 D435i가 아님), 유효 depth 70 % 이상,
640x480에서 24~30 fps, accel 크기 9.8 근처. 자세한 판별표는
[`display/cameratest/README.md`](display/cameratest/README.md).

### 디지털 트윈 — `viz/digital-twin`

```powershell
.\reconstruct.ps1 -Stage Align    # RealityScan 정합 -> 카메라/컴포넌트 육안 확인
.\reconstruct.ps1 -Stage Mesh     # 재구성 영역 지정 후 메시
```

이후 Blender로 `prepare_scan.py`(검수 뷰) → `crop_cart.py`(카트 크롭 + Y-up OBJ) →
`extract_cargo.py`(화물만 분리, 연결 컴포넌트 선택)를 거쳐 Unity 6000.0.84f1에서
`Digital Twin > Build Scan Scene`으로 씬을 만든다.

> 포토그래메트리는 **실제 크기도 질량도 알려주지 않는다.** 보이는 치수 하나를 실측해 스케일을
> 잡아야 하고, 무게중심 · 전도 해석에는 대차 치수 · 지지점 · 실측 하중이 따로 필요하다.

---

## 하드웨어 트러블슈팅 — 실제로 겪은 것들

**HX711 / 로드셀**
- 전원 인가 · 재배선 후 몇 분간 1000 counts 이상 드리프트한다. **영점 · 보정은 5분 이상 지난 뒤에.**
- 브리지 건전성 확인: gain 128과 64로 읽어 비율이 **약 2.0** 이어야 한다. 크게 벗어나면 입력이 떠 있다.
- GPIO2/3은 Pi 내장 풀업 때문에 스캔에서 항상 "구동 중"으로 보인다. HX711이 아니다.
- GPIO 한 쌍은 한 프로세스만 점유할 수 있다(lgpio busy). 로거를 끄고 계량해야 한다.

**MPU-6050 / I2C**
- **SDA/SCL이 서로 바뀌면 버스 전체가 죽는다.** 모든 주소가 EREMOTEIO, 커널에
  `i2c_designware ... timeout waiting for bus ready`. 선을 바로잡은 뒤
  `sudo dtparam i2c_arm=off && sleep 1 && sudo dtparam i2c_arm=on` 으로 컨트롤러를 리셋한다.
- **전원이 들어온 상태로 I2C 선을 옮기지 말 것.** 옮겼으면 위 리셋을 예상해야 한다.
- 두 칩을 구별하는 확실한 방법은 **자이로 바이어스 지문**이다 (한쪽 gx ≈ -1430, 다른 쪽 ≈ -215).
- "평평하게"는 사람 기준 브레드보드가 평평한 것이고, 헤더에 꽂힌 모듈은 그때 **세워져 있다.**
  보정 전에 어느 raw 축에 중력이 실렸는지 먼저 본다.
- `i2c_bitbang_scan.py`는 sudo 없이 스캔되지만 핀을 GPIO 모드로 남긴다. 끝나면 dtparam 리셋.

**D435i**
- **전면 보호필름**을 안 떼면 IR 상이 뭉개진다. 유효 depth 픽셀 2.3 % → 74.7 %. 이상하면 이것부터.
- 검은 광택면은 IR을 흡수해 depth가 안 잡힌다. 고장이 아니라 물리적 한계이고,
  **검은 비닐 · 광택 포장재 화물에서 같은 일이 생긴다.**
- 최소 측정 거리 0.28 m. 이보다 가까우면 전부 0이다.
- GUI는 X11(Xwayland)로 강제해야 한다. `WAYLAND_DISPLAY`를 그냥 `unset`하면 libwayland가 기본
  소켓 `wayland-0`으로 폴백해 결국 Wayland에 붙고, GLFW가 초당 200줄씩 에러를 뱉는다.

**SSH 원격 작업**
- Python 파일을 `ssh '... <<EOF'` 히어독으로 쓰면 f-string 안의 따옴표 · 백슬래시가 망가진다.
  로컬에서 쓰고 `scp` 하는 게 낫다.
- 프롬프트가 있을 수 있는 원격 스크립트는 `</dev/null` + `timeout` 으로 돌린다. `input()` 하나에 세션이 멈춘다.
- 백그라운드는 `setsid nohup ... </dev/null >out 2>&1 & disown`.
- `pkill -f logger.py` 는 ssh 셸 자신을 죽인다(exit 255). `pkill -f "^python3 logger"` 를 쓴다.

---

## 기여 규칙

- 기본 브랜치는 **`dev`** 다. 작업 브랜치는 `{이름}/{작업}` 형식 (예: `minhyung/digital-twin-assets`).
- 이슈 제목은 **`{feat/fix/refactor/chore} : 작업 내용`** 형식
  ([새 작업 템플릿](.github/ISSUE_TEMPLATE/feature_request.yml)).
- 버그는 [버그 리포트 템플릿](.github/ISSUE_TEMPLATE/bug_report.yml)으로, 파트를 골라서 올린다.
- PR은 `dev`로 보내고 [PR 템플릿](.github/PULL_REQUEST_TEMPLATE)을 채운다.
