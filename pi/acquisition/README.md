# pi/acquisition — HX711 + MPU-6050 동기 타임스탬프 로거

HX711 여러 개(공유 SCK)와 MPU-6050 2개를 **하나의 시계**로 기록한다.
핀 번호는 코드에 없고 전부 환경변수다.

드라이버는 `pi/sensors/`, 보정은 `pi/calibration/`, 배선 점검은 `pi/tools/` 에 있다.
**아래 명령은 전부 `pi/` 폴더에서 실행한다.**

## 배선

### HX711 — 공유 SCK

SCK 한 줄을 모든 모듈에 병렬로 물린다. DT 만 모듈마다 따로 파이로 간다.

```
 ┌──────────────────────────────────────────────┐
 │ +  ●●●●●●●●●●●●●●●●●●●●●●●●●   ← 3.3V   (파이 물리 1번)
 │ −  ●●●●●●●●●●●●●●●●●●●●●●●●●   ← GND    (파이 물리 39번)
 │
 │ +  ●●●●●●●●●●●●●●●●●●●●●●●●●   ← SCK 버스 (파이 물리 36번 = GPIO16)
 │ −  ●●●●●●●●●●●●●●●●●●●●●●●●●   ← 여분 GND
 └──────────────────────────────────────────────┘
```

일반 구멍은 5칸이 한 묶음이라 SCK 7가닥(모듈 6 + 파이 1)이 안 들어간다.
**전원 레일 한 줄을 SCK 전용으로 쓰고 테이프에 "SCK" 라고 써 붙인다.**
빨간 줄이라 무심코 5V 꽂기 쉽다.

모듈 1개당 4가닥: `VCC→3.3V레일`, `GND→GND레일`, `SCK→SCK레일`, `DT→파이 각자 핀`.

기본 핀 배치 (환경변수로 바꿀 수 있음):

| 용도 | BCM | 물리핀 |
|---|---|---|
| SCK (공유) | GPIO16 | 36 |
| DT c1 | GPIO5 | 29 |
| DT c2 | GPIO6 | 31 |
| DT c3 | GPIO13 | 33 |
| DT c4 | GPIO19 | 35 |
| DT c5 | GPIO26 | 37 |
| DT c6 | GPIO20 | 38 |
| 3.3V | — | 1 |
| GND | — | 39 |

**전원은 5V 가 아니라 3.3V.** 5V 로 주면 HX711 이 DT 를 5V 로 출력해서 5V 내성이
없는 파이 GPIO 로 들어가고, 반대로 HX711 의 입력 HIGH 문턱(≈0.7×VDD=3.5V)이
파이가 주는 3.3V SCK 보다 높아진다. 3.3V 로 주면 둘 다 해결된다.
대가는 여기압이 낮아져 신호가 ~66% 로 줄어드는 것뿐이고, 스케일 값만 바뀐다.

### HX711 — 모듈별 SCK (공유 배선을 못 할 때)

이미 모듈마다 SCK 가 따로 파이로 와 있으면 다시 납땜하지 않아도 된다.
`HX_SCK` 에 SCK 를 **DT 와 같은 순서로** 나열하면 드라이버가 `group_write` 로
**여러 SCK 를 호출 한 번에 같이 토글**한다. 커널이 한 번에 세팅하므로 채널 간
엣지 시차가 사실상 없고, 공유 배선과 같은 동시성을 얻는다.

```bash
HX_SCK=21,26,17,13 HX_DT=20,16,18,19 HX_LABELS=SW,SE,NW,NE python3 acquisition/log_sync.py
```

`describe()` 가 `SCK=group_write` 로 나오면 동시 토글이 걸린 것이다.
`gpio_write x4` 로 나오면 그룹을 못 잡아 핀별로 쓰는 중이고, 이때는 채널 간
수 us 시차가 생긴다(10SPS 기준 무시할 수준이지만 알고는 있을 것).

### MPU-6050

두 센서 모두 `VCC→3.3V`, `GND→GND`, `SDA→GPIO2`(물리 3), `SCL→GPIO3`(물리 5).
구분은 핀이 아니라 **주소**다: `AD0→3.3V` 인 쪽이 `0x69`, 비운 쪽이 `0x68`.
버스 하나에 주소가 둘뿐이라 2개가 한계다.

## 환경변수

`config.py` 의 docstring 이 원본이다. 요약:

| 변수 | 기본값 | 설명 |
|---|---|---|
| `HX_SCK` | `16` | 공유 SCK 핀 (BCM) |
| `HX_DT` | `5,6,13,19,26,20` | 모듈별 DT 핀 (BCM) |
| `HX_LABELS` | `c1,c2,...` | DT 순서대로 붙일 이름 |
| `HX_GAIN` | `128` | 128 / 64 / 32 |
| `HX_PULL` | `up` | DT 내부 풀. up 이면 죽은 채널이 HIGH 로 떠서 바로 잡힌다 |
| `HX_RATE` | `10` | 모듈 RATE 핀 설정과 맞출 것 (10 또는 80 SPS) |
| `HX_TIMEOUT` | `1.0` | DRDY 기다리는 한계(초) |
| `HX_CHIP` | 자동 | gpiochip 번호. 비우면 4 → 0 순으로 시도 |
| `HX_INVERT` | 없음 | 누르면 값이 줄어드는 채널 라벨. 드라이버가 부호를 뒤집는다. 바꾸면 `tare` 부터 다시 |
| `HX_TENSION` | `T1,T2` | 벨트 장력(인장형) 채널 라벨. 나머지는 바닥 하중 채널 |
| `COG_CORNERS` | `FL,FR,RL,RR` | 무게중심용 바닥 셀. 앞왼,앞오,뒤왼,뒤오 순서 |
| `BED_L` / `BED_W` | 없음 | 앞왼-앞오 / 앞왼-뒤왼 셀 간격(mm). 없으면 무게중심을 % 로 |
| `COG_CAUTION` / `COG_WARNING` / `COG_DANGER` | `25` / `15` / `7` | 무게중심이 판 가장자리에서 셀 간격의 이 % 안이면 주의 / 경고 / 위험 |
| `COG_RESID` / `COG_RESID_A` | `10` / `20` | IMU 예측 무게중심과 실측의 차 허용(셀 간격 %) = `COG_RESID` + `COG_RESID_A` x \|a\|(G). 허용의 1 · 1.4 · 1.8배부터 주의 · 경고 · 위험 (화물 이동) |
| `MOVE_WIN_S` / `MOVE_ALPHA` | `1.0` / `0.6` | 화물 이동을 볼 최근 구간(초)과, 그 구간에서 그 단계여야 하는 비율 |
| `MOVE_CONFIRM_S` / `MOVE_DANGER_S` | `1.5` / `3.0` | 이동이 이만큼(초) 이어지면 최소 경고 / 잔차가 위험 수준일 때 위험 |
| `COG_LAG` | `0.2` | 로드셀이 IMU 보다 늦는 시간(초) |
| `COG_H` | `190` | 무게중심 높이 h 시작값(mm). 주행 중 데이터로 고쳐 나가고, 화물 이동이 위험으로 확정되거나 화물을 내리면 이 값으로 돌아간다 |
| `JUDGE_IMU` | IMU 첫 번째 | 하중 이동 예측 · 무게 보정에 쓸 차체 IMU |
| `TENSION_A` / `TENSION_MU` / `TENSION_ANGLE` | `0.5` / `0.4` / `90` | 장력 하한 = 그쪽 무게 × g(A−μ)fs/(2μ sinα), fs 1.1 (`analysis/tension.py`). `HX_TENSION` 첫 번째 = N쪽(앞 두 셀), 두 번째 = S쪽(뒤 두 셀). 벨트 누름(장력 × 2 sinα)을 셀 합에서 뺄 때도 α 를 쓴다 |
| `TENSION_FLOOR` / `TENSION_MAX` | `2` / `80` | 장력 하한의 최솟값 / 상한(N, 벨트 · 셀이 견디는 힘). 하한의 100 · 75 · 50% 밑, 상한의 50 · 75 · 100% 넘음이 주의 · 경고 · 위험 |
| `IMU_BUS` | `1` | I2C 버스 |
| `IMU_ADDRS` | `0x68` | 파이 I2C 에 직접 물린 센서 주소 |
| `IMU_LABELS` | `imu1` | 주소 순서대로 붙일 이름 |
| `IMU_UDP_PORT` | `5005` | ESP32 무선 센서를 받을 UDP 포트. ESP32 펌웨어의 `PI_PORT` 와 같아야 한다. `CAM_PORT` 와 달라야 한다. `0` 이면 안 받음 |
| `IMU_UDP_LABEL` | `imu2` | ESP32 센서 이름 (CSV 열 `imu2_*`) |
| `IMU_UDP_ADDR` | `0x68` | ESP32 센서의 MPU 주소. 보정 키는 `esp32:<주소>` |
| `IMU_HZ` | `100` | 목표 샘플레이트 |
| `IMU_CAL` | 자동 | `imu_cal.json` 경로. 비우면 `calibration/imu_cal.json` |
| `CAM_PORT` | `5006` | `pipeline.py` 가 트윈 노드(D435i) 패킷을 받을 UDP 포트. 패킷이 오면 `<이름>_camera.xlsx`. `0` 이면 안 받음 |
| `LOG_DIR` | `acquisition/logs` | 로그 상위 폴더 |
| `LOG_SECONDS` | `0` | 기록 길이. 0 이면 Ctrl+C 까지 |
| `LOG_NAME` | 시각 | 런 이름 (`log_sync.py` 는 폴더, `pipeline.py` 는 파일 이름) |

```bash
# 설정 확인만
python3 acquisition/config.py

# 4채널만, 모서리 이름으로
HX_DT=5,6,13,19 HX_LABELS=FL,FR,RL,RR python3 acquisition/log_sync.py

# 60초 기록
LOG_SECONDS=60 python3 acquisition/log_sync.py
```

## 순서

```bash
# 0) I2C 살아있는지. 68/69 안 보이면 GPIO2/3 이 GPIO 모드로 남은 것
#    (진단 스크립트 뒤에 흔함) -> pinctrl set 2 a3 pu; pinctrl set 3 a3 pu
i2cdetect -y 1

# 1) 모듈 1개만 꽂고 확인 -> 되면 하나씩 추가
HX_DT=5 HX_LABELS=c1 python3 tools/check_wiring.py

# 2) 6개 다 붙었으면 전체 확인
python3 tools/check_wiring.py

# 3) 모서리를 하나씩 눌러 라벨 확정 (움직이는 채널에 * 가 붙는다)
python3 tools/check_wiring.py live

# 4) 기록 (raw CSV). 사람이 볼 엑셀은 아래 pipeline.py
HX_LABELS=FL,FR,RL,RR,T1,T2 LOG_SECONDS=60 python3 acquisition/log_sync.py
```

## 센서 전부 → 엑셀 (`pipeline.py`)

바닥 로드셀(하중)·벨트 장력·IMU 를 한 시계로 읽어 **시간순 엑셀** 로 남긴다.
한 행 = 한 `Frame` = HX711 샘플 한 번(약 10Hz). HX711 변환을 기다리는 ~100ms 동안 IMU 를
`IMU_HZ` 로 읽어 그 구간 평균을 같은 행에 붙인다.

| 파일 | 하는 일 |
|---|---|
| `frame.py` | `Frame` / `ImuReading` 정의. 단계 사이의 계약 |
| `source.py` | 센서 열기 · HX711 대기 루프 · IMU 구간 평균 → `FrameSource.frames()` 가 `Frame` 을 낸다 |
| `log_xlsx.py` | `XlsxSink`: `write(frame, result)` → CSV 한 행, 끝날 때 엑셀 |
| `log_camera.py` | `CameraLog`: 트윈 노드 UDP 패킷을 **별도 프로세스**로 받아 `<이름>_camera` CSV / 엑셀 ([아래](#카메라-엑셀-이름_cameraxlsx)) |
| `../analysis/analyzer.py` | `Analyzer(cfg)(frame)` → `Result` (총하중 · 무게중심 + 판정 4개 · 종합 위험도). [../README.md](../README.md#판정--위험도-analysis) |
| `../pipeline.py` | 위를 잇는 실행 파일 |

### PC 에서 먼저 (VS Code, 파이 없이)

`--sim` 을 주면 가짜 센서(`sim.py`)로 전체 흐름이 돈다. 18초쯤 제동하다 화물이 앞으로
밀리고 T1 벨트가 느슨해지는 장면, 41~45초에 4kg 을 더 실어 15kg 을 넘기는 장면이 들어 있다.
화면에 화물 이동 · 무게중심 · 장력 · 적재 알림이 한 번씩 뜨고, 엑셀 `판정` 시트에 남으면 정상이다.
판 크기는 `BED_L=250 BED_W=392`(env.sh 실측값)가 기본이고, h 추정값은 시나리오의 150mm 근처로 수렴한다.
가짜 카메라도 같이 돌아서 `sim_<시각>_camera.xlsx` 가 생기고, 같은 경과(초)에 부피중심이 앞오른쪽으로 옮겨진다.
가감속 중 로드셀 무게중심이 출렁이는 건 하중 이동이라 카메라 쪽에는 안 보인다.

```bash
pip install openpyxl numpy            # 처음 한 번 (numpy 는 가짜 카메라용)
cd pi
python pipeline.py --sim                          # Ctrl+C 로 끝. LOG_SECONDS=30 이면 30초
```

VS Code 실행 구성(F5)을 쓴다면 program 경로를 `pi/pipeline.py` 로 맞춘다.
결과는 `pi/acquisition/logs/sim_<시각>.xlsx`. 시뮬레이션 결과는 파일 이름이 `sim_` 으로
시작하고 요약 시트에도 "시뮬레이션" 이라고 적혀서 실측과 섞이지 않는다.

### 파이에서

HX711 원시값은 카운트라서 채널마다 영점과 배율이 있어야 g 가 된다
(`g = (raw - offset) * scale_g_per_count`). 장력 셀도 g 로 보정하고 로거가 N 으로
바꿔 적는다 (1 kgf = 9.81 N).

```bash
# BED_L / BED_W 는 예시. 실제 셀 간격(mm)으로
export HX_DT=5,6,13,19,26,20 HX_LABELS=FL,FR,RL,RR,T1,T2 BED_L=400 BED_W=600

python3 calibration/calibrate.py tare            # 판 비우고 벨트 풀고 -> 전 채널 영점
python3 calibration/calibrate.py span 1000       # 판 위에 1kg (위치 무관) -> 바닥 셀 공통 배율
python3 calibration/calibrate.py cell T1 1000    # T1 고리에 1kg 매달기 -> 장력 배율
python3 calibration/calibrate.py cell T2 1000
python3 calibration/calibrate.py show            # 확인

LOG_SECONDS=60 python3 pipeline.py               # 60초 -> acquisition/logs/<시각>.xlsx
```

- `span` 은 바닥 채널 **합**이 올린 무게와 맞게 공통 배율 하나를 구한다(장력 채널은 빠진다).
  셀마다 감도가 눈에 띄게 다르면 `cell FL 1000` 처럼 그 모서리 바로 위에 올려 하나씩 잡는다.
- 보정 전에 `check_wiring.py live` 로 모서리를 하나씩 눌러 **전 채널이 + 로 커지는지** 본다.
  반대로 가는 채널(셀이 뒤집혀 달렸거나 선색 규칙이 다른 셀)은 `HX_INVERT=SW` 처럼 적으면
  드라이버가 부호를 뒤집는다. A+/A- 를 바꿔 꽂은 것과 같다. 한 채널만 반대인 채로 `span` 을
  하면 합계에서 그 셀이 빠지는 셈이라 공통 배율부터 틀린다.
- `HX_INVERT` 를 바꾸면 그 채널의 원시값 부호가 바뀌므로 `tare` 부터 다시 한다. `calib.json` 에
  영점 잡을 때의 반전 상태가 남아서, 안 맞으면 `span`/`cell` 은 거부하고 로거는 그 채널을 raw 만 적는다.
- **열 구성은 `HX_LABELS` / `IMU_LABELS` 로 고정된다.** 한 행에 설정한 HX711 전 채널과 IMU 전부의
  자리가 항상 있고, 값이 없는 센서는 칸만 비운다. 실행마다 열이 달라지지 않는다.
- 보정값이 없는 채널은 g/N 칸이 비고 raw 만 채워진다 (요약 시트 메모).
- 시작할 때 DT 가 안 떨어지는 모듈은 읽기에서는 빼지만 열은 남긴다 (화면 경고 + 요약 시트에 "응답 없음").
  바닥 셀이 하나라도 비면 총하중 · 무게중심도 비운다.
- IMU 는 `calibration/imu_cal.json` 을 찾아 바이어스를 빼고 장착 자세(up 축)를 돌린다.
  못 찾으면 보정 없이 적고, 못 연 IMU 는 칸을 비운 채 나머지를 기록한다.
- SSH 로 길게 돌릴 땐 `tmux` 안에서. SSH 가 끊기면(SIGHUP) 그때까지를 엑셀로 만들고 끝난다.

### 엑셀 열 (`측정` 시트)

| 열 | 내용 |
|---|---|
| 시각, 경과(초) | 벽시계(ms), 시작부터 초 |
| FL(g) … 총하중(g) | 바닥 셀별 무게와 합계 |
| 무게중심 X, Y | 계획서 식. 앞왼 셀이 원점, X 는 앞왼→앞오, Y 는 앞왼→뒤왼 방향. `BED_L`/`BED_W` 를 주면 mm, 아니면 % (50 = 가운데). 총하중 100g 미만이면 빈칸 |
| T1 장력(N) … | 벨트 장력 |
| imu1 ax(G) … gz(°/s) | 행 구간 평균 |
| imu1 roll/pitch(°) | 상보필터 기울기 (`tools/imu_angle.py` 와 같은 방식). 가감속·선회 중엔 가속도가 섞여 틀어지므로 정차 중 값이 적재함 기울기 |
| FL raw … | HX711 원시값. 보정을 다시 하면 여기서 다시 계산할 수 있다 |
| flags | `sck_high`(타이밍 깨짐), `stuck_hi:FL`(모듈 응답 없음), `imu_miss:imu1` 등 |

`요약` 시트: 측정 시각·길이·속도, 채널별 역할·핀·보정값, 메모, 열마다
평균/최소/최대/표준편차/처음/마지막.

| 파일 | 내용 |
|---|---|
| `logs/<이름>.xlsx` | 끝날 때(Ctrl+C / `LOG_SECONDS` / kill / SSH 끊김) CSV 로 만든다 |
| `logs/<이름>.csv` | 기록하면서 1초마다 디스크에 쓴다. 전원이 나가도 여기까지 남고 엑셀로 바로 열린다 (UTF-8 BOM) |
| `logs/<이름>_camera.xlsx` / `.csv` | 카메라 패킷이 왔을 때만. [카메라 엑셀](#카메라-엑셀-이름_cameraxlsx) |
| `calibration/calib.json` | 채널별 offset / scale. `calibration/calibrate.py` 가 만든다 |

CSV 만 남았을 때(전원 차단 등) 엑셀 다시 만들기:

```bash
python3 -c "from acquisition import log_xlsx; log_xlsx.build_xlsx('acquisition/logs/X.csv', 'acquisition/logs/X.xlsx')"
```

### 카메라 엑셀 (`<이름>_camera.xlsx`)

D435i 는 센서 `Frame` 에 섞지 않는다. 트윈 노드(`twin/pi_twin_node.py`)가 따로 돌며 높이맵을 UDP 로
보내고, `pipeline.py` 가 기록하는 동안 `log_camera.py` 가 **별도 프로세스**로 받아 패킷 하나를 한 행으로 적는다.
같은 프로세스 스레드로 받으면 GIL 때문에 HX711 클럭이 밀릴 수 있어서다 (아래 설계 메모).

```bash
python3 twin/pi_twin_node.py                 # 카메라가 이 파이에 있으면 (기본 127.0.0.1:5006 = CAM_PORT. 5005 는 ESP32 IMU)
python3 twin/pi_twin_node.py <파이 주소>      # 카메라가 다른 노드(display)에 있으면 (그쪽 ~/d435i/.venv 에서)
LOG_SECONDS=60 python3 pipeline.py           # -> <이름>.xlsx + <이름>_camera.xlsx
```

트윈 노드를 먼저 띄워 두고 `pipeline.py` 를 돌린다. 패킷이 하나도 안 오면 카메라 파일은 안 만들고
센서 엑셀 요약 시트에 "카메라 패킷 없음" 만 남긴다.

| 열 | 내용 |
|---|---|
| 시각, 경과(초) | **이 파이가 받은 시각.** 경과(초) 0 이 센서 엑셀과 같아서 두 파일을 경과(초)로 바로 맞춰 볼 수 있다 |
| 유효 칸(%) | 높이맵에서 depth 가 잡힌 칸 비율 |
| 최고 / 평균 높이(mm) | 기준면 위 최고 높이, 화물 칸 평균 높이 |
| 적재 면적(m²), 부피(L) | 기준면보다 20 mm 넘게 높은 칸을 화물로 보고 합한 것 |
| 부피중심 X, Y(m) | 칸 높이로 가중한 중심. 밀도가 고르면 형상으로 본 무게중심. **카메라 격자 좌표**라 로드셀 무게중심과 원점 · 방향이 다르다 |
| 카메라 기울기 변화(°) | 트윈 노드 시작 때보다 카메라가 기운 각. 2° 를 넘으면 `moved` |
| 카메라 가속도 x/y/z(m/s²) | D435i 내장 IMU. 카메라 좌표 (x 오른쪽, y 아래, z 앞). 정지 상태에선 위쪽을 가리킨다 |
| loadcell_g … (트윈 노드) | 트윈 노드 헤더의 `sensors` 값 (지금은 자리표시자라 빈칸). 열은 첫 패킷의 키로 정한다 |
| seq, 카메라 프레임 시각(ms), 지연(ms), flags | 트윈 노드 순번, RealSense 프레임 타임스탬프, 보낸 시각 → 받은 시각 (다른 장비면 시계 차이 포함), `gap:N`(놓친 패킷) · `restart` · `moved` · `no_stats` |

`요약` 시트: 보낸 곳, 받은 / 놓친 프레임, 지연, 높이맵 격자 · 원점, 열마다 통계.

`트윈 기준` 시트: 격자 (nx, ny, 칸 크기, 원점 x0 · y0) + 설치 보정값 (기준면 높이 `ref_h`, 월드 회전 `R`, 설치 때 가속도,
depth intrinsics · `depth_scale`, 최대 거리, 스트림 · 필터, 카메라 시리얼, 녹화 파일 이름). 트윈 노드가 도중에 다시 시작하면
블록이 하나 더 생긴다. 칸 `(ix, iy)` 의 월드 좌표 = `(x0 + (ix+0.5)·칸, y0 + (iy+0.5)·칸, ref_h + 높이/1000)`. 월드 원점이 카메라, z 가 위쪽이다.

컬러 · depth 원본은 카메라 엑셀이 아니라 트윈 노드의 녹화 파일(`<시각>_d435i.db3`, 기본 `logs/`)에 있다.
카메라 엑셀의 `카메라 프레임 시각(ms)` 열이 그 파일의 depth 프레임 타임스탬프와 같아서, 이걸로 센서 엑셀 · 카메라 엑셀 · 녹화를 맞춘다.

CSV 만 남았을 때는 `log_camera.build_xlsx('X_camera.csv', 'X_camera.xlsx')` 로 다시 만든다 (`트윈 기준` 시트는 빠진다).

## `log_sync.py` 가 만드는 파일

`LOG_DIR/<런 이름>/`

| 파일 | 내용 |
|---|---|
| `load.csv` | `t,t_ns,seq,<라벨...>,sck_us,flags` — 한 행 = **전 채널 동시 샘플** |
| `imu.csv` | `t,t_ns,imu,ax,ay,az,gx,gy,gz,temp` — raw LSB |
| `events.log` | 타임아웃 / 경고 |
| `meta.json` | 시계 기준점, 핀 배치, 변환계수, 통계 |

`load.csv` 가 wide 인 이유: 공유 SCK 라 6채널이 정말 같은 순간의 값이다.
타임스탬프 하나가 전 채널에 유효하므로 굳이 쪼갤 이유가 없다.
`imu.csv` 는 센서마다 읽는 시각이 ~1ms 다르므로 센서별 행으로 남긴다.

### 시계

전부 `CLOCK_MONOTONIC`(ns). 시스템 전체 시계라 프로세스가 달라도 같은 값이고,
NTP 가 시각을 점프시켜도 흔들리지 않는다. 절대시각이 필요하면 `meta.json` 으로:

```
wall = t0_wall_unix + (t_ns - t0_mono_ns) / 1e9
```

`t` 열은 이미 `(t_ns - t0_mono_ns)/1e9` 인 상대 초다.

### 두 파일 정렬

레이트가 다르니(로드셀 10Hz, IMU 100Hz) 캡처 단계에서 한 행으로 합치지 않는다.
분석할 때 붙이는 게 맞다:

```python
import pandas as pd
load = pd.read_csv("load.csv")
imu  = pd.read_csv("imu.csv").query("imu == 'imu1'")
m = pd.merge_asof(load.sort_values("t"), imu.sort_values("t"),
                  on="t", direction="nearest", tolerance=0.05)
```

## 설계 메모

**왜 프로세스를 나누는가.** HX711 은 `PD_SCK` 가 60us 넘게 HIGH 면 파워다운으로
들어간다. 파이썬 스레드로 묶으면 GIL 때문에 클럭 버스트 도중 선점당해 SCK 가
수 ms 동안 HIGH 로 머물 수 있고 그 샘플이 깨진다. HX711 수집 / IMU 수집 /
파일 쓰기를 별 프로세스로 두면 그 간섭이 없다. 추가로
`os.sched_setscheduler(SCHED_FIFO)` 를 시도하는데 root 가 필요하므로,
타이밍이 불안하면 `sudo -E python3 acquisition/log_sync.py` (환경변수 유지) 로 돌린다.

**그래도 넘칠 수 있으므로** 매 샘플마다 실제 SCK HIGH 폭을 재서 `sck_us` 열에 남기고,
60us 를 넘으면 `flags` 에 `sck_high` 를 붙인다. 이 플래그가 많으면 그 구간 값은
믿지 말 것.

**언제 타임스탬프를 찍는가.** 모든 DT 가 LOW 로 떨어진 순간 = 변환이 끝난 시각이다.
24비트를 클럭해 내보내기 **전**에 찍는다. 다 읽고 나서 찍으면 클럭 시간만큼 늦다.

**공유 SCK 가 위상을 맞춰준다.** 25번째 펄스가 끝나는 순간부터 모든 칩의 다음 변환이
같이 시작되므로, 읽을 때마다 6개의 변환 위상이 저절로 정렬된다. 시작할 때와
타임아웃 복구 시에는 `sync_reset()`(SCK 를 길게 HIGH → 전 모듈 동시 파워다운 →
내리면 동시 재시작)으로 명시적으로 맞춘다.

**`group_read`.** DT 여러 개를 lgpio 그룹으로 잡아 호출 한 번에 비트마스크로 읽는다.
비트당 호출이 1/n 로 줄고 채널 간 미세 시차도 없다. 이 API 가 안 먹으면 핀별
`gpio_read` 로 자동 전환하고 `describe()` 에 이유를 적는다.

**`HX_PULL=up` 인 이유.** GPIO4~27 은 기본 풀다운이라, 모듈이 죽어 DT 가 플로팅이면
LOW = "준비됨" 으로 오인된다. 풀업이면 HIGH(=안 준비됨)로 남아 타임아웃으로 잡히고
어느 채널인지 `events.log` 에 남는다.

## 주의

- 브레드보드 24가닥은 차량이 움직이면 접촉 불량이 난다. 주행용으로는 만능기판에
  납땜하거나 최소한 SCK 버스를 고정할 것.
- `sudo` 로 돌릴 때는 `-E` 를 붙여야 환경변수가 유지된다.
