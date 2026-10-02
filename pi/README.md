# pi — 센서 노드 (로드셀 · IMU · 깊이카메라)

**모든 명령은 `pi/` 폴더에서 실행한다.** (예: `python3 pipeline.py`)

## 파이프라인 (`pipeline.py`)

센서를 돌릴 때는 `pipeline.py` 하나를 실행한다. 단계마다 파일이 나뉘어 있고, 단계 사이는
**`Frame`(센서값)과 `Result`(판정 결과)로만** 주고받는다. **판정은 pi 에서 하고**, display 는 결과를 그린다.

```
sensors/ ──▶ acquisition/source.py ──Frame──▶ analysis/analyzer.py ──Result──▶ acquisition/log_xlsx.py
 드라이버       FrameSource.frames()   (~10Hz)   Analyzer(cfg)(frame)              CSV / 엑셀
                                                 (cog.py: 총하중 · 무게중심)       sink.write(frame, result)
```

| `Frame` 필드 | 내용 |
|---|---|
| `seq`, `t_ns`, `t_s`, `wall` | 순번, HX711 변환 완료 시각(monotonic ns), 시작부터 초, 벽시계 |
| `load_g` | 바닥 셀 `{라벨: g}` |
| `tension_n` | 장력 셀 `{라벨: N}` |
| `raw` | HX711 전 채널 `{라벨: count}` |
| `imu` | `{라벨: ImuReading}` — 구간 평균 `ax..gz`, 기울기 `roll/pitch`, 구간 최대 `ax_peak/ay_peak`, 샘플 수 `n` |
| `flags` | `sck_high`, `stuck_hi:<라벨>`, `imu_miss:<라벨>` 등 |

| `Result` 필드 | 내용 |
|---|---|
| `total_g` | 바닥 셀 합(g). 하나라도 값이 없으면 `None` |
| `cog` | `CoG(x, y, unit)`. 앞왼 셀 원점, `unit` 은 `"mm"`(BED_L/BED_W 있을 때) 또는 `"%"`. 모서리 값이 없거나 100 g 미만이면 `None` |

- 키는 설정(`HX_LABELS`, `IMU_LABELS`, `HX_TENSION`)으로 고정된다. 응답 없는 센서 · 보정 없는 채널은 값이 `None`.
- `analysis/` 와 기록 쪽은 `Frame` 만 보고 드라이버를 모른다. 실측과 `--sim` 이 같은 코드로 돈다.
- **판정을 추가할 때**: `Result` 에 필드를 하나 더하고 `Analyzer.__call__` 에서 채운다. 시간 추세가 필요하면
  상태를 `Analyzer` 에 둔다. 기록 · 전송 쪽은 `Result` 를 통째로 받으므로 함수 모양이 안 바뀐다.

| 폴더 | 파이프라인 단계 | 내용 |
|---|---|---|
| `sensors/` | ① 센서 값 받기 | 드라이버만. 값을 읽는 순간 `CLOCK_MONOTONIC` 으로 시각을 찍어 `(t_ns, 값)` 을 돌려준다 |
| `acquisition/` | ② 수집 · 정렬 · 기록 | 설정(`config.py`, `env.sh`), 프레임(`frame.py`, `source.py`), 기록(`log_xlsx.py` 엑셀, `log_sync.py` raw CSV). 자세한 건 [acquisition/README.md](acquisition/README.md) |
| `analysis/` | ③ 판정 | `analyzer.py` 입구(`Frame` → `Result`), `cog.py` 총하중 · 무게중심 |
| `calibration/` | 보정 | 로드셀 `calibrate.py` → `calib.json`, IMU `imu_calibrate.py` → `imu_cal.json` |
| `tools/` | 점검 · 진단 | 배선 확인, IMU 실시간 각도, I2C 진단. 평소엔 안 쓴다 |
| `twin/` | 디지털트윈 (D435i) | 깊이 프레임 → 2.5D 높이맵 → UDP. `acquisition/` 과 **별도 프로세스** |

## sensors/

| 파일 | 센서 | 비고 |
|---|---|---|
| `hx711_bus.py` | HX711 여러 개 (바닥 하중 4 + 벨트 장력) | 공유 SCK / 모듈별 SCK 둘 다. 전 채널 동시 샘플 |
| `mpu6050.py` | MPU-6050 (파이 I2C 직결) | `read_raw()` 는 raw LSB, `to_units()` 로 보정·축 회전 |
| `mpu6050_udp.py` | MPU-6050 (ESP32 → WiFi UDP) | `read_raw()` 모양이 위와 같다. 시각은 파이가 **받은** 시각 |
| `sim.py` | 위 센서 전부의 가짜 버전 | PC 에서 `--sim` 으로 파이 없이 전체 흐름 확인 |

`sensors/` 는 import 전용이다. 직접 실행하는 스크립트는 `acquisition/`, `calibration/`, `tools/` 에 있다.

## 빠른 시작

```bash
# PC 에서 (파이 없이)
python pipeline.py --sim

# 파이에서
source acquisition/env.sh                   # 핀 / 라벨 / 역할
python3 tools/check_wiring.py               # HX711 배선 확인
python3 calibration/calibrate.py tare       # 로드셀 영점
python3 calibration/imu_calibrate.py --auto # IMU 보정 (rest pose, 정지)
LOG_SECONDS=60 python3 pipeline.py
```

## IMU (MPU-6050)

### 배선
- 두 센서 공통: VCC → 3.3V, GND → GND, SDA → GPIO2 (물리 3번), SCL → GPIO3 (물리 5번)
- 주소 구분: AD0 → 3.3V 인 센서 = `0x69`, AD0 비운 센서 = `0x68`
- SDA/SCL 이 서로 바뀌면 버스 전체가 멈춘다 (두 센서 다 안 보임). 선 옮길 땐 전원 끄고.

### 보정 (`calibration/imu_calibrate.py`)
```bash
python3 calibration/imu_calibrate.py                     # 센서마다 rest pose 로 두고 Enter
python3 calibration/imu_calibrate.py --auto              # 프롬프트 없이 즉시 (정지 상태)
python3 calibration/imu_calibrate.py --auto --gyro-only  # 자이로 오차만 (정지만 필요)
python3 calibration/imu_calibrate.py --only bus1:0x68    # 한 센서만
```
- 센서를 "0도로 삼고 싶은 자세"(rest pose)로 놓고 돌린다.
- 중력이 실린 축(up)을 자동 감지한다. 모듈이 세워져 있어도(+x) 누워 있어도(+z) 된다.
- `calibration/imu_cal.json` 에 바이어스(ax..gz)와 up 축이 저장되고, 읽을 때 축을 돌려 rest pose 기준 roll/pitch 를 낸다.
- 진짜 수평계처럼 쓰려면 정말 수평인 면에서 보정한다. 브레드보드 위에서 보정하면 그 자세가 0도.
- ESP32 센서는 자동 보정이 안 된다. `imu_cal.json` 에 `"esp32:0x68"` 키로 직접 넣는다.
- 두 칩 구별: 자이로 바이어스 지문 (한쪽 gx ≈ -1430, 다른 쪽 ≈ -215).

### 확인 · 진단 (`tools/`)
```bash
python3 tools/imu_scan.py              # 버스별 장치 + 센서별 raw 1샘플
python3 tools/imu_angle.py             # 실시간 roll / pitch (상보필터 α=0.98)
python3 tools/imu_udp_receiver.py      # ESP32 무선 IMU 실시간 (UDP 5005)
python3 tools/test_remap.py            # 6개 장착 방향 x 30/20도 축 회전 검증 (PC 에서도 됨)

python3 tools/i2c_diag.py              # 주소별 errno, SDA/SCL 유휴 레벨, 전체 스캔
python3 tools/i2c_stuck.py             # 통신 중 SDA/SCL 고속 샘플 -> 컨트롤러 멈춤 vs 선이 눌림 구분
python3 tools/i2c_bitbang_scan.py      # GPIO2/3 을 소프트웨어 I2C 로 직접 흔들어 스캔 (sudo 불필요)
```
`i2c_bitbang_scan.py` 뒤에는 핀이 GPIO 모드로 남는다. 하드웨어 컨트롤러가
"timeout waiting for bus ready" 로 멈췄을 때도 같은 방법으로 리셋한다:
`sudo dtparam i2c_arm=off && sleep 1 && sudo dtparam i2c_arm=on`

## twin/ (RealSense D435i 디지털트윈)

로드셀/IMU 로거와 **따로 도는 프로세스**다. HX711 행에 섞이지 않고, 자기 패킷을 UDP 로 보낸다.

| 파일 | 내용 |
|---|---|
| `twin_core.py` | 순수 numpy 코어 (pyrealsense2 불필요). `world_rotation`, `deproject`, `HeightMapper`(2.5D), `voxel_downsample`, `TSDF`(3D, 비교용) |
| `pi_twin_node.py` | depth 848x480@30 → 필터 → 2cm 격자 높이맵(int16 mm) → UDP 5005 송신 |
| `twin_receiver.py` | 수신 예제. 패킷을 높이맵 numpy 배열 + 헤더로 복원 |
| `bench.py` | `out/` 의 저장 프레임으로 2.5D vs 3D 처리 시간 측정 → `out/bench_<arch>.json` |
| `pi_install_pyrealsense2.sh` | 소스 빌드한 librealsense(v2.58.4, RSUSB)를 `~/d435i/.venv` 에 설치 |

```bash
python3 twin/pi_twin_node.py 192.168.0.10 60   # 파이 -> PC 로 60초 송신 (기본 127.0.0.1, 무한)
python3 twin/twin_receiver.py                  # PC/디스플레이 쪽
python3 twin/bench.py                          # 처리 비용 (PC 에서도 됨)
```

- 시작할 때 1회: 카메라 내장 IMU 로 수평 보정, 첫 프레임에서 격자 범위와 바닥 기준면을 정한다. 설치 후 카메라를 움직이면 다시 시작.
- 패킷 = `[4B 헤더길이][JSON 헤더][zlib(int16 높이맵)]`. 헤더: `seq, t, frame_ts, grid, imu(accel, tilt_change_deg), sensors`
- `sensors` 는 아직 자리표시자 (`read_other_sensors()` 가 `None` 을 돌려준다). 로드셀 값과 묶는 방식과 송신 시점은 미정.
- 현재는 매 프레임 송신한다. 파이(aarch64) 기준 2.5D 높이맵 424x240 → 1cm 격자가 약 6 ms (`out/bench_aarch64.json`).
