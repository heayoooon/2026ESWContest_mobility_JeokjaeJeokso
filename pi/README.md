# pi — 센서 노드 (로드셀 · IMU)

**모든 명령은 `pi/` 폴더에서 실행한다.** (예: `python3 acquisition/log_xlsx.py`)
각 폴더 안으로 들어가서 `python3 log_xlsx.py` 처럼 돌려도 된다.

| 폴더 | 파이프라인 단계 | 내용 |
|---|---|---|
| `sensors/` | ① 센서 값 받기 | 드라이버만. 값을 읽는 순간 `CLOCK_MONOTONIC` 으로 시각을 찍어 `(t_ns, 값)` 을 돌려준다 |
| `acquisition/` | ② 타임스탬프 정렬 · 수집 · 기록 | 설정(`config.py`, `env.sh`), 로거(`log_sync.py`, `log_xlsx.py`). 자세한 건 [acquisition/README.md](acquisition/README.md) |
| `calibration/` | 보정 | 로드셀 `calibrate.py` → `calib.json`, IMU `imu_calibrate.py` → `imu_cal.json` |
| `tools/` | 점검 · 진단 | 배선 확인, IMU 실시간 각도, I2C 진단. 평소엔 안 쓴다 |

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
python acquisition/log_xlsx.py --sim

# 파이에서
source acquisition/env.sh                   # 핀 / 라벨 / 역할
python3 tools/check_wiring.py               # HX711 배선 확인
python3 calibration/calibrate.py tare       # 로드셀 영점
python3 calibration/imu_calibrate.py --auto # IMU 보정 (rest pose, 정지)
LOG_SECONDS=60 python3 acquisition/log_xlsx.py
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
