# SCSS 실시간 송신 (센서 파이)

센서 값을 모아 위험 판정을 계산하고, D435i 카메라 높이맵 · 실측 표면과 **한 프레임으로 묶어 0.1 초마다**
디스플레이 파이(`display/twin`)로 LAN 을 통해 보낸다. 디스플레이는 받는 즉시 디지털 트윈으로 그린다.

```
pi/live/
├─ run_live.sh       켜기 (acquisition/env.sh 를 읽고 두 프로그램을 띄운다)
├─ stop_live.sh      끄기
├─ rec.sh            실측 구간 저장: start [이름] / stop / status
├─ sensor_feed.py    센서(로드셀 4 · 장력 2 · IMU) 읽기 + analysis 4단계 판정 -> UDP 5020 · 멀티캐스트 5021
├─ live_twin.py      D435i 깊이 · 컬러 + 최신 판정 -> 디스플레이 TCP 5010 (10 Hz)
├─ twin_frame.py     높이맵 만들기 · 프레임 보내기
└─ watch_sensors.py  센서 값 동시에 보기 (멀티캐스트, 몇 개든)
```

```
 HX711 · MPU-6050 ─▶ sensor_feed.py ──UDP 5020──▶ live_twin.py ──TCP 5010 (LAN)──▶ 디스플레이 파이
   (pi/acquisition)   (pi/analysis 판정)  │           ▲ D435i                        display/twin
                                          └─ 멀티캐스트 239.255.42.21:5021 ─▶ watch_sensors.py (누구나)
```

## 실행

```bash
cd pi/live
./run_live.sh                 # 디스플레이 기본 192.168.10.2 (LAN). 다른 곳이면 ./run_live.sh <주소>
./stop_live.sh                # 끄기
python3 watch_sensors.py      # 값 보기 (--json 이면 한 행 전부)
```

- 디스플레이 파이에서는 `display/twin/launch.sh` (바탕화면 아이콘) 를 켜 둔다.
- **pipeline.py 와 같이 못 돌린다** (HX711 GPIO 는 한 프로그램만 잡을 수 있다). `run_live.sh` 가 pipeline.py 를 먼저 끈다.
  값만 보고 싶으면 `watch_sensors.py` 를 쓴다 (센서를 건드리지 않아서 동시에 몇 개든 된다).
  `calibrate.py` 처럼 원시 값이 필요한 일만 `stop_live.sh` 로 끄고 한다.
- 카메라 쪽은 pyrealsense2 가 있는 파이썬으로 돈다: 기본 `~/d435i/.venv/bin/python` (`CAM_PY=...` 로 바꾼다).
- IMU1(MPU-6050, I2C 0x68)이 안 읽히면 대개 GPIO2/3 이 I2C 기능에서 풀린 것이다.
  `run_live.sh` 가 켜기 전에 `pinctrl set 2 a3 pu; pinctrl set 3 a3 pu` 로 되돌린다.
- 카메라 바닥 기준: 켤 때 판 윗면(카메라 아래 약 0.579 m)을 찾아 그보다 0.208 m 아래를 0 으로 둔다
  (`--plate-z`, `--plate-above-floor`). 판을 못 찾으면 기본값을 쓴다.

## 실측 구간 저장

```bash
./rec.sh start 급정거        # 지금부터 저장 -> pi/live/runs/<시각>_급정거/
./rec.sh stop               # 끝 (행 수 · 시간 표시)
./rec.sh status
```

폴더마다 `sensors.csv` (센서 · 판정, 0.1 초마다, pipeline.py CSV 와 같은 열 이름), `twin.tws` (디스플레이로 보낸 프레임 그대로),
`summary.json`. 켜져 있는 동안의 전체 기록은 `logs/<시각>_live.csv`.

## 보내는 값

`sensors` 열 이름 · 프레임 형식은 [display/twin/README.md](../../display/twin/README.md) 참고.
판정은 `pi/analysis` (적재 · 무게중심 · 화물 이동 · 앞 벨트 · 뒤 벨트, 0 정상 · 1 주의 · 2 경고 · 3 위험)
그대로이고, 기준은 `acquisition/config.py` · `analysis/*.py` 상수를 따른다.
