# SCSS 실시간 송신 (센서 파이)

센서 값을 모아 위험 판정을 계산하고, D435i 카메라 높이맵 · 실측 표면과 **한 프레임으로 묶어 0.1 초마다**
디스플레이 파이(`display/twin`)로 LAN 을 통해 보낸다. 디스플레이는 받는 즉시 디지털 트윈으로 그린다.

```
pi/live/
├─ run_live.sh       켜기 (acquisition/env.sh 를 읽고 두 프로그램을 띄운다)
├─ stop_live.sh      끄기
├─ rec.sh            실측 구간 저장: start [이름] / stop / status
├─ sensor_feed.py    센서(로드셀 4 · 장력 2 · IMU) 읽기 + analysis 4단계 판정 -> UDP 5020 · 멀티캐스트 5021
│                    적재 · 주행 모드를 UDP 5023 으로 받는다 (디스플레이 단추)
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

## 적재 · 주행 모드

디스플레이 오른쪽 막대의 `적재` / `주행` 단추가 UDP 5023 으로 `mode load` / `mode drive` 를 보내고,
sensor_feed.py 가 다음 프레임부터 `Analyzer.set_mode` 로 바꾼다. 켤 때는 **적재**. 지금 모드는 `sensors` 의 `mode` 열로 나간다.

| | 적재 | 주행 |
|---|---|---|
| 판정 · 화면 | 같다 | 같다 |
| 경고음 · 주의음 | 안 낸다 (디스플레이) | 낸다 |
| 벨트 k · 누름 자리 x, y | 조일 때마다 추정 | 고정 |

- **적재 -> 주행**: 지금 멈춘 구간을 바로 확정해 벨트 k, x, y 를 고정하고(마지막 조임은 원래 다음 변화가 와야 확정된다),
  화물 이동 기준을 다시 잡는다 (`rebase`, 직후 1 초 판정 보류). 화물이 가만히 있을 때 누른다.
- **주행 -> 적재**: 벨트 추정을 다시 켠다. 주행 전 기준은 버리고 2 초 넘게 가만히 있는 구간을 새 기준으로 잡는다.
- 벨트마다 추정에 쓴 조임 횟수는 `c1_steps` / `c2_steps` (0 = 시작값 그대로).
- **기준 재설정 단추**: 같은 UDP 5023 으로 `rebase` 를 받아 다음 프레임에 `Analyzer.rebase()` 를 부른다 (모드는 그대로).
  화물 이동 기준 · 학습한 h · 붙잡고 있던 화물 이동 알림을 지우고 직후 1 초 판정을 보류한다. 과적 · 무게중심 · 장력 판정은 건드리지 않는다.
  그 프레임의 `flags` 에 `rebase` 가 남는다. 적재 -> 주행 전환과 같이 오면 전환이 이미 rebase 하므로 한 번만 한다.
  (실시간 연결이 아니면 디스플레이가 보내지 않는다.)

```bash
echo -n 'mode drive' | nc -u -w1 127.0.0.1 5023    # 손으로 바꾸기 (답: {"ok": true, "mode": "drive"})
echo -n 'rebase'     | nc -u -w1 127.0.0.1 5023    # 기준 재설정 손으로 (답: {"ok": true, "rebase": true})
```

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
