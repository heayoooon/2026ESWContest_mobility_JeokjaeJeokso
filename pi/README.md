# pi — 센서 노드 (로드셀 · IMU · 깊이카메라)

**모든 명령은 `pi/` 폴더에서 실행한다.** (예: `python3 pipeline.py`)

## 파이프라인 (`pipeline.py`)

센서를 돌릴 때는 `pipeline.py` 하나를 실행한다. 단계마다 파일이 나뉘어 있고, 단계 사이는
**`Frame`(센서값)과 `Result`(판정 결과)로만** 주고받는다. **판정은 pi 에서 하고**, display 는 결과를 그린다.

```
sensors/ ──▶ acquisition/source.py ──Frame──▶ analysis/analyzer.py ──Result──▶ acquisition/log_xlsx.py
 드라이버       FrameSource.frames()   (~10Hz)   Analyzer(cfg)(frame)              CSV / 엑셀
                                                 총하중 · 무게중심 + 판정 4개      sink.write(frame, result)
                                                 -> 위험도 정상/주의/위험          (판정 열 + '판정' 시트)

twin/pi_twin_node.py ──UDP──▶ acquisition/log_camera.py (별도 프로세스) ──▶ <이름>_camera 엑셀
 D435i 높이맵 + 요약 (~15-30fps)  패킷 하나 = 한 행. 경과(초) 0 이 센서 엑셀과 같다
      └──▶ <시각>_d435i.db3  컬러 + depth 원본 녹화 (카메라 엑셀의 카메라 프레임 시각(ms) 으로 맞춘다)
```

카메라는 `Frame` 에 넣지 않고 **엑셀을 따로** 만든다. `pipeline.py` 가 도는 동안 트윈 노드 패킷이 오면
끝날 때 `<이름>.xlsx` 옆에 `<이름>_camera.xlsx` 가 생긴다. 안 오면 안 생긴다 (`CAM_PORT`, 기본 UDP 5006).

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
| `cog` | `CoG(x, y, unit)`. 화물 무게중심 = 벨트 누름을 벨트마다 추정한 누름 자리(x, y)에서 뺀 값 (`cog.py`, `belt_fit.py`). 앞왼 셀 원점, `unit` 은 `"mm"`(BED_L/BED_W 있을 때) 또는 `"%"`. 모서리 값이 없거나 화물이 100 g 미만이면 `None` |
| `load_kg` | 적재 무게(kg). 셀 합에서 벨트 누름(장력 × 2 sinα)을 뺀 화물 무게. IMU `az` 로 나눠 위아래 출렁임을 빼고 평활 |
| `measured_kg` | 측정 무게(kg). 셀 합 그대로(벨트 누름 포함), `az` 로 나누고 평활 |
| `transfer` | `Transfer(pred_x, pred_y, kx, ky)` — IMU 로 예측한 무게중심과 추정한 하중 이동 계수. mm 단위면 `k` 가 곧 무게중심 높이 h(mm) |
| `tension_min` | `{장력 라벨: N}` 그 벨트가 맡은 무게로 정한 장력 하한 |
| `belts` | `{장력 라벨: Belt(k, x, y, steps)}` 벨트마다 추정한 누름 계수 k(장력 1N 이 누르는 힘 N)와 누름 자리. 시작값은 `BELT_K`, `BELT_N_Y`/`BELT_S_Y` (x 는 모름 = `None`), `steps` 는 추정에 쓴 조임 횟수 (`belt_fit.py`) |
| `checks` | `[Check(key, name, level, text)]` 판정마다 하나. `level` 0 정상 · 1 주의 · 2 경고 · 3 위험 · `None` 판정 불가 |
| `level` | 종합 위험도 = 적재 · 무게중심 · 장력 단계를 합친 것 ([아래](#종합-위험도-analysisoverallpy)). **화물 이동은 안 들어간다** (`checks` 의 `move` 로 따로 알린다) |
| `changed` | `[(Check, 이전 레벨)]` 이번 프레임에 알림 레벨이 바뀐 판정 (화면 알림 · 엑셀 '판정' 시트). 종합이 바뀌면 key `risk` 인 `Check` 가 붙는다 |

- 키는 설정(`HX_LABELS`, `IMU_LABELS`, `HX_TENSION`)으로 고정된다. 응답 없는 센서 · 보정 없는 채널은 값이 `None`.
- `analysis/` 와 기록 쪽은 `Frame` 만 보고 드라이버를 모른다. 실측과 `--sim` 이 같은 코드로 돈다.
- **판정을 추가할 때**: `Result` 에 필드를 하나 더하고 `Analyzer.__call__` 에서 채운다. 시간 추세가 필요하면
  상태를 `Analyzer` 에 둔다. 기록 · 전송 쪽은 `Result` 를 통째로 받으므로 함수 모양이 안 바뀐다.

| 폴더 | 파이프라인 단계 | 내용 |
|---|---|---|
| `sensors/` | ① 센서 값 받기 | 드라이버만. 값을 읽는 순간 `CLOCK_MONOTONIC` 으로 시각을 찍어 `(t_ns, 값)` 을 돌려준다 |
| `acquisition/` | ② 수집 · 정렬 · 기록 | 설정(`config.py`, `env.sh`), 프레임(`frame.py`, `source.py`), 기록(`log_xlsx.py` 엑셀, `log_sync.py` raw CSV). 자세한 건 [acquisition/README.md](acquisition/README.md) |
| `analysis/` | ③ 판정 | `analyzer.py` 입구(`Frame` → `Result`), `cog.py` 총하중 · 무게중심 · 구역, `total.py` 적재, `transfer.py` h 추정 · 화물 이동, `tension.py` 장력, `belt_fit.py` 벨트 누름 k · 자리 추정, `overall.py` 종합 위험도, `risk.py` 위험도 공통 ([아래](#판정--위험도-analysis)) |
| `calibration/` | 보정 | 로드셀 `calibrate.py` → `calib.json`, IMU `imu_calibrate.py` → `imu_cal.json` |
| `tools/` | 점검 · 진단 | 배선 확인, IMU 실시간 각도, I2C 진단. 평소엔 안 쓴다 |
| `twin/` | 디지털트윈 (D435i) | 깊이 프레임 → 2.5D 높이맵 → UDP. `acquisition/` 과 **별도 프로세스** |

## 판정 · 위험도 (`analysis/`)

프레임마다 판정 4개를 하고 각각 **정상 / 주의 / 경고 / 위험** 을 낸다. 종합 위험도는 그중 적재 · 무게중심 · 장력의
단계를 합친 것이고([아래](#종합-위험도-analysisoverallpy)), 화물 이동은 실제로 움직였을 때 따로 알린다.
기준값은 환경변수다 (`acquisition/config.py` 머리말, `python3 acquisition/config.py` 로 확인). 적재 · 장력 단계 기준과 화물 이동 허용 배수만 `analysis/total.py` · `analysis/tension.py` · `analysis/transfer.py` 맨 위 상수.

| 판정 | 파일 | 주의 | 경고 | 위험 |
|---|---|---|---|---|
| 적재 | `total.py` | 적정 `RATED_KG`(15kg) 의 110% 넘음 | 125% 넘음 | 140% 넘음 |
| 무게중심 | `cog.py` `zone()` | 가장자리에서 셀 간격 `COG_CAUTION`(15%) 안 | `COG_WARNING`(8%) 안 | `COG_DANGER`(3%) 안 |
| 화물 이동 | `transfer.py` | IMU 예측과 실측 차가 허용(셀 간격 `COG_RESID`(7%) + `COG_RESID_A`(20%/G) x \|a\|) 넘음, 최근 `MOVE_WIN_S`(1초)의 `MOVE_ALPHA`(60%) 이상 | 1.4배 넘음 (`MOVE_CONFIRM_S`(1.5초) 이어져도 경고) | 1.8배 넘음 (`MOVE_DANGER_S`(3초) 이어져도 위험) |
| 장력 N / S | `tension.py` | 하한의 105% 밑 / `TENSION_MAX`(80N) 의 50% 넘음 | 95% 밑 / 75% 넘음 | 90% 밑 / 90% 넘음 |

- **적재**: 셀 합에서 벨트가 화물을 누른 몫(장력 × 벨트마다 추정한 k / g)을 빼고 차체 IMU `az` 로 나눠 위아래 출렁임을 뺀다.
  110% = 도로교통법 시행령 제22조 허용 한도, 140% = 도로법 중대 위반(40% 이상 초과). 화물이 넘어지며 벨트를 당길 때처럼 1~2초 튀는
  값에는 안 뜨게 2초 이어져야 알린다.
- **화물 이동** (주행 중 h 추론 → IMU 로 무게중심 변화가 맞는지): 가속도 a(G)로 달리면 무게중심이 h × a 만큼 옮겨진다.
  `X = X0 + kx·(ay − ay0)`, `Y = Y0 + ky·(ax − ax0)` (IMU 는 `COG_LAG` 0.2초 전 값, mm 단위면 k = h).
  가감속이 0.08G 넘는 프레임마다 원점을 지나는 직선을 맞춰 k(=h)를 고쳐 나가고(2026-10-03 급출발에서 k 를 맞춘 방법),
  `COG_H_LEARN=0` 이면 학습하지 않고 h 를 150mm 로 고정한다 (주행이 없는 시연용, 기본은 학습). h 값은 환경변수가 아니라 `acquisition/config.py` 의 `H_START_MM`(190) / `H_FIXED_MM`(150) 이다.
  학습할 때 시작값은 190mm 이고, 화물 이동이 위험으로 확정되거나 화물을 내리면(무게중심이 2초 넘게 없음) 학습한 h 를 버리고 시작값으로 돌아간다.
  예측과 실측의 차가 크면 가감속으로 설명이 안 되는 변화 = 화물이 미끄러지거나 떨어진 것이다.
  화물이 새 자리에서 4초 조용히 있으면 그 자리를 새 기준으로 받아들이고, 화물 이동 알림은 풀린 뒤에도 8초 유지한다.
  **다만 `MOVE_LATCH=1`(기본)이면 경고 이상으로 확정된 이동은 이 자동 복귀 없이 사람이 기준 재설정(또는 적재 → 주행 전환)을 누를 때까지 붙잡는다**
  (`Analyzer._latch_move`): 그동안의 최고 단계를 유지하고, 새 자리를 기준으로 받아들여 판정이 정상으로 돌아와도, 화물이 판에서 떨어져
  무게중심이 없어져도 유지한다. 주의는 한 번의 흔들림일 수 있어 붙잡지 않는다. 붙잡은 동안 화면에 나가는 문구는 그 순간의 이동량(실측 − 예측)으로 만든다.
  또 **주행 모드에서는 위의 "새 자리 4초 → 새 기준"도 끈다** (`TransferCheck.accept_new_spot=False`): 기준이 원위치에 남아 이동량이 원위치 대비로
  이어지고, 화물이 돌아오면 0 으로 돌아온다 (경보 단계는 붙잡은 채). 적재 모드는 싣고 내리는 중이라 예전대로 받아들인다. `MOVE_LATCH=0` 이면 위의 자동 복귀만 한다.
  **기준 재설정**(`Analyzer.rebase()`): 지금 상태를 정상 기준으로 삼는다. 화물 이동 판정만 지운다 (기준 위치 · 학습한 h · 붙잡고 있던 알림 · `MOVE_LATCH` 로 붙잡은 경보).
  직후 1초(`transfer.REBASE_MUTE_S`)는 화물이 자리 잡을 때까지 화물 이동을 판정하지 않으니, **그 1초 안에 쓰러진 화물은 못 잡는다** (새 기준에 그대로 흡수된다). 화물이 가만히 있을 때 누른다.
  과적 · 무게중심 쏠림 · 장력 풀림은 지금 상태를 그대로 보는 판정이라 재설정해도 위반이면 계속 뜬다.
- **장력**: `HX_TENSION` 순서대로 첫 번째 = N(앞)쪽 벨트, 두 번째 = S(뒤)쪽 벨트.
  벨트마다 따로 판정해서 화면에는 `장력 N(c1)`, `장력 S(c2)` 로 뜬다.
  하한 = 맡은 무게 × g(`TENSION_A` − `TENSION_MU`) fs / (2 `TENSION_MU` sin`TENSION_ANGLE`) (EN 12195-1 식 10,
  안전계수 fs 1.1, 최소 1kg 당 1N · `TENSION_FLOOR`). 기본값(0.5G, μ 0.5, 90°)이면 미끄럼 항이 0 이라 최솟값 1 N/kg (μ 0.4 면 1.35 N/kg).
  맡은 무게 = 화물 무게를 무게중심 위치에 따라 두 벨트(누름 y, 시작값 `BELT_N_Y`, `BELT_S_Y`)가 지렛대처럼 나눈 몫이다.
  벨트가 누른 힘은 그 벨트 위치에서 셀에 실리므로, 화물 무게와 무게중심을 낼 때 그 위치에서 뺀다
  (안 빼면 세게 조일수록 하한도 올라간다). 이 무게중심(`cog.py`)은 무게중심 · 화물 이동 판정과 화면에도 그대로 쓰여서,
  화물은 그대로인데 장력만 바꾸면 무게중심이 안 움직인다. 벨트 위치가 기본값(셀 줄 위)이면 앞쪽 두 셀 / 뒤쪽 두 셀 무게와 같다.
  벨트 누름 계수 k 와 누름 자리 x, y 는 정차 중 벨트를 조일 때 자동으로 추정한다 (`belt_fit.py`, `BELT_FIT=1`).
  장력 · 셀 합 · IMU 가 0.3초 넘게 안 움직이면 멈춘 구간으로 보고, 조이기 전 · 뒤 멈춘 구간의 차로 k = Δ셀 합 / ΔT,
  x · y = Δ모멘트 / (k ΔT) 를 낸다 (두 벨트가 같이 바뀌면 최소제곱). 언제 다 조였는지 알려줄 필요 없이, 다음 벨트를 건드리면
  그 전까지의 멈춘 구간으로 낸다. 주행 중 장력 흔들림, 장력이 준 변화(화물 밀림), 8N 보다 작은 변화, 손으로 화물을 누른 구간은 안 쓴다.
  2026-10-25 장력꽉 기록: 고정값(k 1.9, y −20 / 280)이면 무게중심 오차 평균 6.3% · 최대 10.3%,
  추정하면 1.3% · 2.4% (다른 회차에서 추정한 값을 넣은 교차검증).
  상한 `TENSION_MAX` 는 벨트 · 셀이 견디는 힘(EN 의 LC)이다. 조임 장력은 그 50% 까지라 넘으면 과조임 주의.
  하한 쪽은 EN 하한(100%) 앞뒤 5% 로 나눠 105% 밑 주의, 95% 밑 경고, 90% 밑 위험이다.
  상한 쪽은 50% 넘으면 과조임 주의, 75% 경고, 상한에 닿기 전 90% 에서 위험이다 (2026-10-31 발표 자료 기준.
  그 전: 하한 100 · 75 · 50%, 상한 50 · 75 · 100%. 근거: [`analysis/tension_EN12195-1.md`](analysis/tension_EN12195-1.md)).
- 알림: 화면 상태줄 맨 앞에 `[위험: 경고 적재,장력 S(T2) · 주의 장력 N(T1)] [화물 이동 위험]` 처럼 종합 위험도와
  걸린 판정, 그 뒤에 따로 화물 이동이 나오고, 판정 하나 또는 종합의 레벨이 바뀔 때마다 한 줄씩 찍힌다.
  엑셀에는 판정 열(노란 머리글: 적재 무게, 예측 무게중심, h, 장력 하한, 종합 위험도, 판정별 레벨, 위험 내용),
  요약 시트에 판정 기준 · 종합 위험도별 행 수, `판정` 시트에 레벨이 바뀐 순간이 남는다.
- 한 프레임만 튀는 값에 알림이 깜빡이지 않게 판정마다 정해진 시간만큼 이어져야 올리고, 풀린 뒤에도 몇 초 붙잡아 둔다
  (`analyzer.py` `HOLD`).

### 종합 위험도 (`analysis/overall.py`)

적재 · 무게중심 · 장력 판정의 **단계(0 정상 · 1 주의 · 2 경고 · 3 위험)** 만 쓴다. 한 단계가 안전 여유를 1/4 씩 깎는다고 보고
(남은 여유: 정상 1, 주의 3/4, 경고 1/2, 위험 1/4) 판정마다 남은 여유를 곱해 다시 단계로 돌린다.

```
종합 = 4 × (1 − (1 − 적재/4) × (1 − 무게중심/4) × (1 − 장력/4)) 의 정수 부분
```

평균을 내면 위험한 판정 하나가 정상 판정들에 묻힌다 (위험 + 정상 + 정상 → 평균 1 = 주의). 곱하면 이렇게 된다.

| 판정 단계 (나쁜 순) | 종합 | |
|---|---|---|
| 하나만 정상이 아님 | 그 단계 그대로 | 위험 하나 → 위험 |
| 주의 + 주의 | 주의 | |
| 주의 + 주의 + 주의 | **경고** | 한 단계 올림 |
| 경고 + 주의 (+ 주의) | 경고 | |
| 경고 + 경고 | **위험** | 한 단계 올림 |

- 종합은 가장 나쁜 판정보다 낮아지지 않고, 겹쳐도 한 단계까지만 오른다.
- 장력은 벨트 둘 중 단계가 높은 쪽 하나만 넣는다.
- 판정마다 `HOLD` 로 거른 단계를 쓰므로 종합은 따로 거르지 않는다. 2026-10-04 화물추락에서 화물이 떨어지며 벨트를 당겨
  c2 가 0.9 → 11.4N 으로 잠깐 튀어도 장력 위험이 붙잡혀 있어 종합이 내려가지 않는다.
- 종합이 바뀌면 `changed` 에 `종합 위험도` 가 붙고(화면 한 줄 · 엑셀 `판정` 시트), 이유에 걸린 판정과 겹쳐서 올렸는지가 적힌다.

2026-10-04 실측 기록 5개를 적정 10kg 기준으로 넣어 보면 (처음 2초 뺌) **전부 위험**이었다. 정상 주행(정속 · 급출발급정거 · 우회전)도
적재가 적정 10kg 의 약 138%(경고)이고 S쪽 벨트(c2)가 하한의 약 60%(경고)라 경고 둘이 겹치기 때문이다.
지금 기준인 적정 15kg(2026-10-15)이면 정상 주행은 경고(c2 경고 하나), 거기에 `TENSION_MU` 0.45 까지 두면 주의이고
화물추락 · 장력느슨정속은 어느 쪽이든 위험이다. 적재 · 장력 기준값을 정할 때 이 결과를 같이 볼 것.

2026-10-03 실측 기록 4개를 그대로 넣어 보면 화물떨어짐은 4.4초에 화물 이동 위험, 7.1초에 무게중심 위험이 뜨고,
정속 · 급출발 · 우회전에서는 화물 이동이 안 뜬다. 그때 화물이 13~16kg 이라 급출발 · 우회전은 과적(15kg)으로 뜬다.
장력은 그때 벨트 c1 하나(3.7~10N)가 하한(16~19N)보다 한참 낮아 계속 위험이다 — 실제 마찰계수로 `TENSION_MU` 를 맞출 것.

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

## 벨트 누름 추정 실물 테스트 (`belt_test.py`)

주행 없이 짐을 쌓고 벨트를 조이기만 하면, 벨트마다 추정한 k · 누름 자리 x, y 를 출력하고 끝난다.
추정은 판정과 같은 코드(`analysis/belt_fit.py`)다.

```bash
source acquisition/env.sh
python3 belt_test.py                        # 두 벨트가 다 추정되고 5초 그대로면 끝 (Enter · Ctrl+C 로 언제든)
python3 belt_test.py --wait                 # 자동으로 안 끝낸다. Enter · Ctrl+C 로 끝
python3 belt_test.py --replay logs/<이름>.csv   # 기록 CSV 로 다시 돌린다 (PC 에서도)
```

1. 짐을 다 쌓고 **벨트는 푼 채** 손을 떼고 3초 기다린다 → 기준 (장력 0 일 때 무게 · 무게중심)
2. 앞 벨트를 조이고 손을 뗀다 (0.3초 넘게 가만히) → 앞 벨트 추정. 한 번에 8N 넘게, 되도록 15N 이상
3. 뒤 벨트를 조이고 손을 뗀다 → 뒤 벨트 추정
4. (선택) 한쪽 벨트를 반쯤 풀고 손을 뗀다 → **검증**. 장력이 준 구간은 배우지 않아서 진짜 시험이 된다
5. 5초 그대로 두면 끝난다

끝나면 최종 추정값과, 멈춘 구간마다 장력 0 기준 대비 화물 무게 · 무게중심 오차를 보정 전 / 시작값 고정 / 추정값으로 보여 준다.
'학습' 구간은 추정에 쓴 구간이라 추정값 오차가 좋게 나오고, '검증' 구간이 실제 성능이다.
기록은 `pipeline.py` 와 같은 CSV / 엑셀로 `LOG_DIR` 에 `belt_<시각>` 이름으로 남고, 결과는 `<이름>_belt.txt` 에도 적는다.

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
| `twin_core.py` | 순수 numpy 코어 (pyrealsense2 불필요). `world_rotation`, `deproject`, `HeightMapper`(2.5D), `summarize`(높이맵 요약), `voxel_downsample`, `TSDF`(3D, 비교용) |
| `pi_twin_node.py` | depth 848x480@30 → 필터 → 2cm 격자 높이맵(int16 mm) → UDP 5006 송신 (받는 곳 여러 개 가능). 컬러 + depth 원본은 `.db3` 로 녹화 |
| `twin_receiver.py` | 수신 예제. 패킷을 높이맵 numpy 배열 + 헤더로 복원 |
| `bench.py` | `out/` 의 저장 프레임으로 2.5D vs 3D 처리 시간 측정 → `out/bench_<arch>.json` |
| `pi_install_pyrealsense2.sh` | 소스 빌드한 librealsense(v2.58.4, RSUSB)를 `~/d435i/.venv` 에 설치 |

```bash
python3 twin/pi_twin_node.py 192.168.0.10 60   # 파이 -> PC 로 60초 송신 (기본 127.0.0.1, 무한)
python3 twin/pi_twin_node.py 127.0.0.1,192.168.0.10   # 같은 패킷을 여러 곳에 (이 파이의 pipeline.py + PC)
python3 twin/twin_receiver.py                  # PC/디스플레이 쪽
python3 twin/bench.py                          # 처리 비용 (PC 에서도 됨)
```

- 시작할 때 1회: 카메라 내장 IMU 로 수평 보정, 첫 프레임에서 격자 범위와 바닥 기준면을 정한다. 설치 후 카메라를 움직이면 다시 시작.
- 패킷 = `[4B 헤더길이][JSON 헤더][zlib(int16 높이맵)]`. 헤더: `seq, t, frame_ts, grid, imu(accel, tilt_change_deg), stats, sensors, calib`
- `calib` = 설치 보정값 (`ref_h, R, accel0, intr, depth_scale, max_range, stream, filters, camera`). 매 패킷 같은 값이라
  아무 패킷 하나로도 높이맵을 월드 3D 로 다시 세울 수 있다. `pipeline.py` 가 카메라 엑셀 `트윈 기준` 시트에 적는다.
- `stats` = `summarize()` 결과: `valid_pct, max_mm, mean_mm, area_m2, volume_l, cx_m, cy_m, occ_mm`. 기준면보다
  `occ_mm`(20 mm) 넘게 높은 칸을 화물로 보고 면적 · 부피 · 부피중심(높이 가중)을 낸다. `pipeline.py` 가 이걸 카메라 엑셀에 적는다.
- 녹화: 컬러(rgb8) + depth(z16) 848x480@30 원본과 카메라 IMU 를 `<시각>_d435i.db3` 한 파일로 남긴다 (기본 `acquisition/logs/`,
  `BAG_DIR` 로 바꾸고 `BAG=0` 이면 안 함). librealsense 2.58 은 ROS1 `.bag` 대신 ROS2 rosbag2(`.db3`)로만 녹화한다.
  비압축(약 61 MB/s)은 파이 SD 카드(실측 약 36 MB/s)가 못 따라가 프레임이 빠져서 무손실 압축으로 녹화한다
  (실측 약 25 MB/s, 30fps 빠짐 없음, 재생 프레임이 녹화 중 프레임과 바이트 단위로 같음). 1분에 약 1.5 GB.
  realsense-viewer 나 pyrealsense2 `cfg.enable_device_from_file()` 로 다시 돌린다.
- 트윈 노드 포트는 기본 5006 (`pipeline.py` 의 `CAM_PORT`). 5005 는 ESP32 IMU 라서 거기로 보내면 IMU 수신기가
  조용히 버리고 카메라 엑셀이 안 생긴다. 같은 장비면 `pi_twin_node.py 127.0.0.1`, 카메라가 다른 노드에 있으면
  `<파이 주소>` 만 주면 된다. PC 시각화(`twin_receiver.py`)도 5006 으로 받는다 (그래서 `pipeline.py` 와 같은 장비에선 같이 못 돈다).
- `sensors` 는 아직 자리표시자 (`read_other_sensors()` 가 `None` 을 돌려준다). 로드셀 값과 묶는 방식과 송신 시점은 미정.
- 현재는 매 프레임 송신한다. 파이(aarch64) 기준 2.5D 높이맵 424x240 → 1cm 격자가 약 6 ms (`out/bench_aarch64.json`).
