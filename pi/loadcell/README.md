# 로드셀 (HX711) 실험 환경

배선 (BCM 번호):  HX711 DT -> GPIO5,  SCK -> GPIO6,  VCC -> 5V,  GND -> GND
핀이 다르면:      DT_PIN=x SCK_PIN=y python3 read_raw.py

실행 순서 (cd ~/loadcell 에서, 가상환경 불필요 - 시스템 lgpio 사용):
  python3 read_raw.py     # 1. 배선 확인. 로드셀을 누르면 숫자가 크게 변해야 정상
  python3 calibrate.py    # 2. 영점 잡고, 아는 무게(예: 100) 올려서 보정 -> calibration.json 생성
  python3 weigh.py        # 3. 실시간 무게(g) 표시.  --tare 붙이면 시작할 때 다시 영점

파일:
  hx711_lib.py   HX711 드라이버 (lgpio 기반, Pi 4/5 동작)
  calibration.json  calibrate.py 가 저장하는 offset/scale

## 4개 로드셀로 바꾼 뒤 (2026-09-11)
- 스캔 결과 HX711은 GPIO5/6 한 개만 응답 -> 4개 셀이 HX711 하나에 합쳐진(콤비네이터/하프브릿지) 구성으로 판단.
- 배선이 바뀌어 옛 보정(calibration_single_cell_backup.json)은 무효.
- 새 보정: 162g 올린 값은 loaded_snapshot.json 에 저장됨. 무게 다 내린 상태에서
    python3 finish_cal.py     # 영점 측정 + 162g 값으로 scale 계산 -> calibration.json
  그 다음 162g 다시 올리고  python3 weigh.py  로 162 근처 나오는지 확인.
- 다른 무게로 처음부터 다시 하려면 python3 calibrate.py

## 보정 완료 (2026-09-11 13:10)
- 구성: 로드셀 4개 -> HX711 1개 (GPIO5/6). 저울 하나로 취급.
- calibration.json: offset 130740, scale 6.713 counts/g (162g 기준). 노이즈 sd~40 => 분해능 약 ±6g.
- 검증: 162g 올린 상태 실측 152~162g.
- 주의: 전원 넣고 처음 몇 분은 워밍업 드리프트가 커서 보정/영점은 5분 이상 지난 뒤에 할 것.
        영점이 틀어지면  python3 weigh.py --tare  (무게 없는 상태에서) 로 다시 영점.
- 도구: logger.py [초]  -> log.csv 에 raw 를 10Hz 로 기록.  scan_hx711.py -> 헤더에서 HX711 핀 자동 탐색.

## 인장형(SEN-21669 10kg bar+hook) 2개, HX711 모듈 2개 (2026-09-11)
- 셀 선색: 빨강 E+, 검정 E-, 초록 A+, 흰색 A-.  1.0 mV/V, 10kg.  한쪽 끝 고정, 고리 쪽에 매달기.
- 모듈마다 VCC->5V, GND->GND, DT/SCK -> 빈 GPIO (아무 핀). 꽂은 뒤:
    python3 scan_hx711.py     # 모든 HX711 자동 검출 -> cells.json
    python3 read_all.py       # 모든 셀 raw 동시 표시 (배선 확인)
    python3 calibrate_all.py  # 셀마다 영점 + 아는 무게 -> cal_cells.json
    python3 weigh_all.py      # 모든 셀 실시간 g  (--tare 로 재영점)
- 단일 셀용 옛 스크립트(read_raw/calibrate/weigh/pull_test)는 GPIO5/6 전용.
