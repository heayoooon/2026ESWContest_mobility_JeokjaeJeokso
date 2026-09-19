# MPU-6050 (IMU) 2개 각도보정 — pi ~/imu

## 배선 (2026-09-11 확정)
- 두 센서 공통: VCC -> 3.3V, GND -> GND, SDA -> GPIO2 (물리 3번핀) 줄, SCL -> GPIO3 (물리 5번핀) 줄
- 주소 구분: AD0 -> 3.3V 인 센서 = 0x69, AD0 비운 센서 = 0x68
- SDA/SCL 이 서로 바뀌면 버스 전체가 멈춤 (두 센서 다 안 보임). 선 옮길 땐 전원 끄고.

## 실행 (cd ~/imu)
  python3 imu_scan.py                          # 두 센서 스캔 + raw 1개씩
  python3 imu_angle.py                         # 실시간 roll / pitch 두 센서 나란히, Ctrl+C
  python3 imu_calibrate.py                     # 평평하게(칩 위로) 놓고 Enter, 센서마다 -> 가속도+자이로 오차
  python3 imu_calibrate.py --auto              # 프롬프트 없이 즉시 (평평+정지)
  python3 imu_calibrate.py --auto --gyro-only  # 자이로 오차만 (정지만 필요)
  python3 imu_calibrate.py --only bus1:0x68    # 한 센서만

## 진단 도구 (I2C 가 이상할 때)
  python3 i2c_diag.py            # 주소별 errno, SDA/SCL 유휴 레벨, 전체 스캔
  python3 i2c_stuck.py           # 통신 중 SDA/SCL 을 고속 샘플 -> 컨트롤러 멈춤 vs 선이 눌림 구분
  python3 i2c_bitbang_scan.py [--data]   # GPIO2/3 을 소프트웨어 I2C 로 직접 흔들어 스캔 (sudo 불필요)
      주의: 실행 후 핀이 GPIO 모드로 남음 -> sudo dtparam i2c_arm=off && sleep 1 && sudo dtparam i2c_arm=on
  하드웨어 컨트롤러가 "timeout waiting for bus ready" 로 멈추면 위 dtparam off/on 으로 리셋.

## 보정 방식 (2026-09-11 17:10 부터)
- 센서를 "0도로 삼고 싶은 자세"(rest pose)로 놓고 imu_calibrate.py 실행.
- 스크립트가 중력이 실린 축(up)을 자동 감지 -> 모듈이 세워져 있어도(+x) 누워 있어도(+z) 됨.
- imu_cal.json 에 실제 오차(ax..gz)와 up 축이 저장되고, 읽을 때 축을 돌려 rest pose 기준 roll/pitch 를 냄.
- 검증: python3 test_remap.py  (6개 장착 방향 x 30/20도 기울기 -> 전부 정확, ALL OK)
- 진짜 수평계처럼 쓰려면 정말 수평인 면에 놓고 보정. 브레드보드 위에서 보정하면 그 자세가 0도.

## 상태 2026-09-11 17:10
  bus1:0x68  up=+x (모듈이 브레드보드에 꽂혀 세워짐). 오차 ax 52 / ay 394 / az 483, gx -217 / gy 585 / gz -76.
  bus1:0x69  up=+z (누워 있음).               오차 ax 1817 / ay 196 / az 924, gx -1433 / gy 530 / gz -66.
  둘 다 정지 시 roll/pitch 0.0. 실시간: python3 imu_angle.py

## (이전 기록) 상태 2026-09-11 16:50
  bus1:0x69  평평하게 놓인 센서. 자이로 오차 gx -1424 / gy 528 / gz -58 저장. (자이로 지문 x축 약 -1430)
  bus1:0x68  세워진 센서.       자이로 오차 gx -217 / gy 585 / gz -76 저장.  (자이로 지문 x축 약 -215)
  가속도 오프셋은 둘 다 0 (미보정). 각 센서를 평평하게 놓은 뒤 imu_calibrate.py --auto 로 마무리.
  imu_cal.json 키는 "bus1:0x68" / "bus1:0x69".
