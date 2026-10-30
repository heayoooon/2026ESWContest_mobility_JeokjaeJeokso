export HX_SCK=16
# c2 = 장력 셀 추가 (2026-10-04, DT GPIO6 = 물리 31번)
export HX_DT=13,20,26,18,19,6
export HX_LABELS=c1,SW,SE,NW,NE,c2
# 풀업이면 모듈이 DRDY(LOW)를 못 낸다 (2026-09-28 실측). none 으로 둔다
export HX_PULL=none
# 순서 = N(앞)쪽 벨트, S(뒤)쪽 벨트. 화물 무게를 두 벨트 위치(BELT_N_Y/BELT_S_Y)로 나눈 몫으로 장력 하한을 정한다
export HX_TENSION=c1,c2
export COG_CORNERS=NW,NE,SW,SE
# 판 쪽 볼트 구멍 사이 거리(mm, 2026-10-04 실측). NW-NE 25cm, NW-SW 39.2cm
export BED_L=250
export BED_W=392
# 벨트가 화물을 누르는 남북 위치(mm). 앞쪽 셀 줄(NW-NE 볼트 구멍 선)에서 뒤쪽으로 잰 거리.
# 실측은 앞벨트 3cm, 뒤벨트 28cm 인데, 2026-10-25 장력꽉 기록에서 앞 누름은 셀 줄 밖(-45 ~ -60mm)에 실렸다.
# 그래서 시작값을 -20 / 280 으로 두고, 정차 중 벨트를 조일 때 추정해 고쳐 나간다 (BELT_FIT=1, analysis/belt_fit.py)
export BELT_N_Y=-20
export BELT_S_Y=280
# 벨트 장력 1N 이 화물을 누르는 힘(N) 시작값. 이론 2 sin90° = 2, 10-25 기록 앞 2.1 / 뒤 1.9
export BELT_K=1.9
export BELT_FIT=1
export HX_INVERT=SW
