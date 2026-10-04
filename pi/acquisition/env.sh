export HX_SCK=16
# c2 = 장력 셀 추가 (2026-10-04, DT GPIO6 = 물리 31번)
export HX_DT=13,20,26,18,19,6
export HX_LABELS=c1,SW,SE,NW,NE,c2
# 풀업이면 모듈이 DRDY(LOW)를 못 낸다 (2026-09-28 실측). none 으로 둔다
export HX_PULL=none
# 순서 = N(앞)쪽 벨트, S(뒤)쪽 벨트. c1 은 NW+NE 무게로, c2 는 SW+SE 무게로 장력 하한을 정한다
export HX_TENSION=c1,c2
export COG_CORNERS=NW,NE,SW,SE
# 판 쪽 볼트 구멍 사이 거리(mm, 2026-10-04 실측). NW-NE 25cm, NW-SW 39.2cm
export BED_L=250
export BED_W=392
export HX_INVERT=SW
