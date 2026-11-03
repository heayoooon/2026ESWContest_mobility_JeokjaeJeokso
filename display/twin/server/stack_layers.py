"""카메라 앞면 깊이로 쌓인 화물을 층(상자)별로 나눈다.

cargo_model 은 위에서 보이는 윗면만 맞추므로, 상자 위에 상자를 쌓으면 '윗면 크기 x 바닥까지' 큰 상자 하나가 된다.
카메라는 화물 앞면(카메라 쪽 수직면)도 본다. 그 면에서
  - 앞면이 들어가거나 나온 곳 (위 상자와 아래 상자의 앞면 위치가 다르다)
  - 아래 상자 윗면이 위 상자 옆으로 살짝 보이는 턱 (수평면)
을 찾아 층 경계로 쓴다. 아래 층의 좌우 폭은 그 층 앞면 폭, 깊이(뒤쪽 끝)는 보이지 않으므로 위 상자와 같다고 둔다.
아래 층 옆에 따로 놓인 짐(뽁뽁이로 싼 물병 등)은 '옆 짐' 으로 하나 더 만든다.

출력은 cargo_model 상자와 같은 dict (x0 x1 y0 y1 bottom top color id shape held) + kind ('box' | 'side').
매 프레임 다시 재지만, 화면이 출렁이지 않게 최근 결과의 중앙값으로 고정하고 1 cm 넘게 바뀔 때만 움직인다.
맨 끝에 ObjectMemory 가 물건마다 번호를 붙잡아, 물건 하나를 빼도 나머지 모양 · 번호는 그대로 둔다.
"""
import collections
import json
import logging
import math
import os
import time
import warnings

import numpy as np

try:
    from cargo_model import components
except ImportError:                                     # 시험용: ~/twin_view 가 sys.path 에 없을 때
    def components(mask):
        seen = np.zeros_like(mask, bool); result = []; ny, nx = mask.shape
        for y, x in zip(*np.nonzero(mask)):
            if seen[y, x]:
                continue
            stack = [(y, x)]; seen[y, x] = True; group = []
            while stack:
                yy, xx = stack.pop(); group.append((yy, xx))
                for y1, x1 in ((yy - 1, xx), (yy + 1, xx), (yy, xx - 1), (yy, xx + 1)):
                    if 0 <= y1 < ny and 0 <= x1 < nx and mask[y1, x1] and not seen[y1, x1]:
                        seen[y1, x1] = True; stack.append((y1, x1))
            result.append(np.asarray(group))
        return result

STRIDE = 2
BIN = 0.01                 # 높이 칸 (m)
MIN_LAYER = 0.07           # 층 최소 두께
Y_STEP = 0.011             # 앞면이 이만큼 들어가거나 나오면 경계
LEDGE_N = 20               # 턱(수평면) 점 수 (STRIDE 2 기준)
HIST = 25                  # 중앙값을 낼 최근 결과 수 (10 Hz 에서 2.5 초: 실제 모양 변화는 약 1.3 초 뒤에 반영)
SWITCH = .7                # 층 수는 최근 결과의 이 비율 넘게 같아야 바꾼다
MOVE = 0.01                # 이만큼 넘게 바뀌어야 화면 값을 바꾼다
FLOOR_HIST = 40
ITEMS = False              # 작은 물건 찾기: 상자 위 벨트 뭉치를 물건으로 잘못 잡아 꺼 둔다 (사용자 지적 2026-10-13)
ITEM_MARGIN = .015          # 상자 밖으로 이만큼은 상자 점으로 본다
ITEM_MIN_N = 12            # 작은 물건 최소 점 수 (STRIDE 2)
ITEM_MIN = .025            # 작은 물건 최소 폭 · 높이 (m)
ITEM_MAX = .15             # 이보다 넓으면 작은 물건이 아니다 (상자 맞춤이 틀어진 것)
ITEM_DARK = .3              # 이보다 어두우면 벨트로 본다
ITEM_HIST, ITEM_SHOW, ITEM_KEEP = 30, 6, 2
SIDE_HIST = 120            # 옆 짐 기억 (프레임)
SIDE_SHOW, SIDE_KEEP = 12, 4   # 최근 SIDE_HIST 프레임 중 이만큼 잡혀야 보이고, 이 밑으로 떨어지면 사라진다 (물병 10~60 %, 손 · 가림 1~2 %)
SIDE_MAX = (.16, .14)      # 옆 짐 최대 폭 · 높이 (넘으면 손 · 벨트 · 잘못 잡은 것)
# 물건 기억 (ObjectMemory)
HOLD, HOLD_FAST = 1.0, .7   # 새 물건은 이만큼 이어져야 생긴다 (초). 방금 무게가 늘었으면 짧게
FAST_S = 5.0                # 무게가 늘고 이 시간 안이면 HOLD_FAST
RISE = .05                 # 같은 자리에서 있던 윗면보다 이만큼 높으면 그 위에 올린 물건
TOP_TOL = .025             # 윗면이 이만큼 안이면 같은 물건
PENDING_TOP = .02          # 새 물건 후보끼리 같은 것으로 볼 윗면 차이
LOW_DROP, LOW_FRAC, LOW_HOLD = .05, .6, .2   # 자리 칸의 LOW_FRAC 이 윗면보다 LOW_DROP 넘게 낮은 채 LOW_HOLD 초 -> 없어짐
NODATA_S = 5.0             # 그 자리 깊이 값이 없고 후보도 안 나온 채 이만큼 -> 없어짐
BLIND_FRAC = .1            # 높이맵 전체에서 값 있는 칸이 이 비율 밑이면 카메라가 가려진 것 -> NODATA_S 로 지우지 않는다
GEOM_MOVE, GEOM_HOLD, GEOM_N = .02, 1.5, 20   # 모양은 GEOM_MOVE 넘게 GEOM_HOLD 초 이어질 때만 바꾼다
STRIP_TOP, STRIP_GAP = .012, .30              # 한 상자가 두 줄로 맞춰진 것: 높이 차이 · 사이 거리
# 높이맵으로 물건 따라가기: 크기는 고정, 위치만 옮긴다 (다시 맞추기를 기다리지 않는다). 2026-11-01 실제연습1 녹화 기준
TRACK_R = 12               # 찾는 범위 (칸, 1 칸 = 1 cm)
TRACK_LEVEL, TRACK_LOW = .02, .05            # 윗면 높이로 볼 차이 · 이만큼 낮으면 '빈 칸' (상자 뚜껑은 눌려 3 cm 까지 차이 난다)
TRACK_MOVE, TRACK_N, TRACK_GAIN = 2, 2, .1   # 이만큼(칸) 넘게, 이만큼(프레임) 이어지고, 점수가 넓이의 이 비율 넘게 좋아져야 옮긴다
TRACK_SEEN, TRACK_VACATED = .3, .2           # 새 자리 안에 보여야 할 윗면 비율 (뒤쪽은 카메라에 안 보인다) · 떠난 칸 중 빈 칸 비율
LEVEL_NEAR, LEVEL_SHOW = .05, .015           # 윗면 높이를 따라갈 범위 · 화면 값을 바꿀 차이 (뚜껑이 눌리거나 물건이 얹혀 몇 cm 씩 변한다)
MOVING_MAX = 5.0           # 자리가 빈 채 근처에 윗면이 있어도 이만큼 지나면 없어진 것
LOCK_S, GROW, GROW_S = 3.0, 1.15, 10.0   # 생기고 LOCK_S 뒤에는 크기를 후보로 안 바꾼다. 단 GROW_S 안에 후보가 지금 크기를 덮고 GROW 배 넘게 크면 넓힌다 (일부만 보였던 것)
MIN_BOX = .012             # 바닥 넓이가 이보다 작은 상자는 만들지 않는다 (m2, 약 11 x 11 cm). 놓는 동안 · 다시 켠 뒤 잡힌 조각
STEADY_STD, STEADY_S = 60.0, .6                # 무게 흔들림(g) 이 이보다 크면 손이 닿아 있는 것 -> 후보로 새 물건을 안 만든다 (튄 무게를 믿지 않는다)
COLD_DW = 1000.0                               # 켠 뒤 무게가 이만큼 변하면 짐을 올린 것 (주행 흔들림은 1 kg 안)
COLD_G = 1500.0                                # 켠 순간 이미 이만큼 올라와 있을 때만 켠 직후로 친다 (빈 판에서 시작하면 무게 계단이 정한다)
COLD_S, COLD_MAX, COLD_SHORT, COLD_LONG = 5.0, 15.0, .08, .14  # 켠 뒤 COLD_MAX 초 안에 상자를 처음 잡고 COLD_S 초 동안은 받침 위 병 자국만으로도 물통을 만든다 (무게 기록이 없다). 벨트(폭 5 cm)는 안 걸린다
BOTTLE_NARROW = .13                            # 평평해도 짧은 쪽이 이 안이면 물병 표면일 수 있다: 무게가 안 늘고 물통이 없을 때만 상자
# 켠 직후 병이 얹혀 상자 윗면이 대부분 가려진 경우 (2026-11-03 끝쪽 상자 + 눕힌 2 L: 윗면 .35 는 병 앞 3 cm 띠만 보이고 병 면 .43 이 가장 넓다).
# 가장 넓은 면이 병 크기로 좁고, 카메라 쪽 STEP_ROWS 칸 안에 그 폭의 STEP_PART 넘게 더 낮은 평평한 턱이 있고 (차이 STEP_RISE m), 병 증거가 있으면
# (면 안에 병을 뚫고 본 낮은 칸 · 빈 칸이 STEP_SEE 넘게, 또는 파란 뚜껑) 턱이 상자 윗면이다. STEP_HITS 프레임 이어져야 믿는다. 병은 _cold_bottle 이 자국으로 만든다
STEP_ROWS, STEP_PART, STEP_RISE, STEP_SEE, STEP_HITS = 4, .4, (.04, .16), .04, 5
BOX_MIN_SIDE = .12                             # 평평함을 못 잰 후보를 상자로 칠 최소 짧은 쪽 (m). 물병 지름 11 cm 보다 넓어야 한다
ARM_W, ARM_H = .12, .30                       # 이보다 좁고 높으면 팔로 보고 물건으로 만들지 않는다
PARTIAL_AREA, PARTIAL_H = 1.5, .06            # 아래 물건을 '일부만 본 것' 으로 볼 넓이 비 · 두께
PLATE_X = .20              # 중심이 이보다 멀면 판 밖 (m)
PLATE_Y = .66              # 판 먼 쪽 끝 (실측 .63 + 여유). 중심이 이보다 멀면 판 밖: ROI 를 y .75 까지 넓혀서 판 뒤 바닥에 선 물건도 보인다 (2026-11-03)
PLATE_X0, PLATE_X1, PLATE_Y0, PLATE_Y1 = -.245, .215, .07, .63   # 실측 판의 왼쪽 · 오른쪽 · 가까운 쪽 · 먼 쪽 끝 (카메라 좌표 m). 왼쪽이 PLATE_X 보다 멀어서 왼쪽 끝 상자가 잘려 좁게 보였다
PLATE_Z_MAX, PLATE_CELLS = .30, 150            # 후보 없이 높이맵으로 받침면을 찾을 때: 이보다 높으면 판이 아니다 · 판으로 볼 최소 칸 수
# 물통: 놓을 때 늘어난 무게가 이 안이면 물통 (g). 아래 끝은 무게 잡음 거르기. 눕힌 2 L 고정 크기 (지름, 길이 m), 방향은 세로(앞뒤) · 가로(좌우) 둘 다
BOTTLE_G = (1200.0, 2300.0)  # 위 끝: 가득 찬 2 L 는 병까지 2 kg 을 조금 넘는다. 아래 끝: 실제 2 L 는 +1.93 ~ +2.1 kg, 벨트 작업 · 주행 흔들림은 +0.35 ~ +1.09 kg (녹화 8 개)
BOTTLE_2L = (.11, .31)
# 무게중심으로 병 따라가기. 위 상자에 놓인 병은 카메라에서 10 cm 안이라 깊이가 안 잡히고 (구멍만), 먼 쪽은 시야 밖이다. 대신 로드셀은 늘 잰다:
# 병 자리 = (지금 무게 x 지금 무게중심 - 놓기 전 무게 x 놓기 전 무게중심) / 병 무게. 2026-11-04 녹화: 이 자리와 파란 뚜껑 거리가 15 ~ 16 cm (병 길이의 절반).
# 합판 가운데 = 로드셀 사각형 가운데 (카메라 좌표 m). 무게가 FOLLOW_G g 안으로 맞고 흔들림 · 가감속 없을 때만 믿고, FOLLOW_MOVE m 넘게 FOLLOW_N 프레임 벗어나면 옮긴다
CELL_CX, CELL_CY, BED_L, BED_W = -.015, .35, 250.0, 392.0
FOLLOW_G, FOLLOW_MOVE, FOLLOW_N, FOLLOW_ACC, FOLLOW_GYRO, FOLLOW_BELT = 200.0, .03, 5, .03, 3.0, 1.5   # 장력이 FOLLOW_BELT N 넘게 변하면 무게중심이 움직인 것은 벨트 때문이다: 기준을 다시 잡는다
# 뚜껑으로 방향: 병 가운데에서 뚜껑까지 이 거리 (m) 일 때 큰 축이 병 방향, 같은 답이 CAP_VOTES 번. 가로인데 보여야 할 뚜껑이 CAP_MISS_N 프레임 안 보이면 뚜껑이 먼 쪽으로 간 세로다
CAP_DIST, CAP_VOTES, CAP_MISS_N, FOV_K, FOV_Z = (.10, .21), 3, 15, 2.19, .786
AXIS_EMPTY_N = 80          # 지금 방향의 병 자리에 놓기 전 사진과 같은 칸이 이만큼 보이면 병은 거기 없다: 다른 방향으로 누워 있다
BOTTLE_OFF_S = 2.0         # 총무게가 놓기 전으로 돌아온 채 자국도 없으면 이만큼(초) 뒤 없앤다 (빼는 계단을 놓쳐도)
# 물통을 뺀 것: 후보가 BOTTLE_GONE 초 안 보이고, 최근 BOTTLE_EVENT_S 초 안에 무게가 내려간 '계단' 이 있다. 계단 = 안정된 총무게(1.5 초 흔들림 STEP_STD g 안)가
# 다른 안정된 값으로 내려갔는데 장력은 그대로. 벨트를 조이고 풀면 장력이 같이 변하고 (실제연습1: 뒤 벨트 풀 때 -4.3 kg), 손으로 누르면 안정되지 않아 걸러진다.
# load_kg 는 벨트를 조일 때 1 kg 넘게 출렁이고 늦어서 여기에 안 쓴다. 계단 크기는 물통 무게의 BOTTLE_DROP_PART 넘게, BOTTLE_DROP g 안
BOTTLE_GONE, BOTTLE_DROP, BOTTLE_DROP_PART, BOTTLE_EVENT_S, BOTTLE_WEIGHT = .8, (150.0, 2600.0), .8, 8.0, 2000.0   # 반쯤 들었다 놓는 것은 뺀 게 아니다
BOTTLE_LIFT_S = 30.0       # 물통을 들고 손이 옛 자리를 덮고 있어도 된다: 무게 계단 뒤 이 시간 안에 자국이 사라지면 뺀 것 (2026-11-03: 든 채 약 7 초)
STEP_STD, STEP_S, STEP_MIN = 40.0, .6, 100.0
BOTTLE_SAME, BOTTLE_SAME_OVER = .2, .15   # 방금 뺀 무게와 이 비율 안으로 같은 무게를 다시 놓고, 옛 물통과 이 비율 넘게 겹치면 옮겨 놓은 같은 병이다 (유령이 남으려면 병 넓이의 약 18 % 는 겹쳐야 한다)
LATE_S = 8.0               # 놓고 이만큼 뒤에 맞춰져도 무게 변화를 찾는다
BELT_CHANGE = 3.0          # 장력이 이만큼(N) 변했으면 총무게 변화에 벨트가 섞인 것 -> 그 변화는 물건 무게로 안 친다
BOTTLE_NEAR, BOTTLE_MOVE, BOTTLE_PLUS_G = .18, .08, 300.0   # 물통 조각으로 볼 거리 · 후보 중심이 이만큼 벗어나면 옮김 · 무게가 이만큼 늘면 조각 아닌 새 물건
# 모양 먼저: 높이맵에서 기억에 없는 평평한 네모를 직접 찾는다 (cargo_model 을 기다리지 않는다). 2026-11-02 실제연습1 기준 상자 윗면은 평평한 칸 .71 ~ .99, 투명 병 자리는 .21 ~ .28
FLAT_RISE, FLAT_TOL, FLAT_N = .04, .015, 100  # 기억보다 이만큼(m) 높은 칸 · 같은 윗면으로 볼 차이 · 최소 칸 수
FLAT_SIDE, FLAT_FILL = .09, .70               # 짧은 쪽 최소 (m) · 사각형 안의 평평한 칸 비율
FLAT_HOLD, FLAT_MAX = .6, 1.2                 # 같은 자리에 이만큼 있으면 상자. 무게가 흔들려도 (손이 닿아 있어도) FLAT_MAX 뒤에는 만든다
FLAT_JOIN, FLAT_JOIN_SIDE, FLAT_JOIN_G = .03, .15, 100.0   # 같은 높이 상자에 붙은 좁은 띠는 그 상자의 맞춤 오차 (무게가 늘었으면 새 물건)
CYL_SHORT, CYL_LONG, CYL_CURVE = (.05, .14), (.14, .36), 3.0   # 원통: 짧은 쪽 · 긴 쪽 범위, 짧은 쪽 단면의 곡률 (지름 11 cm 원통은 약 9 /m)
MARK_STRONG, MARK_PART = 150, .5              # 무게 계단으로 물통을 만들 때 자국 칸 수: 이만큼, 받침이 작으면 그 위에 올라가는 병 넓이의 이 비율
BOTTLE_WAIT, BOTTLE_CHAIN = 3.0, 1.5          # 무게가 늘고 모양이 보일 때까지 기다리는 시간 (초) · 손을 떼는 동안 나뉜 계단을 이어 볼 시간
HAND_H, HAND_N = .16, 15                      # 맨 위 상자보다 이만큼(m) 높은 칸이 이만큼 있으면 손 · 팔이 들어와 있다 (눕힌 물병은 11 cm)
HAND_STILL_N, HAND_STILL_Z = 50, .03           # 판보다 높은 채 높이가 이만큼(m) 안으로 이만큼(프레임) 그대로인 칸은 손이 아니다 (판 위로 지나는 케이블: 2026-11-03 늘 34 ~ 49 칸이라 물병 방향을 다시 못 봤다)
# 무게 계단 직전 사진: 무게가 안정된 동안 BEFORE_EVERY 초마다 높이맵을 모아 (BEFORE_MOVE m 넘게 변한 칸이 BEFORE_QUIET 칸 안일 때만, 손이 없을 때만)
# 계단이 오면 최신 BEFORE_DROP 장 (벌써 찍힌 손 · 병) 을 뺀 칸별 중앙값으로 얼린다. 이것보다 BEFORE_UP m 넘게 높아진 칸만 병 표면으로 친다 (판 위 전자부품 · 전선은 그대로라 안 친다)
BEFORE_EVERY, BEFORE_N, BEFORE_MIN, BEFORE_DROP, BEFORE_MOVE, BEFORE_QUIET, BEFORE_UP = .5, 16, 3, 4, .03, 40, .02
BEFORE_FAST = .2                              # 표본이 BEFORE_MIN 장 모일 때까지는 이 간격으로 모은다 (상자를 빼고 금방 물병을 놓아도 새 사진이 있게)
BEFORE_DOWN = .03                             # 놓기 전보다 이만큼(m) 낮아진 칸도 병 자국이다: 병을 통과한 깊이는 굴절로 받침보다 낮게 찍힌다 (2026-11-01 초기: 윗면 .57 인데 .48 ~ .54)
CAP_ROSE = .3                                 # 뚜껑으로 볼 파란 덩어리는 이 비율 넘게 새로 생긴 칸이어야 한다 (전자부품의 파란 칸을 빼려고)
NARROW_H = .16                                # 받침보다 이만큼(m) 넘게 높은 평평한 면은 좁아도 물병 표면이 아니다 (눕힌 2 L 는 11 cm. 높은 상자는 먼 쪽이 시야 밖이라 좁게 보인다)
# 상자 자리에 값 있는 칸이 이 비율 밑이면 깊이로 모른다 (햇빛에 빈 판은 값이 빠진다) -> DROP_RECENT 초 안의 무게 계단이 그 상자 무게의 DROP_PART 넘으면 뺀 것. 무게를 모르는 상자는 DROP_MIN g
NODATA_FRAC, DROP_RECENT, DROP_PART, DROP_MIN, BOX_DROP_S = .25, 2.0, .5, 350.0, 3.0
# 잰 물통 무게 합이 안정된 총무게보다 BOTTLE_EXCESS g 넘게 크면 그만큼 물통이 없다 (2026-11-02 06:16: 총 127 g 에 1.9 kg 물통이 남음). 상자 무게 비교 허용 비율
BOTTLE_EXCESS, BOX_DROP_TOL = 500.0, .2
# 저장한 물건 되살리기: 총무게가 RESTORE_G 안으로 같을 때만. 파일은 SAVE_EVERY 초에 한 번까지
RESTORE_G, SAVE_EVERY = 300.0, 2.0
# 상자 무게는 놓은 뒤에도 더 오른다 (손을 천천히 뗀다: 실제연습1 +3.5 kg 에 만들어지고 +6.2 kg 까지). 이 시간 안의 계단은 그 상자 무게로 친다
BOX_SETTLE_S = 4.0
# 깊이가 없는 상자를 무게로 지우는 조건: 값 없음이 이만큼(초) 이어지고, 무게가 이 상자 무게 범위 안 (배율 + 여유 g). 방금 다른 상자가 빠졌으면 그 상자 무게다
NODATA_HOLD, DROP_OVER, DROP_OVER_G = .5, 1.5, 500.0
# 물병을 옮겼다고 볼 조건: 원래 자리 중 새 자리와 안 겹치는 칸의 이 비율이 비어 있어야 한다 (벨트 · 손이 자국처럼 보일 때는 원래 자리에 병이 그대로 있다)
VACATED = .4
# 옮겼다고 볼 때는 병 표면 칸이 이만큼은 찍혀야 한다 (진짜 2 L 143 ~ 239 칸, 벨트를 조일 때 53 ~ 130 칸) · 뚜껑이 지금 자리 끝에서 이 거리 안이면 안 옮긴 것
MOVE_UP, CAP_STAY = 120, .06
# 파란 뚜껑: 최소 칸 수, 물병 끝에서 찾는 범위 (밖 m, 안 m), 옆으로 벗어날 수 있는 거리, 받침 위 높이 범위, 뚜껑 반지름, 뚜껑에 맞출 때 움직이는 최소 거리
CAP_N, CAP_OUT, CAP_IN, CAP_SIDE, CAP_Z, CAP_R, CAP_MOVE = 6, .14, .03, .08, (.02, .20), .015, .025   # 높이 조건: 상자 흰 라벨도 파랗게 읽히지만 윗면과 높이가 같다 (실제 뚜껑은 5 cm 위)
CAP_BLUE = 2.0             # 뚜껑 덩어리의 파랑 / 빨강 중앙값이 이 위여야 한다 (2026-11-03 실측: 진짜 뚜껑 2.8 ~ 11, 병에 비친 파랑 · 전자부품 1.4 ~ 1.8)
# 켠 직후 무게 기록 없이 모양으로만 만든 물통: 줄어들 무게를 모르니 모양이 LIGHT_GONE 초 안 보이면 없앤다
LIGHT_GONE = 1.5
# 한 상자를 나눠 본 것: MERGE_S 초 안에 무게가 MERGE_G 안으로 같고 (새 무게 없음) 붙어 있고 윗면 높이가 MERGE_TOP 안이면 합친다 (뚜껑 눌림 3 cm). 더 차이 나면 진짜 다른 상자다 (빈 상자는 무게가 없다)
MERGE_S, MERGE_G, MERGE_GAP, MERGE_TOP, MERGE_X = 4.0, 400.0, .04, .03, .6
BOX_BACK_S = 20.0          # 없어진 상자와 같은 자리 · 같은 높이로 이 시간 안에 다시 잡히면 놓는 중인 새 상자가 아니라 돌아온 그 상자다 (손으로 눌러 잠깐 가렸다)
STACK_SAME = .2            # 같은 자리 같은 무게로 잡힌 두 면을 한 상자로 볼 높이 차이 (m)
# 병 자국: 받침 윗면보다 높은 칸 (병 표면) + 값이 없어진 칸 (투명한 몸통 · 병 뒤 그림자). 높은 칸이 BAND_N 칸 넘으면 그것으로 방향을 본다 (그림자가 안 섞인다)
BAND_LOW, BAND_TOP, BAND_N, BAND_LONG, BAND_SHORT, BAND_RATIO, MARK_N, MARK_FLOOR_UP, MARK_RATIO, FIT_EVERY = .02, .16, 30, .15, .05, 1.25, 60, 30, 1.2, .5
# 페트병: 눕힌 병 = 길쭉한 바닥 (긴 쪽 / 짧은 쪽 >= BOTTLE_ASPECT) + 높이가 짧은 쪽과 비슷, 세운 병 = 좁고 높다
BOTTLE_D = (.045, .10)     # 지름 (m)
BOTTLE_L = (.14, .36)      # 길이 (m)
BOTTLE_ASPECT = 2.2


def _rays(intr):
    y, x = np.mgrid[:intr['height'], :intr['width']].astype(np.float32)
    return np.stack(((x - intr['ppx']) / intr['fx'], (y - intr['ppy']) / intr['fy'], np.ones_like(x)), -1)


def _overlap(a, b):
    ix = max(0.0, min(a['x1'], b['x1']) - max(a['x0'], b['x0']))
    iy = max(0.0, min(a['y1'], b['y1']) - max(a['y0'], b['y0']))
    return ix * iy / max(1e-6, (a['x1'] - a['x0']) * (a['y1'] - a['y0']))


def _overlap_min(a, b):
    """겹친 넓이 / 작은 쪽 넓이 (한쪽이 다른 쪽 안에 들어 있으면 1)."""
    ix = max(0.0, min(a['x1'], b['x1']) - max(a['x0'], b['x0']))
    iy = max(0.0, min(a['y1'], b['y1']) - max(a['y0'], b['y0']))
    area = lambda q: (q['x1'] - q['x0']) * (q['y1'] - q['y0'])
    return ix * iy / max(1e-6, min(area(a), area(b)))


def _cells(frame, o, shrink=.01):
    """높이맵에서 이 바닥 넓이 안의 값 (값 있는 칸만). 높이맵이 없으면 None."""
    h, g = frame.get('h'), frame['hdr'].get('grid')
    if h is None or not g:
        return None
    c = g['cell']
    i0, i1 = int((o['x0'] + shrink - g['x0']) / c), int(np.ceil((o['x1'] - shrink - g['x0']) / c))
    j0, j1 = int((o['y0'] + shrink - g['y0']) / c), int(np.ceil((o['y1'] - shrink - g['y0']) / c))
    v = h[max(0, j0):max(0, j1), max(0, i0):max(0, i1)]
    return v[np.isfinite(v)]


def plate_floor(frame):
    """상자 후보가 아직 없을 때의 받침면: 판 사각형 안 높이맵에서 넓은 가장 낮은 면. 못 찾으면 None."""
    h, g = frame.get('h'), frame['hdr'].get('grid')
    if h is None or not g:
        return None
    c = g['cell']
    xs = g['x0'] + (np.arange(h.shape[1]) + .5) * c
    ys = g['y0'] + (np.arange(h.shape[0]) + .5) * c
    v = np.asarray(h)[ys <= PLATE_Y - .04][:, np.abs(xs) <= PLATE_X]
    v = v[np.isfinite(v) & (v < PLATE_Z_MAX)]
    if len(v) < PLATE_CELLS:
        return None
    hist, e = np.histogram(v, np.arange(np.floor(v.min() * 100) / 100, PLATE_Z_MAX + BIN, BIN))
    good = np.nonzero(hist >= PLATE_CELLS)[0]
    if not len(good):
        return None
    return float(np.median(v[np.abs(v - (e[good[0]] + BIN / 2)) < .012]))


def merge_strips(boxes, frame):
    """벨트 · 물건이 윗면을 가려 한 상자가 두 줄로 맞춰진 것을 하나로. 높이가 같고 나란한데 그 사이 칸도 그 높이 이상이면
    같은 상자다 (사이가 낮으면 진짜로 떨어진 두 상자). 합친 상자에 얹힌 것은 바닥을 그 윗면으로 올린다."""
    out = [dict(b) for b in boxes]
    i = 0
    while i < len(out):
        a = out[i]
        j = i + 1
        while j < len(out):
            b = out[j]
            flat = a.get('kind') in (None, 'box') and b.get('kind') in (None, 'box') and 'cylinder' not in (a.get('shape'), b.get('shape'))
            if not flat or abs(a['top'] - b['top']) > STRIP_TOP:
                j += 1
                continue
            for lo0, lo1, gap0, gap1 in (('y0', 'y1', 'x0', 'x1'), ('x0', 'x1', 'y0', 'y1')):
                common = min(a[lo1], b[lo1]) - max(a[lo0], b[lo0])
                if common < .5 * min(a[lo1] - a[lo0], b[lo1] - b[lo0]):
                    continue
                p, q = (a, b) if a[gap0] <= b[gap0] else (b, a)
                gap = q[gap0] - p[gap1]
                if not 0 < gap < STRIP_GAP:
                    continue
                mid = {lo0: max(a[lo0], b[lo0]), lo1: min(a[lo1], b[lo1]), gap0: p[gap1], gap1: q[gap0]}
                cells = _cells(frame, mid, 0.0)
                if cells is not None and len(cells) >= 4 and (cells >= a['top'] - TOP_TOL).mean() >= .6:
                    for k in ('x0', 'y0', 'bottom'):
                        a[k] = min(a[k], b[k])
                    for k in ('x1', 'y1', 'top'):
                        a[k] = max(a[k], b[k])
                    out.pop(j)
                    j = i
                    break
            j += 1
        i += 1
    for b in out:                                      # 합친 상자 위에 얹힌 것: 바닥까지 내려간 값을 바로잡는다
        under = [a['top'] for a in out if a is not b and _overlap_min(a, b) > .5 and a['top'] < b['top'] - .02
                 and a.get('shape') != 'cylinder']
        if under:
            b['bottom'] = max(b['bottom'], max(under))
    return out


def bottle_kind(b):
    """상자 모양으로 페트병인지. 'bottle' (눕힘) / 'bottle_up' (세움) / None."""
    if b.get('shape') == 'cylinder':
        return 'bottle'                                  # cargo_model 이 뚜껑 색으로 찾은 병
    w, d, h = b['x1'] - b['x0'], b['y1'] - b['y0'], b['top'] - b['bottom']
    lo, hi = min(w, d), max(w, d)
    if BOTTLE_D[0] <= lo <= BOTTLE_D[1] and BOTTLE_L[0] <= hi <= BOTTLE_L[1] and hi >= lo * BOTTLE_ASPECT \
            and .6 * lo <= h <= 1.5 * lo:
        return 'bottle'
    if BOTTLE_D[0] <= lo and hi <= BOTTLE_D[1] * 1.15 and BOTTLE_L[0] <= h <= BOTTLE_L[1] and h >= hi * 1.8:
        return 'bottle_up'
    return None


GEOM = ('x0', 'x1', 'y0', 'y1', 'top')


class ObjectMemory:
    """화물을 하나하나 기억한다. 매번 맞춘 상자 목록(후보)을 그대로 내지 않는다.

    cargo_model 은 입력이 바뀔 때마다 장면 전체를 다시 맞추므로, 물건 하나만 움직여도 다른 상자 모양 · 개수까지 흔들린다.
    그래서 후보는 '새 물건이 생겼나' 를 알아내는 데만 쓰고, 있던 물건은 다음 규칙으로만 바꾼다.
      - 크기: 생길 때 정하고 LOCK_S 초 뒤 고정. 후보가 지금 크기를 다 덮으면서 더 클 때만 넓힌다 (좁아지는 일은 없다)
      - 위치: 프레임마다 높이맵에서 그 크기의 사각형이 가장 잘 맞는 곳으로 옮긴다 (_track). 다른 물건에 덮인 상자는 근거가 없으므로
        움직이지 않는다 (윗상자를 밀어도 아랫상자는 그대로)
      - 없애기: 자리 높이맵이 실제로 낮아졌을 때만 (LOW_FRAC 칸이 LOW_DROP 넘게 낮음, LOW_HOLD 초). 근처에 같은 높이 윗면이 있으면
        옮겨 가는 중으로 본다. 손이 가리면 높아지거나 값이 없으므로 없어지지 않는다
      - 새 물건: HOLD 초 이어지고 무게가 안 흔들리며 (손이 닿아 있으면 기다린다) 무게가 실제로 늘었을 때만. 장력이 변한 무게는 안 친다 (벨트).
        비교할 무게가 없으면 (막 켠 뒤) 그대로 만든다. 같은 자리에서 있던 윗면보다 RISE 넘게 높으면 그 위에 올린 것
      - 상자: 높이맵에서 기억에 없는 평평한 네모를 직접 찾아 FLAT_HOLD 초 뒤 만든다 (무게가 없어도)
      - 물통: 손을 뗀 상태로 무게가 BOTTLE_G 안으로 늘고, 네모가 아니고, 받침 위에 자국이 있을 때만 (눕힌 2 L 고정 크기). 방향 · 자리는 자국으로 정하고 손을 뗄 때마다 다시 본다.
        자국은 무게 계단 직전 사진 (before) 과 달라진 칸만 센다: 판 위 전자부품 · 전선은 그대로라 안 세어지고, 카메라에 안 보이는 무게로는 물통이 안 생긴다
        투명해서 깊이가 뚫리므로, 후보가 BOTTLE_GONE 초 안 보이고 무게도 줄었을 때만 없앤다
      - 번호는 물건마다 고정 -> 화면도 번호로 3D 상자를 붙잡아 다른 물건이 안 움직인다"""

    def __init__(self, save_path=None):
        self.save_path = save_path                     # 물건 기억 파일 (None = 저장 안 함: 시험 · 재생)
        self.saving = True                             # 녹화 재생 중에는 끈다
        self.reset()

    def _save(self, t):
        if not self.save_path or not self.saving or self.want_restore:
            return
        sig = (round((self.w or 0) / 100), tuple((o['id'], o['kind'], o.get('cap'), *(round(o[k], 2) for k in GEOM)) for o in self.objs))
        if sig == self.saved_sig or t - self.saved_t < SAVE_EVERY:
            return
        self.saved_sig, self.saved_t = sig, t
        keys = GEOM + ('bottom', 'level', 'weight', 'dw', 'axis', 'band', 'light', 'from_event', 'sup', 'cap')    # w0 은 놓는 순간에만 쓴다
        objs = []
        for o in self.objs:
            q = {k: o[k] for k in keys if o.get(k) is not None}
            q.update(id=o['id'], kind=o['kind'], shape=o['shape'], color=[float(v) for v in o['color']])
            if o.get('from_geom'):
                q['from_geom'] = {k: float(o['from_geom'][k]) for k in GEOM}
            objs.append({k: (float(v) if isinstance(v, (float, np.floating)) else v) for k, v in q.items()})
        try:
            tmp = self.save_path + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as f:
                json.dump(dict(w=self.w, next_id=self.next_id, objs=objs), f, ensure_ascii=False)
            os.replace(tmp, self.save_path)
            before = {str(o['id']): o['before'] for o in self.objs if o.get('before') is not None}
            old = self.saved_before
            if old is None or old.keys() != before.keys() or any(v is not old[k] for k, v in before.items()):   # 사진은 물통을 만들 때만 바뀐다 (주행 중 무게가 흔들려도 다시 안 쓴다)
                with open(tmp, 'wb') as f:
                    np.savez(f, **before)
                os.replace(tmp, self.save_path + '.before')
                self.saved_before = before
        except OSError as e:
            self._note('물건 기억 저장 실패: %s', e)

    def _restore(self, t):
        """무게를 읽기 시작하면 저장한 물건을 한 번 되살린다."""
        if not self.want_restore or not self.saving or self.w is None or len(self.loads) < 3:
            return
        self.want_restore = False
        try:
            with open(self.save_path, encoding='utf-8') as f:
                d = json.load(f)
        except (OSError, ValueError):
            return
        if d.get('w') is None or abs(d['w'] - self.w) > RESTORE_G:
            self._note('저장한 물건 %d 개 안 씀 (총무게 %.0f -> %.0f g)', len(d.get('objs', ())), d.get('w') or 0, self.w)
            return
        try:
            with np.load(self.save_path + '.before') as f:
                before = {k: f[k] for k in f.files}
        except Exception:                              # 없거나 깨졌으면 사진 없이 되살린다
            before = {}
        for q in d.get('objs', ()):
            o = dict(q, color=np.asarray(q['color'], float), held=False, seen=t, born=t - 100.0, low_since=None, cand=dict(q),
                     hist=collections.deque(maxlen=GEOM_N), load_ref=self.w, centers=collections.deque(maxlen=GEOM_N))
            o.setdefault('weight', BOTTLE_WEIGHT)
            if o['kind'] == 'bottle':
                o.setdefault('from_geom', {k: o[k] for k in GEOM})
                o['before'] = before.get(str(o['id']))
            self.objs.append(o)
        self.next_id = max(d.get('next_id', 1), self.next_id)
        self._note('저장한 물건 %d 개 되살림 (총무게 %.0f g)', len(self.objs), self.w)

    def reset(self):
        self.want_restore = bool(self.save_path)
        self.saved_sig, self.saved_t, self.saved_before = None, -1e9, None
        self.objs = []
        self.pending = []
        self.next_id = 1
        self.loads = collections.deque(maxlen=300)    # (시각, 총무게 g) 벨트를 조이고 풀 때도 변하므로 장력과 같이 본다
        self.belts = collections.deque(maxlen=300)    # (시각, 앞 장력, 뒤 장력)
        self.w = None                                 # 지금 총무게 (1 초 중앙값)
        self.level = None                             # 마지막 안정된 (총무게, 앞 장력, 뒤 장력)
        self.drop_event = None                        # 마지막 내려간 계단 (시각, g)
        self.box_gone_t, self.box_gone_dw = -1e9, None   # 마지막으로 없어진 상자의 시각 · 놓을 때 무게 (그 무게 계단을 물통으로 치지 않으려고)
        self.box_gone_geom = None                     # 마지막으로 없어진 상자 자리
        self.up_event = None                          # 아직 처리 안 한 올라간 계단 (시각, g)
        self.up_t = -1e9                              # 마지막으로 올라간 시각
        self.h_buf = collections.deque(maxlen=5)      # 최근 높이맵 (중앙값으로 잡음을 줄인다)
        self.hm, self.grid, self.floor = None, None, None   # 이번 프레임 높이맵 · 받침면
        self.col = None                               # 이번 프레임 색 (뚜껑 찾기)
        self.bg = None                                # 물통이 없을 때의 높이맵 (원래 값이 없던 칸을 병 자국으로 안 치려고)
        self.bg0 = None                               # 켠 뒤 처음 안정된 높이맵 (표본이 모이기 전에 무게가 오르면 놓기 전 사진으로 쓴다)
        self.ring = collections.deque(maxlen=BEFORE_N)   # 무게가 안정된 동안 모은 높이맵 표본
        self.ring_t, self.ring_prev = -1e9, None
        self.before = None                            # 마지막 무게 계단 직전 사진 (표본 중앙값)
        self.before_stale = False                     # 사진을 찍은 뒤 상자가 없어졌다
        self.high_n, self.high_z = None, None         # 칸마다 판보다 높은 채 높이가 그대로인 프레임 수 · 그 높이
        self.cold_w = None
        self.t_start, self.t_box, self.cold_done = None, None, False   # 켠 시각 · 상자를 처음 잡은 시각 (켠 직후에는 무게 기록이 없다)
        self.step_hits = 0                            # 병이 얹힌 상자 모양이 이어진 프레임 수
        self.cogs = collections.deque(maxlen=12)      # 최근 (시각, 무게중심 x, y) 가감속이 없을 때만
        self.rest = None                              # 무게가 안정된 마지막 (총무게, 무게중심 x, y): 놓기 전 값
        self.up_chain = None                          # (시각, 시작 무게) 손을 떼는 동안 나뉜 계단을 이어 본다
        self.hand_cells = 0                           # 이번 프레임에 기억보다 훨씬 높은 칸 수 (손 · 팔)
        self.flat_now = []                            # 이번 프레임에 찾은 기억에 없는 네모 · 원통

    @staticmethod
    def _note(fmt, *args):
        logging.getLogger('twin.err').info(fmt, *args)

    def _new(self, geom, c, bottom, t, load, kind):
        o = dict(geom, bottom=bottom, id=self.next_id, kind=kind, shape='bottle' if kind == 'bottle' else (c.get('shape') or 'box'),
                 color=np.asarray(c.get('color', (.6, .5, .4)), float), held=False, seen=t, born=t, low_since=None,
                 hist=collections.deque(maxlen=GEOM_N), load_ref=self.w, cand=c, centers=collections.deque(maxlen=GEOM_N),
                 weight=BOTTLE_WEIGHT)
        self.next_id += 1
        self.objs.append(o)
        return o

    def _see(self, o, c, t, load):
        o['seen'] = t
        o['cand'] = c
        o['load_ref'] = self.w
        o['color'] = o['color'] * .95 + np.asarray(c.get('color', o['color']), float) * .05
        area = lambda q: (q['x1'] - q['x0']) * (q['y1'] - q['y0'])
        if area(c) >= .7 * area(o):
            o['hist'].append((t, {k: c[k] for k in GEOM}))

    def _covered(self, o):
        """다른 상자가 위에 얹혀 있나."""
        return any(q is not o and q['kind'] != 'bottle' and q['top'] > o['top'] + .02 and _overlap_min(q, o) > .15 for q in self.objs)

    def _settle(self, o, t):
        """후보로 모양 고치기. 생기고 LOCK_S 초 안에는 최근 큰 후보의 중앙값으로, 그 뒤에는 윗면 높이와 넓히기만."""
        recent = [g for ts, g in o['hist'] if t - ts <= GEOM_HOLD]
        if o['kind'] == 'bottle' or len(recent) < 5 or t - o['hist'][0][0] < GEOM_HOLD * .8:
            return
        med = {k: float(np.median([g[k] for g in recent])) for k in GEOM}
        if not any(abs(med[k] - o[k]) > GEOM_MOVE for k in GEOM):
            return
        area = lambda q: (q['x1'] - q['x0']) * (q['y1'] - q['y0'])
        if t - o['born'] <= LOCK_S:
            o.update(med)
            o['level'] = med['top']
        elif t - o['born'] <= GROW_S and area(med) > GROW * area(o) and not self._covered(o) and all(
                med[a] <= o[a] + GEOM_MOVE for a in ('x0', 'y0')) and all(med[a] >= o[a] - GEOM_MOVE for a in ('x1', 'y1')):
            self._note('물건 #%d 넓힘 (%.0f x %.0f -> %.0f x %.0f cm)', o['id'], (o['x1'] - o['x0']) * 100, (o['y1'] - o['y0']) * 100,
                       (med['x1'] - med['x0']) * 100, (med['y1'] - med['y0']) * 100)
            o.update({k: med[k] for k in ('x0', 'x1', 'y0', 'y1')})
        o['hist'].clear()

    def _level(self, o, hm, g):
        """윗면 높이를 높이맵으로 직접 따라간다 (후보가 안 나와도). 자리 안에서 지금 높이 ±LEVEL_NEAR 칸의 중앙값으로 천천히,
        화면 값(top)은 LEVEL_SHOW 넘게 벌어질 때만 바꾼다."""
        c = g['cell']
        i0, i1 = max(0, int(round((o['x0'] - g['x0']) / c))), max(0, int(round((o['x1'] - g['x0']) / c)))
        j0, j1 = max(0, int(round((o['y0'] - g['y0']) / c))), max(0, int(round((o['y1'] - g['y0']) / c)))
        v = hm[j0:j1, i0:i1]
        lev = o.get('level', o['top'])
        near = v[np.abs(v - lev) <= LEVEL_NEAR]
        if len(near) >= max(6, .15 * v.size):
            lev = lev * .9 + float(np.median(near)) * .1
        o['level'] = lev
        if abs(lev - o['top']) > LEVEL_SHOW:
            o['top'] = lev

    def _track(self, o, hm, g):
        """높이맵에서 물건 자리를 찾아 옮긴다. 크기는 그대로. 점수 = 사각형 안의 윗면 높이 칸 - 2 x 빈 칸 (값 없는 칸 · 더 높은 칸은 모름).

        실제로 옮겼는지 양쪽을 다 본다. 떠난 칸에 윗면이 거의 없어야 하고, 새로 덮는 칸에 윗면이 보이거나 (뒤쪽은 카메라에 안 보여
        값이 없을 수 있다) 떠난 칸이 실제로 낮아야 한다. 상자 뚜껑은 3 cm 까지 눌려 있어 5 cm 넘게 낮아야 '빈 칸' 이다."""
        if o['kind'] == 'bottle' or self._covered(o):
            return
        c, T = g['cell'], o.get('level', o['top'])
        lvl, low = np.abs(hm - T) <= TRACK_LEVEL, hm < T - TRACK_LOW
        for q in self.objs:                            # 옆에 같은 높이로 놓인 다른 물건 칸은 빼서 그쪽으로 옮겨 가지 않게
            if q is not o and q['kind'] != 'bottle' and abs(q['top'] - T) <= TRACK_LOW:
                a0, a1 = max(0, int((q['x0'] - g['x0']) / c)), max(0, int(np.ceil((q['x1'] - g['x0']) / c)))
                b0, b1 = max(0, int((q['y0'] - g['y0']) / c)), max(0, int(np.ceil((q['y1'] - g['y0']) / c)))
                lvl[b0:b1, a0:a1] = False
        ny, nx = hm.shape
        w, h = max(1, int(round((o['x1'] - o['x0']) / c))), max(1, int(round((o['y1'] - o['y0']) / c)))
        i0, j0 = int(round((o['x0'] - g['x0']) / c)), int(round((o['y0'] - g['y0']) / c))
        R = TRACK_R
        pad = R + max(w, h) + 2
        I = np.zeros((ny + 2 * pad + 1, nx + 2 * pad + 1), np.int32)
        L = np.zeros((ny + 2 * pad + 1, nx + 2 * pad + 1), np.int32)
        I[pad + 1:pad + 1 + ny, pad + 1:pad + 1 + nx] = lvl.astype(np.int32) - 2 * low.astype(np.int32)
        L[pad + 1:pad + 1 + ny, pad + 1:pad + 1 + nx] = lvl
        I, L = I.cumsum(0).cumsum(1), L.cumsum(0).cumsum(1)
        J0, I0 = j0 + pad - R, i0 + pad - R
        if J0 < 0 or I0 < 0 or J0 + 2 * R + h >= I.shape[0] or I0 + 2 * R + w >= I.shape[1]:
            return

        def rect(A):
            top, left = A[J0:J0 + 2 * R + 1, I0:I0 + 2 * R + 1], A[J0:J0 + 2 * R + 1, I0 + w:I0 + w + 2 * R + 1]
            bot, br = A[J0 + h:J0 + h + 2 * R + 1, I0:I0 + 2 * R + 1], A[J0 + h:J0 + h + 2 * R + 1, I0 + w:I0 + w + 2 * R + 1]
            return br - bot - left + top

        score, inside = rect(I), rect(L)
        cur = float(score[R, R])
        dj, di = np.mgrid[-R:R + 1, -R:R + 1]
        best = np.unravel_index(np.argmax(score - .05 * np.maximum(np.abs(dj), np.abs(di))), score.shape)
        bj, bi = int(dj[best]), int(di[best])
        area = w * h
        ok = max(abs(bj), abs(bi)) >= TRACK_MOVE and inside[best] >= max(6, TRACK_SEEN * area) and score[best] >= cur + max(4, TRACK_GAIN * area)
        if ok:
            old, new = np.zeros((ny, nx), bool), np.zeros((ny, nx), bool)
            old[max(0, j0):max(0, j0 + h), max(0, i0):max(0, i0 + w)] = True
            new[max(0, j0 + bj):max(0, j0 + bj + h), max(0, i0 + bi):max(0, i0 + bi + w)] = True
            n_lvl, n_low, n_all = int(lvl[new & ~old].sum()), int(low[new & ~old].sum()), int((new & ~old).sum())
            v_lvl, v_low, v_all = int(lvl[old & ~new].sum()), int(low[old & ~new].sum()), int((old & ~new).sum())
            seen_there = n_lvl >= 3 and n_lvl >= .5 * n_all
            left_empty = v_all > 0 and v_low >= TRACK_VACATED * v_all
            ok = v_lvl <= .25 * max(1, v_all) and n_low <= .3 * max(1, n_lvl + n_low) and (seen_there or left_empty) and o['y1'] + bj * c <= PLATE_Y + .03
        if not ok:
            o['mv'] = None
            return
        prev = o.get('mv')
        n = prev[2] + 1 if prev and abs(prev[0] - bj) <= 1 and abs(prev[1] - bi) <= 1 else 1
        o['mv'] = (bj, bi, n)
        if n >= TRACK_N:
            o['x0'] += bi * c; o['x1'] += bi * c
            o['y0'] += bj * c; o['y1'] += bj * c
            o['mv'] = None
            o['hist'].clear()
            self._note('물건 #%d 옮김 (x %+.0f, y %+.0f cm)', o['id'], bi * c * 100, bj * c * 100)
            for q in self.objs:
                q.pop('rest', None)                    # 상자가 움직이면 무게중심 변화가 병 때문이 아니다: 지금 자리를 기준으로 다시 잡는다

    def _moving_away(self, o, hm, g):
        """자리는 비었는데 바로 근처에 같은 높이 윗면이 넓게 있다 = 없어진 게 아니라 옮겨 가는 중."""
        if hm is None:
            return False
        c = g['cell']
        i0, i1 = max(0, int((o['x0'] - g['x0']) / c) - TRACK_R), max(0, int(np.ceil((o['x1'] - g['x0']) / c)) + TRACK_R)
        j0, j1 = max(0, int((o['y0'] - g['y0']) / c) - TRACK_R), max(0, int(np.ceil((o['y1'] - g['y0']) / c)) + TRACK_R)
        lvl = np.abs(hm - o.get('level', o['top'])) <= TRACK_LEVEL
        for q in self.objs:                            # 옆에 같은 높이로 놓인 다른 물건은 빼고
            if q is not o and q['kind'] != 'bottle' and abs(q['top'] - o['top']) <= TRACK_LOW:
                a0, a1 = max(0, int((q['x0'] - g['x0']) / c)), max(0, int(np.ceil((q['x1'] - g['x0']) / c)))
                b0, b1 = max(0, int((q['y0'] - g['y0']) / c)), max(0, int(np.ceil((q['y1'] - g['y0']) / c)))
                lvl[b0:b1, a0:a1] = False
        lvl[g['y0'] + (np.arange(hm.shape[0]) + .5) * c > PLATE_Y] = False   # 판 뒤에 선 같은 높이 물건으로 옮겨 가는 것이 아니다
        near = int(lvl[j0:j1, i0:i1].sum())
        return near >= .4 * ((o['x1'] - o['x0']) * (o['y1'] - o['y0'])) / (c * c)

    @staticmethod
    def _blind(frame):
        """카메라가 가려졌나 (높이맵 전체에 값 있는 칸이 거의 없다)."""
        h = frame.get('h')
        return h is None or np.isfinite(h).mean() < BLIND_FRAC

    def _gone(self, o, frame, t, load, hm):
        if o['kind'] == 'bottle' and o.get('light'):
            if o.get('sup') is not None and not any(q['id'] == o['sup'] for q in self.objs):
                return True                            # 받침 상자가 없어졌다: 옮긴 상자 그림자의 빈 칸을 병 자국으로 세지 않는다
            if self._mark_here(o) >= MARK_N:
                o['seen'] = t
            return t - o['seen'] > LIGHT_GONE
        if o['kind'] == 'bottle':
            if t - o['seen'] < BOTTLE_GONE or load is None:   # 무게를 못 읽는 동안은 뺐는지 모른다 (값이 돌아오면 계단으로 본다)
                return False
            rest = o.get('rest')
            if rest is not None and self._steady(t) and self.w is not None and abs(self.w - rest[0]) <= .25 * o['weight'] and (self.box_gone_t < o['born'] or (self.hand_cells < HAND_N and self._mark_here(o, nolow=True) < MARK_N)):
                o['off_t'] = o.get('off_t') or t       # 빼는 계단을 놓쳐도 (손으로 눌렀다 뗐다) 총무게가 놓기 전으로 돌아오고 자국도 없으면 뺀 것이다
                if t - o['off_t'] >= BOTTLE_OFF_S:
                    self._note('물통 #%d 뺀 것으로 봄 (총무게가 놓기 전으로 돌아옴)', o['id'])
                    return True
            else:
                o['off_t'] = None
            ev = self.drop_event
            if ev is None or t - ev[0] > BOTTLE_LIFT_S or ev[0] < o['born'] or not max(BOTTLE_DROP[0], BOTTLE_DROP_PART * o['weight']) <= ev[1] <= BOTTLE_DROP[1]:
                return False
            if self._mark_here(o, nolow=True) >= MARK_N:
                return False                           # 그 자리에 자국이 그대로다: 다른 물건을 뺀 무게다 (자국이 사라지면 그때 뺀 것)
            others = [q for q in self.objs if q is not o and q['kind'] == 'bottle' and t - q['seen'] >= BOTTLE_GONE]
            if others and self._mark_here(o) > min(self._mark_here(q) for q in others):
                return False                           # 다른 물통 자리가 더 비었다: 그쪽을 뺀 것
            self.drop_event = None                     # 계단 하나에 물통 하나
            self._note('물통 #%d 뺀 것으로 봄 (무게 -%.0f g, 장력 그대로)', o['id'], ev[1])
            return True
        cells = _cells(frame, o)
        c = frame['hdr']['grid']['cell'] if frame['hdr'].get('grid') else None
        if cells is None or c is None or len(cells) < max(6, NODATA_FRAC * (o['x1'] - o['x0']) * (o['y1'] - o['y0']) / (c * c)):
            if o.get('nodata_since') is None:
                o['nodata_since'] = t
            ev = self.drop_event
            if ev is not None and t - ev[0] <= DROP_RECENT and t - o['nodata_since'] >= NODATA_HOLD and not self._covered(o) \
                    and t - self.box_gone_t > BOX_DROP_S and self.hand_cells < HAND_N and self._drop_fits(o, ev[1]):
                self.drop_event = None                 # 깊이가 없어도 이 상자 무게만큼 줄었다
                self._note('상자 #%d 뺀 것으로 봄 (깊이 값 없음, 무게 -%.0f g)', o['id'], ev[1])
                return True
            if cells is None or len(cells) < 6:
                return t - o['seen'] > NODATA_S and not self._blind(frame)   # 값이 없는 채로 오래 (ROI 밖). 카메라 전체가 가려졌으면 모른다
        else:
            o['nodata_since'] = None
        if (cells < o.get('level', o['top']) - LOW_DROP).mean() < LOW_FRAC:
            o['low_since'] = None
            return False
        if o['low_since'] is None:
            o['low_since'] = t
        if t - o['low_since'] < LOW_HOLD:
            return False
        return t - o['low_since'] > MOVING_MAX or not self._moving_away(o, hm, frame['hdr']['grid'])

    def _height_median(self, frame):
        """최근 5 프레임 높이맵 칸별 중앙값 (값 없는 칸은 빼고)."""
        h = frame.get('h')
        if h is None or not frame['hdr'].get('grid'):
            return None
        if self.h_buf and self.h_buf[-1].shape != h.shape:
            self.h_buf.clear()
        self.h_buf.append(np.asarray(h, np.float32))
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            return np.nanmedian(np.stack(self.h_buf), 0)

    def _load(self, t0, t1):
        v = [g for ts, g in self.loads if t0 <= ts <= t1]
        return float(np.median(v)) if len(v) >= 3 else None

    def _flat_cand(self, r):
        """이 자리가 평평한 네모인가 (후보 검사용)."""
        if (r['x1'] - r['x0']) * (r['y1'] - r['y0']) < MIN_BOX or min(r['x1'] - r['x0'], r['y1'] - r['y0']) < FLAT_SIDE or self.hm is None:
            return False
        c, g = self.grid['cell'], self.grid
        i0, i1 = max(0, int(round((r['x0'] - g['x0']) / c))), max(0, int(round((r['x1'] - g['x0']) / c)))
        j0, j1 = max(0, int(round((r['y0'] - g['y0']) / c))), max(0, int(round((r['y1'] - g['y0']) / c)))
        v = self.hm[j0:j1, i0:i1]
        fin = v[np.isfinite(v)]
        return v.size > 0 and fin.size >= FLAT_FILL * v.size and (np.abs(fin - np.median(fin)) <= FLAT_TOL).sum() >= FLAT_FILL * v.size

    def _find_new(self, hm, frame, floor):
        """높이맵에서 기억이 설명 못 하는 물건을 직접 찾는다: 평평한 네모 = 상자, 짧은 쪽 단면이 볼록한 길쭉한 것 = 원통."""
        g = frame['hdr'].get('grid')
        self.hand_cells = 0
        if hm is None or not g or floor is None:
            return []
        c = g['cell']
        exp = np.full(hm.shape, floor, np.float32)
        for o in self.objs:
            i0, i1 = max(0, int((o['x0'] - g['x0']) / c) - 1), max(0, int(np.ceil((o['x1'] - g['x0']) / c)) + 1)
            j0, j1 = max(0, int((o['y0'] - g['y0']) / c) - 1), max(0, int(np.ceil((o['y1'] - g['y0']) / c)) + 1)
            exp[j0:j1, i0:i1] = np.maximum(exp[j0:j1, i0:i1], o['top'])
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            mask = hm - exp > FLAT_RISE
        xs = g['x0'] + (np.arange(hm.shape[1]) + .5) * c
        yy = g['y0'] + (np.arange(hm.shape[0]) + .5) * c
        mask[:, (xs < PLATE_X0 - .01) | (xs > PLATE_X + .02)] = False
        mask[yy > PLATE_Y] = False                     # 판 뒤 물건과 한 덩어리로 묶이면 판 끝 상자까지 판 밖으로 버려진다
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            on_plate = (np.abs(xs) <= PLATE_X)[None, :] & (yy <= PLATE_Y)[:, None]
            high = (hm - max([o['top'] for o in self.objs if o['kind'] != 'bottle'] + [floor]) > HAND_H) & on_plate
            if self.high_n is None or self.high_n.shape != high.shape:
                self.high_n, self.high_z = np.zeros(high.shape, np.int32), hm.copy()
            still = (hm - floor > HAND_H) & on_plate & (np.abs(hm - self.high_z) <= HAND_STILL_Z)   # 판 기준 (상자 윗면 기준이면 상자를 뺄 때 케이블이 새 손이 된다). 상자 위로 들어온 손은 높이가 달라 처음부터 다시 센다
        self.high_z = np.where(still, self.high_z, hm)
        self.high_n = (self.high_n + 1) * still
        self.hand_cells = int((high & (self.high_n < HAND_STILL_N)).sum())
        if mask.sum() < FLAT_N:
            return []
        col = frame.get('col')
        out = []
        for grp in components(mask):
            if len(grp) < FLAT_N:
                continue
            ys, xi = grp[:, 0], grp[:, 1]
            v = hm[ys, xi]
            w, d = (xi.max() - xi.min() + 1) * c, (ys.max() - ys.min() + 1) * c
            short, long_ = min(w, d), max(w, d)
            if CYL_SHORT[0] <= short <= CYL_SHORT[1] and CYL_LONG[0] <= long_ <= CYL_LONG[1]:
                u = (xi if w <= d else ys) * c
                if len(np.unique(u)) >= 4 and np.polyfit(u, v, 2)[0] < -CYL_CURVE:
                    continue                           # 볼록한 길쭉한 것은 상자가 아니다 (물통은 무게 계단이 만든다)
            vals, n = np.unique(np.round(v / FLAT_TOL), return_counts=True)   # 손 · 벨트가 붙어 있어도 가장 넓은 높이의 칸만
            top = float(vals[n.argmax()]) * FLAT_TOL
            sub = np.zeros(hm.shape, bool)
            near = np.abs(v - top) <= FLAT_TOL
            sub[ys[near], xi[near]] = True
            parts = components(sub)
            if not parts:
                continue
            keep = max(parts, key=len)
            if len(keep) < FLAT_N:
                continue
            ys, xi = keep[:, 0], keep[:, 1]
            i0, i1 = int(np.percentile(xi, 2)), int(np.percentile(xi, 98)) + 1
            j0, j1 = int(np.percentile(ys, 2)), int(np.percentile(ys, 98)) + 1
            w, d = (i1 - i0) * c, (j1 - j0) * c
            if min(w, d) < FLAT_SIDE or w * d < MIN_BOX or len(keep) < FLAT_FILL * (i1 - i0) * (j1 - j0):
                continue
            nb = dict(x0=g['x0'] + i0 * c, x1=g['x0'] + i1 * c, y0=g['y0'] + j0 * c, y1=g['y0'] + j1 * c,
                      top=float(np.median(hm[ys, xi])), bottom=float(np.median(exp[ys, xi])), kind='box', shape='box',
                      direct=True, held=False)
            if (nb['y0'] + nb['y1']) / 2 > PLATE_Y:
                continue
            nb['narrow'] = min(w, d) <= BOTTLE_NARROW and nb['top'] - nb['bottom'] <= NARROW_H
            nb['color'] = np.clip(np.median(col[ys, xi], axis=0) / 255., 0, 1) if col is not None and np.shape(col)[:2] == hm.shape else (.6, .5, .4)
            if self._is_strip(nb):
                continue
            out.append(nb)
        return out

    def _is_strip(self, b):
        """같은 높이 상자에 붙은 좁은 띠 = 그 상자를 작게 맞춘 것. 무게가 늘었으면 옆에 새로 놓은 것이다."""
        if min(b['x1'] - b['x0'], b['y1'] - b['y0']) >= FLAT_JOIN_SIDE or (self.level is not None and self.w is not None and self.w - self.level[0] >= FLAT_JOIN_G):
            return False
        for o in self.objs:
            gap = max(o['x0'] - b['x1'], b['x0'] - o['x1'], o['y0'] - b['y1'], b['y0'] - o['y1'])
            if o['kind'] != 'bottle' and gap <= FLAT_JOIN and abs(o['top'] - b['top']) <= TOP_TOL:
                return True
        return False

    def _box_settled(self, ev):
        """물통이 아닌 무게 계단: 방금 놓은 상자에서 손을 마저 뗀 것이다. 그 상자 무게를 고친다 (나중에 뺄 때 비교한다)."""
        fresh = [o for o in self.objs if o['kind'] != 'bottle' and o.get('dw') is not None and o.get('w0') is not None and ev[0] - o['born'] <= BOX_SETTLE_S]
        if fresh and self.level is not None:           # 상자를 잇달아 놓았으면 마지막 상자 몫이다
            o = max(fresh, key=lambda q: q['born'])
            o['dw'] = max(o['dw'], self.level[0] - o['w0'])

    def _bottle_from_weight(self, t, load, hm, frame, floor):
        """손을 뗀 상태로 무게가 물통만큼 늘었고, 네모가 아니고, 받침 위에 자국이 있으면 물통. 모양이 나올 때까지 BOTTLE_WAIT 초 기다린다."""
        ev = self.up_event
        if ev is None:
            return
        if not BOTTLE_G[0] <= ev[1] < BOTTLE_G[1]:
            self.up_event = None
            self._box_settled(ev)
            return
        age = t - ev[0]
        for o in self.objs:                            # 켠 직후 자국으로 방금 만든 물통: 이 계단이 그 물통의 무게다
            if o['kind'] == 'bottle' and o.get('light') and t - o['born'] <= BOTTLE_WAIT:
                o['weight'], o['light'] = ev[1], False
                self.up_event = None
                return
        for o in self.objs:                            # 이 무게가 방금 만든 상자의 무게다
            if o['kind'] != 'bottle' and t - o['born'] <= BOTTLE_WAIT and o.get('dw') is not None and abs(o['dw'] - ev[1]) <= .35 * ev[1]:
                self.up_event = None
                return
        wide = [b for b in self.flat_now if not b.get('narrow')]
        if age > BOTTLE_EVENT_S:                       # 오래된 계단은 버린다
            self.up_event = None
            return
        if not self._steady(t):                        # 무게가 움직이는 동안은 판단하지 않는다 (새 계단이 오면 그것으로 바뀐다)
            return
        if (wide or self.hand_cells >= HAND_N) and age < BOTTLE_WAIT:   # 네모가 보이거나 아직 손이 있다
            return
        mark = None if wide else self._bottle_mark(floor, self.before, plate_needs_before=True)
        if mark is not None and mark['n_up'] >= FLAT_FILL * mark['n'] and mark['flat'] >= FLAT_FILL and min(mark['mark_w'], mark['mark_d']) > BOX_MIN_SIDE:
            mark = None                                # 병보다 넓고 평평한 네모: 내려놓는 중인 상자다 (2026-11-04: 21 x 14 cm 상자를 잠깐 물통으로 그렸다)
        if mark is not None and mark['n'] < mark['need']:
            mark = None                                # 자국이 병 크기만큼은 돼야 한다 (진짜 2 L 는 217 ~ 326 칸. 상자를 놓은 손을 뗄 때 +2112 g 인데 63 칸, 벨트 조일 때 82 칸)
        if mark is None:
            if age < BOTTLE_WAIT:
                return
            self.up_event = None
            if self._bottle_from_cap(ev, t, load, floor):
                return
            self._note('물통 안 만듦 (무게 %+.0f g 인데 받침 위에 자국이 없음)', ev[1])
            self._box_settled(ev)
            return
        self.up_event = None
        rect = {k: mark[k] for k in ('x0', 'x1', 'y0', 'y1', 'axis', 'band', 'sup')}
        base = mark['base']
        drop = self.drop_event
        if drop is not None and t - drop[0] <= BOTTLE_LIFT_S and abs(drop[1] - ev[1]) <= BOTTLE_SAME * ev[1]:   # 방금 뺀 무게를 다시 놓았다: 겹치는 옛 물통은 옮겨 놓은 같은 병이다 (새 병이 옛 자리를 덮어 자국으로는 못 지운다)
            for q in sorted([q for q in self.objs if q['kind'] == 'bottle' and q.get('sup') == rect['sup'] and _overlap_min(q, rect) > BOTTLE_SAME_OVER], key=lambda q: _overlap_min(q, rect))[-1:]:
                self.objs.remove(q)
                self.drop_event = None
                self._note('물통 #%d 옮겨 놓음 (같은 무게 %.0f g)', q['id'], drop[1])
                if q.get('before') is not None:
                    self.before = q['before']
        fake = dict(color=(.7, .8, .9), shape='bottle')
        o = self._new(dict(rect, top=base + BOTTLE_2L[0]), fake, base, t, load, 'bottle')
        o['from_geom'] = dict(rect, top=o['top'])
        o['from_event'] = True
        o['weight'] = ev[1]
        o['before'] = self.before                      # 놓기 전 사진: 다시 볼 때도 이 물통이 만든 변화만 본다
        now = self._load(t - STEP_S, t) or self.w      # 놓기 전 무게 · 무게중심: 병 자리를 무게로 따라간다. 방금 든 병을 바로 놓아 안정된 값이 옛날이면 안 쓴다 (지금 자리로 다시 잡는다)
        o['rest'] = self.rest if self.rest is not None and now is not None and abs(self.rest[0] + ev[1] - now) <= FOLLOW_G else None
        self._forget_moved(o, ev, now)
        o['cap'] = mark['cap']
        self.up_chain = None                           # 이 무게는 이 물통 몫이다
        self._note('물건 #%d 생김: 물통 %s (무게 %+.0f g, 자국 %d 칸)', o['id'], '가로' if o['axis'] == 'x' else '세로', ev[1], mark['n'])

    def _forget_moved(self, o, ev, now):
        """이 계단 직전 무게에 다른 물통 무게가 안 들어 있으면 이미 판에서 빠진 옛 물통이다 (옮겨 놓은 같은 병)."""
        for q in [q for q in self.objs if q is not o and q['kind'] == 'bottle' and q.get('rest') is not None]:
            if now is not None and abs(now - ev[1] - q['rest'][0]) <= FOLLOW_G:
                self.objs.remove(q)
                self.drop_event = None
                self._note('물통 #%d 옮겨 놓음 (놓기 전 무게에 없음)', q['id'])

    def _bottle_from_cap(self, ev, t, load, floor):
        """자국이 안 잡혀도 (가까워서 깊이가 뚫림) 무게중심으로 낸 자리에서 병 길이의 절반 거리에 진짜 뚜껑이 보이면 물통이다."""
        cog, now, rest = self._cog_now(t), self._load(t - STEP_S, t), self.rest
        if cog is None or now is None or rest is None or abs(rest[0] + ev[1] - now) > FOLLOW_G or now - rest[0] <= 0:
            return False
        m = now - rest[0]
        cx = CELL_CX - ((now * cog[0] - rest[0] * rest[1]) / m - BED_L / 2) / 1000
        cy = CELL_CY + ((now * cog[1] - rest[0] * rest[2]) / m - BED_W / 2) / 1000
        sup = self._support(cx, cy)
        base = sup.get('level', sup['top']) if sup is not None else floor
        if base is None:
            return False
        for x, y in self._find_caps(base):
            dx, dy = x - cx, y - cy
            if CAP_DIST[0] <= math.hypot(dx, dy) <= CAP_DIST[1]:
                axis = 'x' if abs(dx) > abs(dy) else 'y'
                o = self._new(dict(self._bottle_rect(cx, cy, axis), top=base + BOTTLE_2L[0]), dict(color=(.7, .8, .9), shape='bottle'), base, t, load, 'bottle')
                self._place_bottle(o, cx, cy, axis)
                o.update(from_event=True, weight=ev[1], before=self.before, rest=rest, cap=(1 if dx > 0 else -1) if axis == 'x' else (1 if dy > 0 else -1))
                self._forget_moved(o, ev, now)
                self.up_chain = None
                self._note('물건 #%d 생김: 물통 %s (무게 %+.0f g, 무게중심과 뚜껑으로)', o['id'], '가로' if axis == 'x' else '세로', ev[1])
                return True
        return False

    def _cold_bottle(self, t, load, floor):
        """켠 직후 (무게 기록이 없을 때): 상자 위 병 자국이 뚜렷하면 물통. 모양이 안 보이면 없어진다 (_gone)."""
        if self.t_start is None:
            self.t_start = t
        if self.cold_w is None:                        # 켠 뒤 처음 읽은 무게: 이미 짐이 있었나 (흔들려도 안정될 때까지 기다리지 않는다)
            self.cold_w = self.w
            return
        if self.cold_w < COLD_G or t - self.t_start > COLD_MAX or self.cold_done or not any(o['kind'] != 'bottle' for o in self.objs):
            return
        if self.hand_cells >= HAND_N:                  # 판 위 케이블도 처음 HAND_STILL_N 프레임은 손으로 세어진다: 기다리는 시간을 여기서 쓰지 않는다
            return
        if self.t_box is None:
            self.t_box = t
        if t - self.t_box > COLD_S:
            self.cold_done = True
            return
        if self.up_event is not None or any(o['kind'] == 'bottle' for o in self.objs) or any(not b.get('narrow') for b in self.flat_now):
            return
        if self.w is None or abs(self.w - self.cold_w) > COLD_DW:   # 켠 뒤 무게가 변했다: 켜기 전부터 있던 물병이 아니다 (무게 계단이 정한다)
            self.cold_done = True
            return
        if load is None or abs(load - self.cold_w) > COLD_DW:       # 지금 막 무게가 변하는 중
            return
        m = self._bottle_mark(floor, plate_needs_before=True, past_support=True)   # 판 위는 안 본다 (사진이 없으면 전자부품이 병 자국으로 세어져 상자 위 병을 가린다)
        if m is None or not m['band'] or m['sup'] is None or m['n_up'] < BAND_N:
            return
        w, d = m['mark_w'], m['mark_d']
        if min(w, d) < COLD_SHORT or max(w, d) < COLD_LONG:
            return
        fake = dict(color=(.7, .8, .9), shape='bottle')
        o = self._new(dict({k: m[k] for k in ('x0', 'x1', 'y0', 'y1', 'axis', 'band', 'sup')}, top=m['base'] + BOTTLE_2L[0]), fake, m['base'], t, load, 'bottle')
        o['from_geom'] = {k: m[k] for k in ('x0', 'x1', 'y0', 'y1')}
        o['from_geom']['top'] = m['base'] + .05        # 병 표면은 받침 위 몇 cm
        o['light'] = True
        o['cap'] = m['cap']
        self._note('물건 #%d 생김: 물통 %s (켠 직후, 자국 %d 칸)', o['id'], '가로' if o['axis'] == 'x' else '세로', m['n'])

    def _find_step(self, hm, floor):
        """병이 얹혀 윗면이 대부분 가려진 상자를 찾는다. 상자 자리 dict (hi = 병 면 높이) 또는 None."""
        g = self.grid
        c = g['cell']
        xs = g['x0'] + (np.arange(hm.shape[1]) + .5) * c
        yy = g['y0'] + (np.arange(hm.shape[0]) + .5) * c
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            up = hm - floor > FLAT_RISE
        up[:, (xs < PLATE_X0 - .02) | (xs > PLATE_X + .02)] = False
        up[yy > PLATE_Y] = False
        if up.sum() < FLAT_N:
            return None
        for grp in components(up):
            if len(grp) < FLAT_N:
                continue
            ys, xi = grp[:, 0], grp[:, 1]
            v = hm[ys, xi]
            vals, n = np.unique(np.round(v / FLAT_TOL), return_counts=True)
            hi = float(vals[n.argmax()]) * FLAT_TOL
            near = np.abs(v - hi) <= FLAT_TOL
            sub = np.zeros(hm.shape, bool)
            sub[ys[near], xi[near]] = True
            parts = components(sub)
            if not parts:
                continue
            keep = max(parts, key=len)
            i0, i1 = int(np.percentile(keep[:, 1], 2)), int(np.percentile(keep[:, 1], 98)) + 1
            j0, j1 = int(np.percentile(keep[:, 0], 2)), int(np.percentile(keep[:, 0], 98)) + 1
            w, d = (i1 - i0) * c, (j1 - j0) * c
            if min(w, d) > BOTTLE_NARROW or not BOTTLE_L[0] <= max(w, d) <= BOTTLE_L[1]:
                continue
            aw = (int(np.percentile(xi[near], 98)) + 1 - int(np.percentile(xi[near], 2))) * c
            ad = (int(np.percentile(ys[near], 98)) + 1 - int(np.percentile(ys[near], 2))) * c
            if min(aw, ad) > BOTTLE_NARROW:            # 그 높이 전체는 넓다: 위 상자 윗면이 투명한 병에 갈린 것이지 병 면이 아니다
                continue
            front = hm[max(0, j0 - STEP_ROWS):j0, i0:i1]
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', RuntimeWarning)
                low = (front - floor > FLAT_RISE) & (hi - front >= STEP_RISE[0]) & (hi - front <= STEP_RISE[1])
            if low.sum() < STEP_PART * (i1 - i0):
                continue
            lv, ln = np.unique(np.round(front[low] / FLAT_TOL), return_counts=True)
            lo = float(np.median(front[low][np.abs(front[low] - float(lv[ln.argmax()]) * FLAT_TOL) <= FLAT_TOL]))
            if (np.abs(front - lo) <= FLAT_TOL).sum() < STEP_PART * (i1 - i0):
                continue
            inside = hm[j0:j1, i0:i1]
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', RuntimeWarning)
                see = ~np.isfinite(inside) | (np.abs(inside - lo) <= .025)
            caps = [q for q in self._find_caps(lo) if i0 * c + g['x0'] - CAP_OUT <= q[0] <= i1 * c + g['x0'] + CAP_OUT and j0 * c + g['y0'] - CAP_OUT <= q[1] <= j1 * c + g['y0'] + CAP_OUT]
            if see.mean() < STEP_SEE and not caps:
                continue                               # 뚫린 칸도 뚜껑도 없다: 낮은 상자 뒤의 높은 상자일 수 있다
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', RuntimeWarning)
                lows = (ys < j0) & (np.abs(v - lo) <= STEP_RISE[0])    # 카메라 쪽 턱 칸 (윗면 모서리는 조금 낮게 찍힌다)
            lx, ly = xi[lows], ys[lows]
            if len(lx) < STEP_PART * (i1 - i0):        # 턱이 이 덩어리에 안 이어져 있다
                continue
            b0, b1 = int(np.percentile(lx, 2)), int(np.percentile(lx, 98)) + 1
            col = self.col
            color = np.clip(np.median(np.asarray(col)[ly, lx], axis=0) / 255., 0, 1) if col is not None and np.shape(col)[:2] == hm.shape else (.6, .5, .4)
            y1 = max(g['y0'] + j1 * c, g['y0'] + j0 * c + BOTTLE_2L[0])   # 병 뒤쪽은 안 보인다: 병 하나 지름만큼은 상자 위다
            return dict(x0=g['x0'] + b0 * c, x1=g['x0'] + b1 * c, y0=g['y0'] + int(np.percentile(ly, 2)) * c, y1=min(y1, PLATE_Y1),
                        top=lo, bottom=floor, hi=hi, kind='box', shape='box', color=color, held=False)
        return None

    def _cold_step(self, t, load, floor, hm):
        """켠 직후 무게 기록이 없을 때: 병이 얹힌 상자를 낮은 턱으로 만든다 (좁은 병 면을 긴 상자로 만들지 않게). 병은 다음 프레임부터 _cold_bottle 이 만든다."""
        if self.t_start is None or self.want_restore or self.cold_w is None or self.cold_w < COLD_G or t - self.t_start > COLD_MAX or self.cold_done \
                or hm is None or floor is None or self.w is None or abs(self.w - self.cold_w) > COLD_DW or not self._steady(t):
            return
        if any(o['kind'] != 'bottle' and o.get('from_step') for o in self.objs):
            return
        s = self._find_step(hm, floor)
        self.step_hits = self.step_hits + 1 if s is not None else 0
        if self.step_hits < STEP_HITS:
            return
        for o in list(self.objs):
            if _overlap_min(o, s) <= .3:
                continue
            if o['kind'] != 'bottle' and abs(o['top'] - s['top']) <= TOP_TOL:
                return                                 # 그 상자는 이미 있다
            if o.get('light') or (o['kind'] != 'bottle' and abs(o['top'] - s['hi']) <= TOP_TOL + FLAT_TOL):
                self.objs.remove(o)                    # 병 면을 상자나 병 모양 후보로 먼저 잡은 것
                self._note('물건 #%d 지움 (병이 얹힌 상자로 다시 봄)', o['id'])
        o = self._new({k: s[k] for k in ('x0', 'x1', 'y0', 'y1', 'top')}, s, s['bottom'], t, load, 'box')
        o['level'] = s['top']
        o['from_step'] = True
        o['dw'] = None
        self._note('물건 #%d 생김: 상자 %.0f x %.0f cm (켠 직후, 병이 얹혀 윗면 일부만 보임)', o['id'], (o['x1'] - o['x0']) * 100, (o['y1'] - o['y0']) * 100)

    def _before_sample(self, t, hm):
        """무게가 안정된 동안 높이맵 표본을 모은다. 표본끼리 많이 다르면 (손이 지나간다) 안 모은다."""
        if hm is None or t - self.ring_t < (BEFORE_EVERY if len(self.ring) >= BEFORE_MIN else BEFORE_FAST) or self.hand_cells >= HAND_N:
            return
        self.ring_t = t
        prev, self.ring_prev = self.ring_prev, hm
        if prev is None or prev.shape != hm.shape:
            self.ring.clear()
            return
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            moved = np.abs(hm - prev) > BEFORE_MOVE
        g = self.grid
        moved = int(moved[g['y0'] + (np.arange(hm.shape[0]) + .5) * g['cell'] <= PLATE_Y][:, np.abs(g['x0'] + (np.arange(hm.shape[1]) + .5) * g['cell']) <= PLATE_X + .05].sum())   # 판 위 칸만 (판 뒤 사람은 상관없다)
        if moved <= BEFORE_QUIET:
            self.ring.append(hm)

    def _freeze_before(self):
        """무게가 올라간 첫 계단: 모은 표본으로 '놓기 전 사진' 을 얼린다. 표본이 모자라면 지난 사진을 그대로 쓴다 (상자가 없어진 뒤라 지난 사진이 틀리면 한 장이라도 쓴다)."""
        if len(self.ring) >= (1 if self.before_stale else BEFORE_MIN):
            self.before_stale = False
            use = list(self.ring)
            use = use[:max(BEFORE_MIN, len(use) - BEFORE_DROP)]
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', RuntimeWarning)
                self.before = np.nanmedian(np.stack(use), 0)
        elif self.before is None:
            self.before = self.bg0
        self.ring.clear()

    @staticmethod
    def _rose(before, hm):
        """놓기 전보다 높아졌거나 놓기 전에 값이 없던 칸."""
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            return ~np.isfinite(before) | (hm - before > BEFORE_UP)

    @staticmethod
    def _fell(before, hm):
        """놓기 전보다 낮아진 칸 (병을 통과해 낮게 찍힌 깊이)."""
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            return hm - before < -BEFORE_DOWN

    def _steps(self, t, hm=None):
        """총무게가 안정될 때마다 전과 비교한다: 장력이 그대로인데 내려갔으면 물건을 뺀 계단으로 기억한다."""
        v = [g for ts, g in self.loads if t - STEP_S <= ts <= t]
        if len(v) < 5 or float(np.std(v)) > STEP_STD:
            return
        now = float(np.median(v))
        bs = [b for b in self.belts if t - STEP_S <= b[0] <= t]
        c1, c2 = (float(np.median([b[i] for b in bs])) for i in (1, 2)) if bs else (None, None)
        if self.level is None:
            self.level = (now, c1, c2)
            self.bg = self.bg0 = hm
            return
        prev, p1, p2 = self.level
        belt = c1 is not None and p1 is not None and (abs(c1 - p1) >= BELT_CHANGE or abs(c2 - p2) >= BELT_CHANGE)
        if abs(now - prev) < STEP_MIN and not belt:
            if hm is not None and self.up_event is None and not any(o['kind'] == 'bottle' for o in self.objs):
                self.bg = hm
            if self.up_event is None and t - self.up_t > BOTTLE_CHAIN:
                self._before_sample(t, hm)
                cog = self._cog_now(t)
                if cog is not None:
                    self.rest = (now, cog[0], cog[1], c1, c2)
            return
        if now < prev and not belt and self.up_chain is not None and t - self.up_chain[0] <= BOTTLE_CHAIN and now - self.up_chain[1] >= BOTTLE_G[0]:
            self.up_event = (t, now - self.up_chain[1])   # 눌렀던 손을 뗐다: 처음과 비교한 무게가 진짜다
            self.level = (now, c1, c2)
            return
        if now < prev:
            self.up_chain = None
        if not belt and now < prev:
            self.ring.clear()
            if self.up_event is not None and prev - now >= BOTTLE_DROP_PART * self.up_event[1]:
                self.up_event = None                   # 놓았다가 물통을 만들기 전에 도로 뺐다: 무게가 이미 없다
            old = self.drop_event
            if not self._box_drop(t, prev - now) and (old is None or t - old[0] > BOTTLE_LIFT_S or prev - now >= BOTTLE_DROP_PART * BOTTLE_G[0]):      # 방금 없어진 상자의 무게가 아닐 때만 '물통을 뺀 계단' 후보. 물병보다 가벼운 출렁임 (손으로 눌렀다 뗐다) 이 물통을 뺀 계단을 덮어쓰지 않게 (2026-11-03: -1915 g 뒤 -772 g)
                self.drop_event = (t, prev - now)
        elif not belt:
            base = self.up_chain[1] if self.up_chain is not None and t - self.up_chain[0] <= BOTTLE_CHAIN else prev
            if base == prev and (t - self.up_t > LATE_S or any(o['born'] >= self.up_t for o in self.objs)):   # 앞 계단이 아직 물건을 안 만들었으면 (손으로 받친 채 멈췄다 마저 놓는다) 그 전 사진을 그대로 쓴다
                self._freeze_before()
            elif base == prev:
                self.ring.clear()                      # 이 표본에는 놓는 물건이 없거나 병이 찍혀 있다: 다음 사진에 쓰면 앞 물병 칸이 새 칸이 된다
            self.up_chain = (t, base)
            self.up_event = (t, now - base)            # 손을 떼는 동안 무게가 두 번에 나뉘어 올라도 한 번으로 본다
            self.up_t = t
        self.level = (now, c1, c2)

    def _cog_now(self, t):
        """지금 무게중심 (mm). 가감속 · 흔들림 없이 STEP_S 초 이어졌을 때만."""
        v = [q for q in self.cogs if t - q[0] <= STEP_S]
        if len(v) < 5 or not self._steady(t):
            return None
        return float(np.median([q[1] for q in v])), float(np.median([q[2] for q in v]))

    def _bottle_est(self, o, t):
        """무게중심으로 낸 병 가운데 (카메라 좌표 m). 놓기 전 값이 없거나 다른 물건 무게가 섞였으면 지금 자리를 기준으로 다시 잡고 None."""
        cog = self._cog_now(t)
        now = self._load(t - STEP_S, t)
        if cog is None or now is None:
            return None
        rest, w = o.get('rest'), o['weight']
        bs = [b for b in self.belts if t - STEP_S <= b[0] <= t]
        c1, c2 = (float(np.median([b[i] for b in bs])) for i in (1, 2)) if bs else (None, None)
        belt = rest is not None and c1 is not None and None not in rest[3:] and (abs(c1 - rest[3]) > FOLLOW_BELT or abs(c2 - rest[4]) > FOLLOW_BELT)
        if rest is None or belt or (abs(now - rest[0] - w) > FOLLOW_G and abs(now - rest[0]) > .25 * w):
            cx, cy = (o['x0'] + o['x1']) / 2, (o['y0'] + o['y1']) / 2
            bx, by = BED_L / 2 - (cx - CELL_CX) * 1000, BED_W / 2 + (cy - CELL_CY) * 1000
            if now - w > 0:                            # 지금 자리에 병이 있다고 두고 나머지 무게 · 무게중심을 낸다
                o['rest'] = (now - w, (now * cog[0] - w * bx) / (now - w), (now * cog[1] - w * by) / (now - w), c1, c2)
            return None
        m = now - rest[0]
        if abs(m - w) > FOLLOW_G:
            return None                                # 들고 있거나 일부만 얹혔다
        return (CELL_CX - ((now * cog[0] - rest[0] * rest[1]) / m - BED_L / 2) / 1000, CELL_CY + ((now * cog[1] - rest[0] * rest[2]) / m - BED_W / 2) / 1000)

    def _place_bottle(self, o, cx, cy, axis):
        """병 가운데 · 방향으로 자리를 고친다 (크기는 2 L 고정)."""
        r, half = BOTTLE_2L[0] / 2, BOTTLE_2L[1] / 2
        rect = dict(x0=cx - r, x1=cx + r, y0=cy - half, y1=cy + half) if axis == 'y' else dict(x0=cx - half, x1=cx + half, y0=cy - r, y1=cy + r)
        sup = self._support(cx, cy)
        o.update(rect, axis=axis, sup=None if sup is None else sup['id'], band=True)
        o['from_geom'] = dict(rect, top=o['top'])

    def _unchanged(self, o, rect):
        """이 자리에 놓기 전 사진과 같은 칸 수 (보이는 칸만). 사진이 없으면 None."""
        before, hm, g = o.get('before'), self.hm, self.grid
        if before is None or hm is None or before.shape != hm.shape:
            return None
        c = g['cell']
        i0, i1 = max(0, int(round((rect['x0'] - g['x0']) / c))), max(0, int(round((rect['x1'] - g['x0']) / c)))
        j0, j1 = max(0, int(round((rect['y0'] - g['y0']) / c))), max(0, int(round((rect['y1'] - g['y0']) / c)))
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            return int((np.abs(hm[j0:j1, i0:i1] - before[j0:j1, i0:i1]) <= BEFORE_UP).sum())

    def _bottle_follow(self, o, t):
        """무게중심으로 자리, 뚜껑으로 방향을 따라간다. 물통이 하나일 때만 (둘이면 어느 병이 움직였는지 모른다)."""
        if sum(q['kind'] == 'bottle' for q in self.objs) != 1:
            return
        est = self._bottle_est(o, t)
        if est is None:
            o['far_n'] = 0
            return
        cx, cy = (o['x0'] + o['x1']) / 2, (o['y0'] + o['y1']) / 2
        axis = 'x' if o['x1'] - o['x0'] > o['y1'] - o['y0'] else 'y'
        caps = [(q[0] - est[0], q[1] - est[1]) for q in self._find_caps(o['top'] - BOTTLE_2L[0])]
        seen = [q for q in caps if CAP_DIST[0] <= math.hypot(*q) <= CAP_DIST[1]]
        pos = est
        if seen:                                       # 뚜껑이 보이면 뚜껑이 더 정확하다 (무게중심은 몇 cm 오차)
            dx, dy = max(seen, key=lambda q: math.hypot(*q))
            want = ('x', 1 if dx > 0 else -1) if abs(dx) > abs(dy) else ('y', 1 if dy > 0 else -1)
            half = BOTTLE_2L[1] / 2 - CAP_R
            pos = (est[0] + dx - want[1] * half, est[1] + dy) if want[0] == 'x' else (est[0] + dx, est[1] + dy - want[1] * half)
        elif self._mark_here(o, nolow=True) >= MARK_N:  # 병이 그 자리에 그대로 보인다: 무게중심이 움직인 것은 다른 이유다
            o['far_n'] = o['cap_miss'] = 0
            return
        if not (PLATE_X0 - .1 <= pos[0] <= PLATE_X1 + .1 and PLATE_Y0 - .02 <= pos[1] <= PLATE_Y1 + .02):
            o['far_n'] = 0
            return                                     # 판 밖: 무게중심 값을 믿을 수 없다
        o['far_n'] = o.get('far_n', 0) + 1 if math.hypot(pos[0] - cx, pos[1] - cy) > FOLLOW_MOVE else 0
        if o['far_n'] >= FOLLOW_N:
            self._place_bottle(o, pos[0], pos[1], axis)
            o['far_n'] = 0
            self._note('물건 #%d 물통 옮김 (x %.2f, y %.2f)', o['id'], pos[0], pos[1])
        if seen:
            o['cap_miss'] = 0
            o['cap_vote'] = (want, o['cap_vote'][1] + 1) if o.get('cap_vote') and o['cap_vote'][0] == want else (want, 1)
            if o['cap_vote'][1] >= CAP_VOTES and (want[0] != axis or o.get('cap') != want[1]):
                if want[0] != axis:
                    self._note('물건 #%d 물통 돌림 (%s, 뚜껑으로)', o['id'], '가로' if want[0] == 'x' else '세로')
                self._place_bottle(o, pos[0], pos[1], want[0])
                o['cap'] = want[1]
        elif axis == 'y':                              # 세로 자리가 보이는데 비어 있다: 시야 밖으로 가로로 누워 있다
            o['cap_miss'] = o.get('cap_miss', 0) + 1 if (self._unchanged(o, o) or 0) >= AXIS_EMPTY_N else 0
            if o['cap_miss'] >= CAP_MISS_N:
                self._note('물건 #%d 물통 돌림 (가로, 세로 자리가 비어 있음)', o['id'])
                self._place_bottle(o, est[0], est[1], 'x')
                o['cap'] = None
                o['cap_miss'] = 0
        elif axis == 'x' and o.get('cap') in (-1, 1):  # 가로면 뚜껑이 시야 안일 자리인데 안 보인다: 뚜껑이 먼 쪽으로 간 세로다
            sx, sy = est[0] + o['cap'] * BOTTLE_2L[1] / 2, est[1]
            inside = sy < FOV_K * (FOV_Z - (o['top'] - .05)) - .02 and abs(sx) < .25
            o['cap_miss'] = o.get('cap_miss', 0) + 1 if inside else 0
            if o['cap_miss'] >= CAP_MISS_N:
                self._note('물건 #%d 물통 돌림 (세로, 뚜껑이 안 보임)', o['id'])
                self._place_bottle(o, est[0], est[1], 'y')
                o['cap'] = 1
                o['cap_miss'] = 0

    def _drop_fits(self, o, drop):
        """이 무게 계단이 이 상자 무게로 설명되나. 상자 무게를 모르면 큰 물건이 빠진 것만 친다."""
        if o.get('dw') is None:
            return drop >= DROP_MIN
        w = max(o['dw'], DROP_MIN)
        return (o['dw'] < DROP_MIN or drop >= DROP_PART * w) and drop <= DROP_OVER * w + DROP_OVER_G

    def _box_drop(self, t, drop):
        """이 무게 계단이 방금 없어진 상자의 무게인가. 상자 무게를 모르면 그렇다고 본다."""
        if t - self.box_gone_t > BOX_DROP_S:
            return False
        return self.box_gone_dw is None or abs(drop - self.box_gone_dw) <= BOX_DROP_TOL * self.box_gone_dw

    def _excess_bottle(self):
        """잰 물통 무게 합이 안정된 총무게로 설명이 안 되면 가장 오래된 물통 하나 (아니면 None). 무게를 못 잰 물통은 안 따진다."""
        if self.level is None:
            return None
        known = [o for o in self.objs if o['kind'] == 'bottle' and o['weight'] != BOTTLE_WEIGHT and not o.get('light')]
        if known and sum(o['weight'] for o in known) > self.level[0] + BOTTLE_EXCESS:
            return known[0]
        return None

    def _placed(self, first, t):
        """놓으면서 늘어난 무게 (g). 처음 보이기 직전과 지금을 비교하고, 안 늘었으면 더 앞(LATE_S 초 전)과도 비교한다
        (놓고 몇 초 뒤에야 맞춰지는 일이 있다). 그 사이 장력이 변했으면 벨트가 섞여 믿을 수 없어 NaN, 비교할 값이 없으면 None."""
        now = self._load(t - .6, t)
        if now is None:
            return None
        result = None
        for a, b in ((first - 2.5, first - 1.0), (first - LATE_S - 1.5, first - LATE_S)):
            before = self._load(a, b)
            if before is None:
                continue
            for i in (1, 2):
                p, q = [v[i] for v in self.belts if a <= v[0] <= b], [v[i] for v in self.belts if t - .6 <= v[0] <= t]
                if p and q and abs(float(np.median(q)) - float(np.median(p))) >= BELT_CHANGE:
                    return float('nan')
            result = now - before
            if result >= BOTTLE_G[0]:
                return result
        return result

    def _steady(self, t):
        """무게가 안 흔들리나 (손이 안 닿아 있나). 무게를 모르면 그렇다고 친다."""
        v = [g for ts, g in self.loads if t - STEADY_S <= ts <= t]
        return len(v) < 3 or float(np.std(v)) <= STEADY_STD

    def _bottom(self, o, floor):
        if o['kind'] == 'bottle':                      # 물통은 높이가 고정이라 윗면 비교 없이, 중심 바로 아래 상자 위 (옆 상자에 걸친 것은 받침이 아니다)
            sup = next((q for q in self.objs if q['id'] == o.get('sup') and q['kind'] != 'bottle'), None) if o.get('sup') is not None else None
            if sup is None:
                sup = self._support((o['x0'] + o['x1']) / 2, (o['y0'] + o['y1']) / 2)
            under = [sup['top']] if sup is not None else []
        else:
            under = [q['top'] for q in self.objs if q is not o and q['kind'] != 'bottle' and _overlap_min(q, o) > .15
                     and q['top'] <= o['top'] - .02]
        bottom = max(under) if under else o['bottom']
        if not under and floor is not None and 0 < bottom - floor < .25:
            bottom = floor
        return bottom

    def _part_of_bottle(self, c, load):
        """물통 근처의 좁은 후보 = 물통 조각 (투명해서 뚜껑 · 몸통이 따로 잡힌다). 그 뒤 무게가 늘었으면 새 물건일 수 있어 뺀다."""
        if self._flat_cand(c):                         # 평평한 네모는 물통 조각이 아니라 상자다
            return None
        narrow = min(c['x1'] - c['x0'], c['y1'] - c['y0']) <= .13 and max(c['x1'] - c['x0'], c['y1'] - c['y0']) <= BOTTLE_L[1]
        cx, cy = (c['x0'] + c['x1']) / 2, (c['y0'] + c['y1']) / 2
        for o in self.objs:
            if o['kind'] != 'bottle':
                continue
            if self.w is not None and o['load_ref'] is not None and self.w - o['load_ref'] >= BOTTLE_PLUS_G:
                continue                               # 그 뒤 무게가 늘었다: 조각이 아니라 새 물건일 수 있다
            # 이 물통을 만든 그 물체 (무게로 물통이 됐으니 넓게 잡혀도 물통이다)
            if _overlap_min(c, o['from_geom']) > .5 and abs(c['top'] - o['from_geom']['top']) <= TOP_TOL:
                return o
            if not narrow or not (o['bottom'] + .02 <= c['top'] <= o['top'] + .03):
                continue
            if math.hypot((o['x0'] + o['x1']) / 2 - cx, (o['y0'] + o['y1']) / 2 - cy) > BOTTLE_NEAR:
                continue
            return o
        return None

    def _support(self, cx, cy):
        """이 점 바로 아래 맨 위 상자 (없으면 None)."""
        under = [q for q in self.objs if q['kind'] != 'bottle' and q['x0'] <= cx <= q['x1'] and q['y0'] <= cy <= q['y1']]
        return max(under, key=lambda q: q['top']) if under else None

    def _bottle_rect(self, cx, cy, axis='y'):
        """눕힌 2 L 물통 자리. axis = 긴 쪽 방향 ('y' 세로, 'x' 가로). 받침 상자가 충분히 길면 그 위에 들어오게 민다."""
        r, half = BOTTLE_2L[0] / 2, BOTTLE_2L[1] / 2
        sup = self._support(cx, cy)
        if axis == 'y':
            if sup is not None and sup['y1'] - sup['y0'] >= 2 * half:
                cy = min(max(cy, sup['y0'] + half), sup['y1'] - half)
            return dict(x0=cx - r, x1=cx + r, y0=cy - half, y1=cy + half)
        if sup is not None and sup['x1'] - sup['x0'] >= 2 * half:
            cx = min(max(cx, sup['x0'] + half), sup['x1'] - half)
        return dict(x0=cx - half, x1=cx + half, y0=cy - r, y1=cy + r)

    def _blocked(self, rect, cx, cy):
        """이 자리가 받침보다 높은 다른 상자에 걸리나 (거기에 놓을 수 없다)."""
        sup = self._support(cx, cy)
        base = sup['top'] if sup is not None else -1.0
        return any(q is not sup and q['kind'] != 'bottle' and q['top'] > base + .02 and _overlap_min(q, rect) > .15 for q in self.objs)

    def _bottle_mark(self, floor, before=None, plate_needs_before=False, past_support=False):
        """받침 위의 병 자국을 찾아 물통 자리 · 방향을 낸다. 없으면 None.
        상자마다 윗면 안에서 윗면보다 높은 칸 + 값이 없어진 칸을 세어 가장 많은 받침을 고른다. 빈 판은 값이 원래 잘 빠져서 높은 칸이 있어야 한다.
        before = 놓기 전 사진: 있으면 그때보다 높아진 칸만 '높은 칸' 이다. plate_needs_before: 사진이 없으면 판 위는 안 본다 (판 위 전자부품을 병으로 칠 수 있다)."""
        hm, g = self.hm, self.grid
        if hm is None or not g:
            return None
        c = g['cell']

        def cells(q, grow=0):
            return (max(0, int(round((q['x0'] - g['x0']) / c)) - grow), max(0, int(round((q['x1'] - g['x0']) / c)) + grow),
                    max(0, int(round((q['y0'] - g['y0']) / c)) - grow), max(0, int(round((q['y1'] - g['y0']) / c)) + grow))

        fin = np.isfinite(hm)
        was = np.isfinite(self.bg) if self.bg is not None and self.bg.shape == hm.shape else np.ones(hm.shape, bool)
        xs = g['x0'] + (np.arange(hm.shape[1]) + .5) * c
        yy = g['y0'] + (np.arange(hm.shape[0]) + .5) * c
        rose = self._rose(before, hm) if before is not None and before.shape == hm.shape else None
        fell = self._fell(before, hm) if rose is not None else np.zeros(hm.shape, bool)
        boxes = [o for o in self.objs if o['kind'] != 'bottle']
        best = None
        for S in sorted(boxes, key=lambda q: q['top'], reverse=True) + [None]:
            base = S.get('level', S['top']) if S is not None else floor
            if base is None or (S is None and plate_needs_before and rose is None):
                continue
            region = np.zeros(hm.shape, bool)
            if S is not None:
                i0, i1, j0, j1 = cells(S)
                region[j0 + 1:max(j0 + 1, j1 - 1), i0 + 1:max(i0 + 1, i1 - 1)] = True     # 상자가 조금 밀려도 가장자리는 안 친다
            else:
                region[:, (xs >= PLATE_X0 - .01) & (xs <= PLATE_X + .03)] = True
                region[yy > PLATE_Y] = False
                for q in boxes:
                    i0, i1, j0, j1 = cells(q, 2)
                    region[j0:j1, i0:i1] = False
            for q in boxes:                            # 받침 위에 얹힌 더 높은 상자 자리는 뺀다
                if q is not S and q['top'] > base + .02:
                    i0, i1, j0, j1 = cells(q, 1)
                    region[j0:j1, i0:i1] = False
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', RuntimeWarning)
                up = region & fin & (hm - base > BAND_LOW) & (hm - base < BAND_TOP)
            if rose is not None:
                up &= rose
            lost = region & ~fin & was
            occ = up | lost | (region & fell)
            if occ.sum() < MARK_N:
                continue
            grow = occ.copy()                          # 투명해서 칸이 군데군데 빈다: 한 칸씩 넓혀 이은 뒤 묶는다
            grow[1:, :] |= occ[:-1, :]; grow[:-1, :] |= occ[1:, :]; grow[:, 1:] |= occ[:, :-1]; grow[:, :-1] |= occ[:, 1:]
            parts = components(grow)
            if not parts:
                continue
            part = max(parts, key=len)
            keep = np.zeros(hm.shape, bool)
            keep[part[:, 0], part[:, 1]] = True
            u, o_ = keep & up, keep & occ
            n_up, n_occ = int(u.sum()), int(o_.sum())
            if n_occ < MARK_N or (S is None and n_up < MARK_FLOOR_UP):
                continue
            score = n_up * 1.5 + (n_occ - n_up)
            if best is None or score > best['score']:
                best = dict(score=score, S=S, base=base, u=u, occ=o_, n_up=n_up, n=n_occ)
        if best is None:
            return None

        def extent(mask):
            jj, ii = np.nonzero(mask)
            return int(np.percentile(ii, 3)), int(np.percentile(ii, 97)) + 1, int(np.percentile(jj, 3)), int(np.percentile(jj, 97)) + 1

        axis, use = None, best['occ']
        if best['n_up'] >= BAND_N:                     # 병 표면이 찍혔다: 그림자가 안 섞인 칸으로 본다
            i0, i1, j0, j1 = extent(best['u'])
            w, d = (i1 - i0) * c, (j1 - j0) * c
            if max(w, d) >= BAND_LONG and min(w, d) >= BAND_SHORT and max(w, d) >= BAND_RATIO * min(w, d):
                axis, use = ('x' if w > d else 'y'), best['u']
        if axis is None:
            i0, i1, j0, j1 = extent(best['occ'])
            w, d = (i1 - i0) * c, (j1 - j0) * c
            if max(w, d) >= MARK_RATIO * min(w, d):
                axis = 'x' if w > d else 'y'
        if axis == 'x' and past_support and best['S'] is not None and best['n_up'] >= BAND_N:    # 켠 직후: 받침 안 자국은 벨트 + 받침 밖으로 나간 병이면 넓적해 보인다. 병 면을 받침 밖까지 이어 재 본다
            vals, cnt = np.unique(np.round(hm[best['u']] / FLAT_TOL), return_counts=True)
            hi = float(vals[cnt.argmax()]) * FLAT_TOL
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', RuntimeWarning)
                face = np.abs(hm - hi) <= FLAT_TOL
            grown = face.copy()                        # 투명 · 뽁뽁이라 칸이 군데군데 빈다: 한 칸씩 넓혀 잇는다
            grown[1:, :] |= face[:-1, :]; grown[:-1, :] |= face[1:, :]; grown[:, 1:] |= face[:, :-1]; grown[:, :-1] |= face[:, 1:]
            pieces = [q for q in components(grown) if best['u'][q[:, 0], q[:, 1]].any()]
            if pieces:
                q = max(pieces, key=len)
                on = np.zeros(hm.shape, bool)
                on[q[:, 0], q[:, 1]] = True
                on &= face
                a0, a1, b0, b1 = extent(on)
                if (b1 - b0) * c >= BAND_LONG and (b1 - b0) >= BAND_RATIO * (a1 - a0):
                    axis, use = 'y', on
        i0, i1, j0, j1 = extent(use)
        r, L = BOTTLE_2L[0] / 2, BOTTLE_2L[1]
        front = g['y0'] + extent(best['occ'])[2] * c if use is best['occ'] else g['y0'] + j0 * c    # 카메라 쪽 끝은 그림자가 없어 믿을 수 있다
        cx = g['x0'] + (i0 + i1) / 2 * c
        if axis == 'x':                                # 상자 밖으로 나간 부분까지 넣어 가운데를 잡는다
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', RuntimeWarning)
                band = fin & (hm - best['base'] > BAND_LOW) & (hm - best['base'] < BAND_TOP)
            if rose is not None:
                band &= rose
            row0, row1 = j0, max(j0 + 1, j1)
            ii = np.nonzero(band[row0:row1].any(0) & (np.abs(xs) <= PLATE_X + .10))[0]
            if len(ii) >= 3:
                cx = g['x0'] + (int(np.percentile(ii, 3)) + int(np.percentile(ii, 97)) + 1) / 2 * c
            rect = dict(x0=cx - L / 2, x1=cx + L / 2, y0=front, y1=front + 2 * r)
        else:
            rect = dict(x0=cx - r, x1=cx + r, y0=front, y1=front + L)
        cap = None                                     # 뚜껑이 물병 끝 근처에 보이면 그 끝을 뚜껑에 맞춘다 (상자 밖으로 나온 부분은 자국에 안 잡힌다). -1 = x0 · y0 쪽, 1 = x1 · y1 쪽
        caps = self._find_caps(best['base'], rose)
        for x, y in caps if axis is not None else ():
            if axis == 'y' and abs(x - (rect['x0'] + rect['x1']) / 2) <= CAP_SIDE:
                if rect['y0'] - CAP_OUT <= y <= rect['y0'] + CAP_IN:
                    rect.update(y0=y - CAP_R, y1=y - CAP_R + L)
                    cap = -1
                elif rect['y1'] - CAP_IN <= y <= rect['y1'] + CAP_OUT:
                    rect.update(y0=y + CAP_R - L, y1=y + CAP_R)
                    cap = 1
            elif axis == 'x' and abs(y - (rect['y0'] + rect['y1']) / 2) <= CAP_SIDE:
                if rect['x0'] - CAP_OUT <= x <= rect['x0'] + CAP_IN:
                    rect.update(x0=x - CAP_R, x1=x - CAP_R + L)
                    cap = -1
                elif rect['x1'] - CAP_IN <= x <= rect['x1'] + CAP_OUT:
                    rect.update(x0=x + CAP_R - L, x1=x + CAP_R)
                    cap = 1
            if cap is not None:
                break
        S = best['S']
        if S is None and cap is None:                  # 판 위 병은 뚜껑이 안 보이면 끝을 모른다 (보이는 칸만으로는 판 밖으로 밀린다): 판 끝 안으로 되민다
            over = max(rect['x1'] - PLATE_X1 - .01, 0) - max(PLATE_X0 - .01 - rect['x0'], 0)
            rect.update(x0=rect['x0'] - over, x1=rect['x1'] - over)
            over = max(rect['y1'] - PLATE_Y1 - .01, 0)
            rect.update(y0=rect['y0'] - over, y1=rect['y1'] - over)
        side = 100.0 if S is None else min(S['x1'] - S['x0'], S['y1'] - S['y0']) / c    # 받침의 짧은 쪽 (칸)
        need = min(MARK_STRONG, MARK_PART * BOTTLE_2L[0] / c * min(BOTTLE_2L[1] / c, side))
        zs = hm[best['u']]
        flat = float((np.abs(zs - np.median(zs)) <= FLAT_TOL).mean()) if len(zs) else 0.0
        return dict(rect, flat=flat, capped=cap is not None, cap=cap, caps=caps, need=need, axis=axis or 'y', band=axis is not None, sup=None if S is None else S['id'], base=best['base'], n=best['n'], n_up=best['n_up'],
                    mark_w=(i1 - i0) * c, mark_d=(j1 - j0) * c)

    def _bottle_fit(self, cx, cy, floor, axis='y'):
        """자국을 모를 때 (켠 직후 뚜껑만 보일 때): axis 방향으로 두되 다른 상자에 걸리면 반대로."""
        rect = self._bottle_rect(cx, cy, axis)
        if self._blocked(rect, cx, cy):
            turn = 'x' if axis == 'y' else 'y'
            other = self._bottle_rect(cx, cy, turn)
            if not self._blocked(other, cx, cy):
                return dict(other, axis=turn, band=False)
        return dict(rect, axis=axis, band=False)

    def _mark_here(self, o, nolow=False):
        """이 물통 자리에 아직 자국이 있나 (칸 수). nolow: 자리 대부분이 낮으면 안 센다 (받침 뚜껑이 통째로 눌린 채면 병을 빼도 낮다. 일부만 낮으면 굴절된 병 칸이다)."""
        hm, g = self.hm, self.grid
        if hm is None or not g:
            return 0
        c = g['cell']
        i0, i1 = max(0, int(round((o['x0'] - g['x0']) / c))), max(0, int(round((o['x1'] - g['x0']) / c)))
        j0, j1 = max(0, int(round((o['y0'] - g['y0']) / c))), max(0, int(round((o['y1'] - g['y0']) / c)))
        v = hm[j0:j1, i0:i1]
        base = o['top'] - BOTTLE_2L[0]
        was = np.isfinite(self.bg[j0:j1, i0:i1]) if self.bg is not None and self.bg.shape == hm.shape else np.ones(v.shape, bool)
        before = o.get('before')
        known = before is not None and before.shape == hm.shape
        new = self._rose(before, hm)[j0:j1, i0:i1] if known else True
        low = self._fell(before, hm)[j0:j1, i0:i1] if known else False
        if nolow and known and np.size(low) and np.mean(low) >= LOW_FRAC:
            low = False
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            return int((~np.isfinite(v) & was).sum() + (((v - base > BAND_LOW) & new) | low).sum())    # 값이 없어진 칸 (원래 값이 있던 곳만) + 받침보다 높은 칸 + 낮아진 칸 (놓기 전 사진이 있으면 그때와 달라진 칸만)

    @staticmethod
    def _cap_at_end(o, cap):
        """뚜껑이 지금 물통 자리의 끝에 있나 (그러면 물병은 안 움직였다)."""
        x, y = cap
        if o['x1'] - o['x0'] > o['y1'] - o['y0']:                # 가로: 왼쪽 · 오른쪽 끝
            return abs(y - (o['y0'] + o['y1']) / 2) <= CAP_STAY and min(abs(x - o['x0']), abs(x - o['x1'])) <= CAP_STAY
        return abs(x - (o['x0'] + o['x1']) / 2) <= CAP_STAY and min(abs(y - o['y0']), abs(y - o['y1'])) <= CAP_STAY

    def _vacated(self, o, m):
        """지금 물통 자리 중 새 자리와 안 겹치는 칸이 얼마나 비었나 (0 ~ 1). 값이 원래 없던 칸은 모른다."""
        hm, g = self.hm, self.grid
        c = g['cell']
        cells = lambda q: (max(0, int(round((q['x0'] - g['x0']) / c))), max(0, int(round((q['x1'] - g['x0']) / c))),
                           max(0, int(round((q['y0'] - g['y0']) / c))), max(0, int(round((q['y1'] - g['y0']) / c))))
        part = np.zeros(hm.shape, bool)
        i0, i1, j0, j1 = cells(o)
        part[j0:j1, i0:i1] = True
        i0, i1, j0, j1 = cells(m)
        part[j0:j1, i0:i1] = False
        v = hm[part]
        was = np.isfinite(self.bg)[part] if self.bg is not None and self.bg.shape == hm.shape else np.ones(v.shape, bool)
        base = o['top'] - BOTTLE_2L[0]
        fin = np.isfinite(v)
        before = o.get('before')
        high = v[fin] - base > BAND_LOW
        if before is not None and before.shape == hm.shape:
            high = (high & self._rose(before, hm)[part][fin]) | self._fell(before, hm)[part][fin]
        marked = int(high.sum() + (~fin & was).sum())
        empty = int(fin.sum()) - int(high.sum())
        return empty / max(1, empty + marked)

    def _forget_box(self, o):
        """놓기 전 사진에 찍힌 없어진 상자 칸을 지운다 (그대로 쓰면 그 자리 전체가 낮아진 칸, 곧 병 자국으로 세어진다).
        상자 높이의 칸만 지우고, 켠 뒤 첫 사진이 그 아래를 봤으면 그 값을 쓴다 (전자부품 칸까지 '모름' 이 되면 새 칸, 곧 병 표면으로 세어진다)."""
        g = self.grid
        if not g:
            return
        c = g['cell']
        i0, i1 = max(0, int((o['x0'] - g['x0']) / c) - 2), max(0, int(np.ceil((o['x1'] - g['x0']) / c)) + 2)
        j0, j1 = max(0, int((o['y0'] - g['y0']) / c) - 2), max(0, int(np.ceil((o['y1'] - g['y0']) / c)) + 2)

        def blank(b):
            b = b.copy()
            sub = b[j0:j1, i0:i1]
            mid = (o['bottom'] + o['top']) / 2
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', RuntimeWarning)
                gone = ~(sub <= mid)
                sub[gone] = np.nan
                if self.bg0 is not None and self.bg0.shape == b.shape:
                    old = self.bg0[j0:j1, i0:i1]
                    under = gone & (old <= mid)
                    sub[under] = old[under]
            return b

        self.before_stale = True
        if self.before is not None:
            self.before = blank(self.before)
        for q in self.objs:
            if q.get('before') is not None:
                q['before'] = blank(q['before'])

    def _find_caps(self, base, rose=None):
        """파란 덩어리 중 받침 위 병 높이에 있는 것들의 자리 [(x, y), ...]. 물병에 비친 파란색도 섞이므로 쓰는 쪽에서 물병 끝에 있는 것을 고른다."""
        hm, g, col = self.hm, self.grid, self.col
        if col is None or np.shape(col)[:2] != hm.shape:
            return []
        col = np.asarray(col, np.float32)
        blue = (col[..., 2] > col[..., 0] * 1.32) & (col[..., 1] > col[..., 0] * 1.15) & (col[..., 2] > 65)
        if blue.sum() < CAP_N:
            return []
        c = g['cell']
        out = []
        for part in components(blue):
            if len(part) < CAP_N or (rose is not None and rose[part[:, 0], part[:, 1]].mean() < CAP_ROSE):
                continue
            c_ = col[part[:, 0], part[:, 1]]
            if float(np.median(c_[:, 2] / np.maximum(c_[:, 0], 1.0))) < CAP_BLUE:
                continue
            z = hm[part[:, 0], part[:, 1]]
            z = z[np.isfinite(z)]
            if len(z) and CAP_Z[0] <= float(np.median(z)) - base <= CAP_Z[1]:
                out.append((g['x0'] + (part[:, 1].mean() + .5) * c, g['y0'] + (part[:, 0].mean() + .5) * c))
        return out

    def _bottle_refit(self, o, t, floor):
        """손을 뗀 상태에서 다시 본다 (무게가 그대로여도 옮겼을 수 있다). 같은 답이 두 번 이어질 때만 고친다. 다른 받침으로 옮기려면 지금 자리가 비어야 한다."""
        if t - o.get('fit_t', -1e9) < FIT_EVERY or not self._steady(t) or self.hand_cells >= HAND_N or any(not b.get('narrow') for b in self.flat_now) \
                or sum(q['kind'] == 'bottle' for q in self.objs) != 1:
            return
        o['fit_t'] = t
        m = self._bottle_mark(floor, o.get('before'), plate_needs_before=True)
        if m is None or not m['band'] or m['n'] < m['need']:   # 자국이 병 크기가 안 되면 믿지 않는다 (만들 때와 같은 기준)
            o['fit_last'] = None
            return
        cx, cy = (m['x0'] + m['x1']) / 2, (m['y0'] + m['y1']) / 2
        est = self._bottle_est(o, t)
        if est is not None and math.hypot(est[0] - cx, est[1] - cy) > BOTTLE_MOVE:
            o['fit_last'] = None                       # 무게중심이 아는 병 자리와 멀다: 전자부품 · 손이 자국처럼 보인 것
            return
        same_place = m['axis'] == o.get('axis') and math.hypot((o['x0'] + o['x1']) / 2 - cx, (o['y0'] + o['y1']) / 2 - cy) <= BOTTLE_MOVE and m['sup'] == o.get('sup')
        if not same_place and (m['n_up'] < MOVE_UP or any(self._cap_at_end(o, q) for q in m['caps'])):
            o['fit_last'] = None                       # 병 표면이 뚜렷하지 않거나 뚜껑이 그대로다: 벨트 · 손이 자국처럼 보인 것
            return
        last, o['fit_last'] = o.get('fit_last'), (m['axis'], cx, cy)
        if last is None or last[0] != m['axis'] or math.hypot(last[1] - cx, last[2] - cy) > .04:
            return
        if m['sup'] != o.get('sup') and self._mark_here(o) >= MARK_N:
            return
        o['band'] = True
        if not same_place and self._vacated(o, m) < VACATED:
            return                                     # 원래 자리에 병이 그대로 있다: 벨트 · 손이 자국처럼 보인 것
        if same_place:                                 # 같은 자리: 뚜껑이 보이면 길이 방향만 뚜껑에 맞춘다
            keys = ('x0', 'x1') if m['axis'] == 'x' else ('y0', 'y1')
            if m['capped']:
                o['cap'] = m['cap']
                if abs(o[keys[0]] - m[keys[0]]) > CAP_MOVE:
                    o.update({k: m[k] for k in keys})
            return
        self._note('물건 #%d 물통 자리 고침 (%s, 자국 %d 칸)', o['id'], '가로' if m['axis'] == 'x' else '세로', m['n'])
        o.update({k: m[k] for k in ('x0', 'x1', 'y0', 'y1', 'axis', 'sup', 'cap')})
        o['from_geom'] = {k: m[k] for k in ('x0', 'x1', 'y0', 'y1')}
        o['from_geom']['top'] = m['base'] + BOTTLE_2L[0]

    def _same_box(self, g, dw, t):
        """방금 만든 상자와 같은 물건인가. 같으면 그 상자를 넓히고 True. 놓는 중에 잡힌 낮은 조각은 여기서 지운다."""
        for q in self.objs:                            # 이미 있는 상자와 같은 자리 같은 높이 (같은 윗면을 후보 둘로 잡은 것)
            if q['kind'] != 'bottle' and _overlap_min(q, g) > .5 and abs(q['top'] - g['top']) <= TOP_TOL:
                return True
            if q['kind'] != 'bottle' and t - q['born'] <= MERGE_S and _overlap_min(q, g) > .7 and abs(q['top'] - g['top']) <= MERGE_TOP:
                q['top'] = q['level'] = max(q['top'], g['top'])   # 방금 만든 상자를 내려놓는 중: 윗면만 따라간다
                return True
        if dw is None or math.isnan(dw):
            return False
        for q in [o for o in self.objs if o['kind'] != 'bottle' and t - o['born'] <= MERGE_S and o.get('dw') is not None]:
            if _overlap_min(q, g) > .7 and abs(dw - q['dw']) <= MERGE_G and abs(q['top'] - g['top']) <= STACK_SAME:
                q['top'] = q['level'] = max(q['top'], g['top'])   # 놓으면서 무게가 더 안 늘었는데 같은 자리에 다른 높이: 내려놓는 중인 한 상자를 위아래 둘로 잡은 것이다 (2026-11-04: .46 + .51)
                self._note('물건 #%d 윗면 고침 (같은 상자, 무게 %+.0f g)', q['id'], dw)
                return True
        if dw < BOTTLE_G[0]:
            return False
        for q in [o for o in self.objs if o['kind'] != 'bottle' and t - o['born'] <= MERGE_S and o.get('dw') is not None]:
            common = min(q['x1'], g['x1']) - max(q['x0'], g['x0'])
            gap = max(q['y0'] - g['y1'], g['y0'] - q['y1'])
            if abs(dw - q['dw']) <= MERGE_G and common >= MERGE_X * min(q['x1'] - q['x0'], g['x1'] - g['x0']) and gap <= MERGE_GAP \
                    and abs(q['top'] - g['top']) <= MERGE_TOP:
                for k in ('x0', 'y0'):
                    q[k] = min(q[k], g[k])
                for k in ('x1', 'y1', 'top'):
                    q[k] = max(q[k], g[k])
                q['level'] = q['top']
                q['hist'].clear()
                self._note('물건 #%d 넓힘 (같은 상자: %.0f x %.0f cm)', q['id'], (q['x1'] - q['x0']) * 100, (q['y1'] - q['y0']) * 100)
                return True
        return False

    def _is_arm(self, p):
        """좁고 아주 높은 것은 화물이 아니라 팔 (세운 병보다 높다)."""
        g = p['geom']
        support = [o['top'] for o in self.objs if o['kind'] != 'bottle' and _overlap_min(o, g) > .4 and o['top'] < g['top']]
        h = g['top'] - max(support + [p['c']['bottom']])
        return p['kind'] != 'bottle' and max(g['x1'] - g['x0'], g['y1'] - g['y0']) < ARM_W and h > ARM_H

    def _too_small(self, p, kind):
        g = p['geom']
        return kind != 'bottle' and (g['x1'] - g['x0']) * (g['y1'] - g['y0']) < MIN_BOX

    def _promote(self, p, t, load):
        g = p['geom']
        dw = self._placed(p['first'], t)
        kind = p['kind']
        direct = p['c'].get('direct')
        cap = p['c'].get('shape') == 'cylinder' and not direct   # 뚜껑 색이 실제로 보인 것
        if kind == 'bottle' or cap:                    # 병 모양 후보는 켠 직후에만 (무게 기록이 없을 때). 그 뒤로는 무게 계단 + 자국이 정한다
            if dw is not None or any(o['kind'] == 'bottle' for o in self.objs):
                return
            if self._support((g['x0'] + g['x1']) / 2, (g['y0'] + g['y1']) / 2) is not None and not self.cold_done:
                return                                 # 받침 상자가 있으면 자리 · 방향은 병 자국이 정한다 (_cold_bottle)
            kind = 'bottle'
        elif dw is not None and math.isnan(dw):        # 벨트를 조이거나 푸는 중
            return
        elif direct and p['c'].get('narrow') and (dw is None or BOTTLE_G[0] <= dw < BOTTLE_G[1] or (dw < BOTTLE_G[1] and any(o['kind'] == 'bottle' or (o.get('from_step') and _overlap_min(o, g) > .3) for o in self.objs))):
            return                                     # 물병 폭의 띠: 무게가 물병만큼 늘었거나 물통이 있으면 병 표면이다 (2 L 보다 무거우면 좁아도 상자, 좁은 띠는 손을 뗀 뒤에만 본다)
        elif not direct:                               # 후보(cargo_model): 네모로 확인되거나, 무게가 상자만큼 늘고 좁지 않을 때만
            short = min(g['x1'] - g['x0'], g['y1'] - g['y0'])
            if dw is None:
                if short < FLAT_SIDE:                  # 켠 직후: 좁은 띠는 물병 조각일 수 있다
                    return
            elif not (dw >= BOTTLE_G[0] and self._flat_cand(g)) and not (dw >= BOTTLE_G[1] and short >= BOX_MIN_SIDE):
                return
        if kind == 'bottle':
            cx, cy = (g['x0'] + g['x1']) / 2, (g['y0'] + g['y1']) / 2
            b = dict(self._bottle_fit(cx, cy, self.floor, 'x' if g['x1'] - g['x0'] > g['y1'] - g['y0'] else 'y'), top=p['c']['top'])
            o = self._new(b, p['c'], p['c']['bottom'], t, load, 'bottle')
            o['from_geom'] = dict(g)
            o['light'] = True                          # 무게로 확인 못 한 물통: 모양이 안 보이면 없앤다
            self._note('물건 #%d 생김: 물통 (켠 직후, 근거 %s)', o['id'], '뚜껑 색' if cap else '병 모양')
        else:
            if self._too_small(p, kind):               # 안 만들 후보로 아래 물건을 지우지 않게 먼저 본다
                return
            # 새 물건이 아래 물건보다 훨씬 넓거나 아래 것이 아주 얇으면, 아래 것은 상자가 놓이는 동안 일부만 본 것이다
            # (작은 상자 위에 큰 상자를 얹는 일은 드물다). 지우고 그 자리를 새 물건이 차지한다
            area = lambda q: (q['x1'] - q['x0']) * (q['y1'] - q['y0'])
            self.objs = [o for o in self.objs if not (
                o['kind'] != 'bottle' and _overlap_min(o, g) > .4 and o['top'] < g['top'] - RISE
                and (area(g) > PARTIAL_AREA * area(o) or o['top'] - o['bottom'] < PARTIAL_H))]
            # 무게로 먼저 만든 물통 자리에 상자가 나타났다 = 상자를 놓다가 잠깐 멈춘 무게였다
            back = self.box_gone_geom is not None and t - self.box_gone_t <= BOX_BACK_S and _overlap_min(self.box_gone_geom, g) > .7 and abs(self.box_gone_geom['top'] - g['top']) <= MERGE_TOP
            for o in [] if back else [q for q in self.objs if q.get('from_event') and t - q['born'] <= FAST_S + LATE_S
                                      and (_overlap_min(q, g) > .3 or _overlap_min(q['from_geom'], g) > .3)]:
                self.objs.remove(o)
                self._note('물건 #%d 지움 (물통이 아니라 놓는 중인 상자였다)', o['id'])
            if self._same_box(g, dw, t):
                return
            o = self._new(g, p['c'], p['c']['bottom'], t, load, 'box')
            o['dw'] = dw
            o['w0'] = self.level[0] if self.level is not None else None
            self.up_chain = None                       # 이 무게는 이 상자 몫이다
            self._note('물건 #%d 생김: 상자 %.0f x %.0f cm (놓을 때 무게 %s g)', o['id'], (g['x1'] - g['x0']) * 100, (g['y1'] - g['y0']) * 100,
                       '-' if dw is None else '%+.0f' % dw)
        for q in self.objs:                            # 무게 기준을 새 값으로 (다른 물통이 '무게가 줄었다' 고 착각하지 않게)
            q['load_ref'] = self.w

    def update(self, cands, frame, t, floor, load, belts=None):
        """load = 총무게 g, belts = (앞 장력, 뒤 장력) N. 장력이 없으면 (옛 녹화) 벨트 변화를 모른 채 총무게만 본다."""
        if load is not None:
            self.loads.append((t, float(load)))
        if belts is not None and None not in belts:
            self.belts.append((t, float(belts[0]), float(belts[1])))
        sn = frame['hdr'].get('sensors') or {}
        if sn.get('cog_x_mm') is not None and sn.get('cog_y_mm') is not None and abs(sn.get('imu1_ax_G') or 0) < FOLLOW_ACC and abs(sn.get('imu1_ay_G') or 0) < FOLLOW_ACC \
                and abs(sn.get('imu1_gz_dps') or 0) < FOLLOW_GYRO:
            self.cogs.append((t, float(sn['cog_x_mm']), float(sn['cog_y_mm'])))
        self.w = self._load(t - 1.0, t)
        if self.w is None and load is not None:
            self.w = float(load)
        hm = self._height_median(frame)
        self.hm, self.grid, self.floor = hm, frame['hdr'].get('grid'), floor
        self.col = frame.get('col')
        self._restore(t)
        self.flat_now = self._find_new(hm, frame, floor)
        self._steps(t, hm)
        self._bottle_from_weight(t, load, hm, frame, floor)
        self._cold_bottle(t, load, floor)
        self._cold_step(t, load, floor, hm)
        for c in list(cands) + self.flat_now:
            kind = c.get('kind') or 'box'
            if abs((c['x0'] + c['x1']) / 2) > PLATE_X or (c['y0'] + c['y1']) / 2 > PLATE_Y:  # 판 밖 (바닥에 놓인 짐 · 사람)
                continue
            same = [o for o in self.objs if o['kind'] != 'bottle' and _overlap_min(o, c) > .4 and abs(o['top'] - c['top']) <= TOP_TOL]
            if same:
                self._see(max(same, key=lambda o: _overlap_min(o, c)), c, t, load)
                continue
            inside = [o for o in self.objs if o['kind'] != 'bottle' and _overlap_min(o, c) > .6
                      and o['bottom'] - .02 <= c['top'] <= o['top'] + TOP_TOL]
            if inside:                                 # 있는 물건 높이 안쪽의 면 (덮개 · 턱 · 맞춤 오차)
                continue
            bottle = self._part_of_bottle(c, load)
            if bottle is not None:
                bottle['seen'] = t
                bottle['load_ref'] = self.w
                bottle['centers'].append((t, (c['x0'] + c['x1']) / 2, (c['y0'] + c['y1']) / 2))
                continue
            p = next((q for q in self.pending if q['kind'] == kind and _overlap_min(q['geom'], c) > .5
                      and abs(q['geom']['top'] - c['top']) <= PENDING_TOP), None)
            if p is None:
                p = dict(kind=kind, first=t, geom={k: c[k] for k in GEOM}, hist=[], c=c)
                self.pending.append(p)
            p['last'], p['c'] = t, c
            p['hist'].append({k: c[k] for k in GEOM})
            p['geom'] = {k: float(np.median([g[k] for g in p['hist'][-15:]])) for k in GEOM}
        keep = []
        for p in self.pending:
            if t - p['last'] > .5:
                continue
            direct = p['c'].get('direct')
            hold = FLAT_HOLD if direct else (HOLD_FAST if t - self.up_t <= FAST_S else HOLD)
            if t - p['first'] < hold or (not self._steady(t) and (not direct or p['c'].get('narrow') or t - p['first'] < FLAT_MAX)):
                keep.append(p)
            elif not self._is_arm(p) and self._part_of_bottle(dict(p['geom'], bottom=p['c']['bottom']), load) is None:
                self._promote(p, t, load)
        self.pending = keep
        for o in list(self.objs):
            if self._gone(o, frame, t, load, hm):
                self.objs.remove(o)
                self._note('물건 #%d 없어짐 (%s)', o['id'], o['kind'])
                if o.get('before') is not None:        # 지금 사진에는 이 물통이 찍혀 있을 수 있다
                    self.before = o['before']
                if o['kind'] != 'bottle':              # 무게 계단이 상자 없어지기 조금 전에 왔어도 물통으로 치지 않는다
                    self.box_gone_t, self.box_gone_dw = t, o.get('dw')
                    self.box_gone_geom = {k: o[k] for k in GEOM}
                    self._forget_box(o)
                    if self.drop_event is not None and t - self.drop_event[0] <= BOX_DROP_S and self._box_drop(t, self.drop_event[1]):
                        self.drop_event = None
                for q in self.objs:
                    q['load_ref'] = self.w
        ex = self._excess_bottle()
        if ex is not None:
            self.objs.remove(ex)
            self.drop_event = None                     # 이 무게 감소는 이 물통으로 처리했다
            if ex.get('before') is not None:
                self.before = ex['before']
            self._note('물통 #%d 없앰 (물통 무게 %.0f g 를 총무게 %.0f g 로 설명 못 함)', ex['id'], ex['weight'], self.level[0])
        res = []
        for o in sorted(self.objs, key=lambda q: q['top']):
            self._settle(o, t)
            if hm is not None and o['kind'] != 'bottle' and not self._covered(o):
                self._level(o, hm, frame['hdr']['grid'])
                self._track(o, hm, frame['hdr']['grid'])
            bottom = self._bottom(o, floor)
            if o['kind'] == 'bottle':                  # 눕힌 2 L 고정. 찍힌 모양으로 방향 · 자리를 고치고, 안 찍히면 후보 중심이 오래 벗어날 때 그리로 옮긴다
                o['top'] = bottom + BOTTLE_2L[0]
                self._bottle_follow(o, t)
                self._bottle_refit(o, t, floor)
                recent = [(x, y) for ts, x, y in o['centers'] if t - ts <= GEOM_HOLD]
                if len(recent) >= 8 and not o.get('band'):
                    mx, my = float(np.median([a for a, _ in recent])), float(np.median([b for _, b in recent]))
                    if math.hypot(mx - (o['x0'] + o['x1']) / 2, my - (o['y0'] + o['y1']) / 2) > BOTTLE_MOVE:
                        o.update(self._bottle_fit(mx, my, floor))
                        o['centers'].clear()
            res.append(dict(id=o['id'], x0=o['x0'], x1=o['x1'], y0=o['y0'], y1=o['y1'], bottom=bottom, top=o['top'],
                            color=np.clip(o['color'], 0, 1), kind=o['kind'], shape=o['shape'], held=False, src=o['cand'], cap=o.get('cap')))
        self._save(t)
        return res


class StackLayers:
    def __init__(self, save_path=None):
        self.ray_key = None
        self.side_hist = {k: collections.deque(maxlen=SIDE_HIST) for k in (False, True)}   # 옆 짐은 런이 바뀌어도 기억
        self.items = []          # 작은 물건 추적 (런이 바뀌어도 그대로)
        self.next_item = 0
        self.carry = None        # 마지막으로 낸 층 (런이 바뀌어 다시 잴 동안 한 덩어리로 보이지 않게)
        self.memory = ObjectMemory(save_path)   # 화물을 하나하나 기억 (번호 고정, 하나 빼도 나머지는 그대로)
        self.reset()

    def reset(self):
        self.memory.reset()
        self.stamp = None
        self.floors = collections.deque(maxlen=FLOOR_HIST)
        if not hasattr(self, "floor"):
            self.floor = None    # 받침면은 런이 바뀌어도 그대로 (카메라 고정). 새 값이 1 cm 넘게 다르면 바뀐다
        self.hist = collections.defaultdict(lambda: collections.deque(maxlen=HIST))
        self.shown = {}          # 원래 상자 id -> 화면에 낸 층 목록
        self.ms = 0.0

    # ------------------------------------------------------------ 점 구름
    def cloud(self, frame):
        hdr = frame['hdr']; sf = hdr['surface']; intr = sf['intr']
        key = tuple(intr.values()) + tuple(np.asarray(sf['R']).ravel())
        if key != self.ray_key:
            self.rays = (_rays(intr) @ np.asarray(sf['R'], np.float32).T)[::STRIDE, ::STRIDE]
            self.ray_key = key
        dep = frame['depth'][::STRIDE, ::STRIDE].astype(np.float32) * .001
        P = self.rays * dep[..., None]
        P[..., 2] -= hdr['calib']['ref_h']
        dx = P[2:, 1:-1] - P[:-2, 1:-1]
        dy = P[1:-1, 2:] - P[1:-1, :-2]
        n = np.cross(dx, dy)
        n /= np.maximum(np.linalg.norm(n, axis=-1, keepdims=True), 1e-9)
        Q = P[1:-1, 1:-1]
        ok = (dep[1:-1, 1:-1] > 0) & (np.linalg.norm(dx, axis=-1) < .03) & (np.linalg.norm(dy, axis=-1) < .03)
        rgb = frame['rgb'][::STRIDE, ::STRIDE][1:-1, 1:-1]
        front = ok & (np.abs(n[..., 1]) > .7)
        top = ok & (np.abs(n[..., 2]) > .85)
        return Q[front], rgb[front], Q[top], Q[ok], rgb[ok]

    # ------------------------------------------------------------ 바닥 (화물 받침면)
    def find_floor(self, top, below):
        z = top[(np.abs(top[:, 0]) <= PLATE_X + .02) & (top[:, 1] <= PLATE_Y), 2]     # 판 위 점만 (판 옆 · 뒤 바닥의 낮은 물건을 받침면으로 잡으면 판 전체가 상자로 나온다)
        z = z[z < below - .08]
        if len(z) < 60:
            return None
        hist, e = np.histogram(z, np.arange(np.floor(z.min() * 100) / 100, below, BIN))
        good = np.nonzero(hist >= 60)[0]
        if not len(good):
            return None
        i = good[0]                                     # 가장 낮은 넓은 수평면
        zz = z[np.abs(z - (e[i] + BIN / 2)) < .012]
        return float(np.median(zz))

    # ------------------------------------------------------------ 한 상자 나누기
    def split(self, b, floor, fp, fc, tp):
        sel = (fp[:, 0] > b['x0'] - .05) & (fp[:, 0] < b['x1'] + .05) & (fp[:, 1] > b['y0'] - .08) & (fp[:, 1] < b['y0'] + .06) \
            & (fp[:, 2] > floor + .004) & (fp[:, 2] < b['top'] - .004)
        p, c = fp[sel], fc[sel]
        wide = (fp[:, 0] > b['x0'] - .16) & (fp[:, 0] < b['x1'] + .16) & (fp[:, 1] > b['y0'] - .08) & (fp[:, 1] < b['y0'] + .08) \
            & (fp[:, 2] > floor + .004) & (fp[:, 2] < b['top'] - .004)
        pw, cw = fp[wide], fc[wide]
        edges = np.arange(floor, b['top'] + 1e-6, BIN)
        nb = len(edges) - 1
        if nb < 2 * MIN_LAYER / BIN or len(p) < 80:
            return None
        idx = np.clip(((p[:, 2] - floor) / BIN).astype(int), 0, nb - 1)
        cnt = np.bincount(idx, minlength=nb)
        ymed = np.full(nb, np.nan)
        for i in np.nonzero(cnt >= 8)[0]:
            ymed[i] = np.median(p[idx == i, 1])
        # 경계 후보 점수: 앞면 단차 + 턱
        score = np.zeros(nb + 1)
        zl = np.zeros(nb + 1)
        for k in range(int(MIN_LAYER / BIN), nb - int(MIN_LAYER / BIN) + 1):
            lo, hi = ymed[max(0, k - 4):k], ymed[k:k + 4]
            lo, hi = lo[np.isfinite(lo)], hi[np.isfinite(hi)]
            if len(lo) >= 2 and len(hi) >= 2:
                step = abs(np.median(hi) - np.median(lo))
                if step >= Y_STEP:
                    score[k] += min(step / Y_STEP, 3)
            zk = edges[k]
            led = (np.abs(tp[:, 2] - zk) < .009) & (tp[:, 0] > b['x0'] - .07) & (tp[:, 0] < b['x1'] + .07) \
                & (tp[:, 1] > b['y0'] - .06) & (tp[:, 1] < b['y1'] + .02)
            if led.sum() >= LEDGE_N:
                score[k] += 1.5
                zl[k] = float(np.median(tp[led, 2]))
        # 점수가 가장 높은 경계부터, 층 두께를 지키며 최대 두 개
        cuts = []
        for k in np.argsort(-score):
            if score[k] < 1 or len(cuts) == 2:
                break
            z = zl[k] if zl[k] else edges[k]
            if z - floor < MIN_LAYER or b['top'] - z < MIN_LAYER or any(abs(z - q) < MIN_LAYER for q in cuts):
                continue
            cuts.append(z)
        if not cuts:
            return None
        cuts.sort()
        bounds = [floor] + cuts + [b['top']]
        layers = []
        for j in range(len(bounds) - 1):
            z0, z1 = bounds[j], bounds[j + 1]
            last = j == len(bounds) - 2
            m = (p[:, 2] > z0 + .01) & (p[:, 2] < z1 - .01)
            col = np.median(c[m], axis=0) / 255. if m.sum() > 20 else np.asarray(b.get('color', (.6, .5, .4)), float)
            if last:                                     # 맨 위 층 = 윗면으로 맞춘 원래 상자
                x0, x1, y0 = b['x0'], b['x1'], b['y0']
                col = np.asarray(b.get('color', col), float)
            else:
                bins = [i for i in range(int((z0 - floor) / BIN), min(nb, int(np.ceil((z1 - floor) / BIN)))) if cnt[i] >= 8]
                if len(bins) < 2:
                    x0, x1, y0 = b['x0'], b['x1'], b['y0']
                else:
                    lo = [np.percentile(p[idx == i, 0], 5) for i in bins]
                    hi = [np.percentile(p[idx == i, 0], 95) for i in bins]
                    x0, x1 = float(np.median(lo)), float(np.median(hi))
                    y0 = float(np.nanmedian(ymed[bins]))
                # 이 층 윗면 턱이 보이면 거기까지 넓힌다
                led = (np.abs(tp[:, 2] - z1) < .009) & (tp[:, 0] > b['x0'] - .07) & (tp[:, 0] < b['x1'] + .07) \
                    & (tp[:, 1] > b['y0'] - .06) & (tp[:, 1] < b['y1'] + .02)
                if led.sum() >= LEDGE_N:
                    x0 = min(x0, float(np.percentile(tp[led, 0], 5)))
                    x1 = max(x1, float(np.percentile(tp[led, 0], 95)))
            if x1 - x0 < .04:
                x0, x1 = b['x0'], b['x1']
            layers.append(dict(x0=x0, x1=x1, y0=min(y0, b['y1'] - .05), y1=b['y1'], bottom=z0, top=z1 - (0 if last else .002),
                               color=np.clip(col, 0, 1), kind='box'))
        # 맨 아래 층 옆에 따로 놓인 짐
        low = layers[0]
        for sgn in (1, -1):
            out = (pw[:, 0] > low['x1'] + .025) if sgn > 0 else (pw[:, 0] < low['x0'] - .025)
            m = out & (pw[:, 2] < low['top'] - .01)
            if m.sum() < 40:
                continue
            q = pw[m]
            xs = np.percentile(q[:, 0], [5, 95]); zs = np.percentile(q[:, 2], [3, 97])
            if xs[1] - xs[0] < .035 or zs[1] - zs[0] < .04 or xs[1] - xs[0] > SIDE_MAX[0] or zs[1] - floor > SIDE_MAX[1]:
                continue
            y0 = float(np.median(q[:, 1]))
            if sgn > 0:
                x0, x1 = low['x1'] + .003, float(xs[1])
            else:
                x0, x1 = float(xs[0]), low['x0'] - .003
            layers.append(dict(x0=x0, x1=x1, y0=y0, y1=min(b['y1'], y0 + .2), bottom=floor, top=float(zs[1]),
                               color=np.clip(np.median(cw[m], axis=0) / 255., 0, 1), kind='side'))
        return layers

    # ------------------------------------------------------------ 매 프레임
    def update(self, frame, boxes):
        """boxes = cargo_model 상자 목록. (화면에 낼 상자 목록, 받침면 높이 또는 None)"""
        t0 = time.perf_counter()
        hdr = frame['hdr']
        self.memory.saving = 'replay' not in (hdr.get('src') or ())
        stamp = (hdr.get('session'), hdr.get('run'))
        if self.stamp is not None and (stamp != self.stamp[0] or (hdr.get('t_run') or 0) < self.stamp[1]):
            self.reset()
        self.stamp = (stamp, hdr.get('t_run') or 0)
        boxes = list(boxes or [])
        if 'depth' not in frame or 'surface' not in hdr:
            return boxes, self.floor
        if self.floor is None:                            # 상자 후보가 없어도 판 높이를 안다 (낮은 상자는 후보가 안 되어 받침면을 못 정하고, 그러면 상자를 못 찾았다)
            f = plate_floor(frame)
            if f is not None:
                self.floors.append(f)
                if len(self.floors) >= 3:
                    self.floor = float(np.median(self.floors))
        t_now = float(hdr.get('t_run') or 0)
        sens = hdr.get('sensors') or {}
        load = sens.get('total_g')
        belts = (sens.get('c1_N'), sens.get('c2_N'))
        if not boxes:                                     # 후보가 없어도 있던 물건은 그대로 (없어졌는지는 높이맵으로)
            self.ms = (time.perf_counter() - t0) * 1000
            return self.memory.update([], frame, t_now, self.floor, load, belts), self.floor
        fp, fc, tp, ap, ac = self.cloud(frame)
        solid = [b for b in boxes if b.get('shape') != 'cylinder' and b['top'] - b['bottom'] >= .06 and (b['y0'] + b['y1']) / 2 <= PLATE_Y]
        if solid:
            f = self.find_floor(tp, min(b['top'] for b in solid))
            if f is not None:
                self.floors.append(f)
        if len(self.floors) >= 3:
            f = float(np.median(self.floors))
            if self.floor is None or abs(f - self.floor) > MOVE:
                self.floor = f
        out = []
        # 화물 덩어리(가장 넓은 단단한 상자) 하나를 '쌓인 화물' 로 본다. cargo_model 의 상자 번호는 프레임마다
        # 서로 바뀌므로(1 <-> 2) 번호가 아니라 이 자리로 기록을 쌓는다. 그 위에 얹힌 것(덮개 · 벨트 뭉치 · 중복 추적)은 뺀다.
        main = max(solid, key=lambda q: (q['x1'] - q['x0']) * (q['y1'] - q['y0'])) if solid else None
        for b in boxes:
            if b is main:
                continue
            if main is not None and _overlap(b, main) > .3 and b['top'] <= main['top'] + .04:
                continue                                  # 화물 위에 얹힌 덮개 · 벨트 뭉치 · 같은 상자를 두 번 잡은 것
            nb = dict(b)
            if self.floor is not None and b.get('shape') != 'cylinder' and b['top'] - b['bottom'] >= .06 \
                    and b['bottom'] - self.floor < .25:
                nb['bottom'] = self.floor                 # 판 위에 따로 놓인 상자: 판에서 시작
                nb['kind'] = 'box'
            out.append(nb)
        if main is not None:
            if self.floor is None or main['bottom'] - self.floor < .05:
                out.insert(0, main)
            else:
                self.hist['main'].append(self.split(main, self.floor, fp, fc, tp))
                layers = self.stable('main', main)
                if not layers and self.carry and _overlap(main, self.carry[-1]) > .5 and abs(main['top'] - self.carry[-1]['top']) < .04:
                    layers = [dict(q) for q in self.carry[0]]  # 런이 바뀐 직후: 직전 층 모양을 빌려 쓰고 맨 위 층만 지금 상자로
                    tops = [q for q in layers if q['kind'] == 'box']
                    for k in ('x0', 'x1', 'y0', 'y1', 'top'):
                        tops[-1][k] = main[k]
                if not layers:
                    nb = dict(main); nb['bottom'] = self.floor; nb['kind'] = 'box'
                    out.insert(0, nb)
                else:
                    n_box = sum(q['kind'] == 'box' for q in layers)
                    for j, L in enumerate(layers):
                        nb = dict(L)
                        nb['id'] = j + 1                  # 아래 층 1, 위 층 2, 옆 짐 그다음 (런이 바뀌어도 같다)
                        nb['held'] = main.get('held', False)
                        nb['shape'] = 'box'
                        if j == n_box - 1 and nb['kind'] == 'box':
                            nb['src'] = main              # 맨 위 층: 원래 상자의 경고 표시를 넘긴다
                        out.insert(j, nb)
        # 쌓인 상자 윗면에 얹힌 얇은 것(열린 덮개 · 감아 둔 벨트)은 화물이 아니고 매번 모양이 바뀌어 깜빡인다 -> 뺀다
        stack = [b for b in out if b.get('kind') == 'box']
        out = [b for b in out if not (b.get('kind') is None and b['top'] - b['bottom'] < .06
                                      and any(_overlap(b, q) > .3 and b['bottom'] >= q['top'] - .03 for q in stack))]
        side = [dict(q, id=800 + i) for i, q in enumerate(q for q in out if q.get('kind') == 'side')]   # 옆 짐은 800 번대 (물건 번호와 안 겹치게)
        cands = self.bottles(merge_strips([q for q in out if q.get('kind') != 'side'], frame))
        out = self.memory.update(cands, frame, t_now, self.floor, load, belts) + side
        if ITEMS and self.floor is not None:
            out += self.track_items(self.find_items(ap, ac, out, self.floor))
        self.ms = (time.perf_counter() - t0) * 1000
        return out, self.floor

    @staticmethod
    def bottles(out):
        """페트병 모양(또는 뚜껑 색으로 찾은 병)은 kind 'bottle' 로 (화면이 상자 대신 병으로 그린다). 쌓은 층 · 옆 짐은 그대로.
        병 한쪽을 따로 잡은 작은 상자 (투명한 몸통과 뚜껑 쪽이 따로 맞춰진 것): 병과 같은 받침, 병 폭과 비슷하고
        병 긴 쪽 줄 위에 겹치거나 10 cm 안으로 붙어 있으면 병에 합친다."""
        res = []
        for b in out:
            nb = b
            if b.get('kind') in (None, 'box') and bottle_kind(b):
                nb = dict(b)
                nb['kind'] = 'bottle'
            res.append(nb)
        return [q for q in res if not StackLayers.merge_into_bottle(q, [p for p in res if p.get('kind') == 'bottle'])]

    @staticmethod
    def merge_into_bottle(q, bottles):
        if q.get('kind') not in (None, 'box') or max(q['x1'] - q['x0'], q['y1'] - q['y0']) >= .16:
            return False
        for p in bottles:
            if abs(q['bottom'] - p['bottom']) > .02:
                continue
            along_y = (p['y1'] - p['y0']) >= (p['x1'] - p['x0'])
            a0, a1, c0, c1 = ('x0', 'x1', 'y0', 'y1') if along_y else ('y0', 'y1', 'x0', 'x1')   # a = 가로 (병 폭), c = 병 길이 방향
            cross = max(0.0, min(q[a1], p[a1]) - max(q[a0], p[a0])) / max(1e-6, min(q[a1] - q[a0], p[a1] - p[a0]))
            gap = max(q[c0] - p[c1], p[c0] - q[c1], 0.0)
            if cross > .5 and gap < .10 and (q[a1] - q[a0]) < (p[a1] - p[a0]) * 1.6:
                for k in ('x0', 'y0', 'bottom'):
                    p[k] = min(p[k], q[k])
                for k in ('x1', 'y1', 'top'):
                    p[k] = max(p[k], q[k])
                return True
        return False

    # ------------------------------------------------------------ 작은 물건
    def find_items(self, ap, ac, solids, floor):
        """상자(층 · 옆 짐)로 설명되지 않는 점들을 판 위 1.5 cm 칸으로 묶어 작은 물건을 찾는다.
        걸러 내는 것: 공중에 떠 있는 점(늘어진 벨트), 얇은 판(상자 모서리 · 벨트 면), 상자에 붙어 높이까지 같은 것(상자 맞춤 오차)."""
        m = ITEM_MARGIN
        keep = ap[:, 2] > floor + .012
        for o in solids:
            keep &= ~((ap[:, 0] > o['x0'] - m) & (ap[:, 0] < o['x1'] + m) & (ap[:, 1] > o['y0'] - m) & (ap[:, 1] < o['y1'] + m)
                      & (ap[:, 2] > o['bottom'] - m) & (ap[:, 2] < o['top'] + m))
        q, c = ap[keep], ac[keep]
        if len(q) < ITEM_MIN_N:
            return []
        cell = .015
        org = np.array([-.6, -.2])
        ij = np.floor((q[:, :2] - org) / cell).astype(int)
        ok = (ij >= 0).all(1) & (ij < 80).all(1)
        q, c, ij = q[ok], c[ok], ij[ok]
        cnt = np.zeros((80, 80), int)
        np.add.at(cnt, (ij[:, 1], ij[:, 0]), 1)
        found = []
        for g in components(cnt >= 2):
            lab = np.zeros(cnt.shape, bool)
            lab[g[:, 0], g[:, 1]] = True
            sel = lab[ij[:, 1], ij[:, 0]]
            if sel.sum() < ITEM_MIN_N:
                continue
            p = q[sel]
            lo, hi = np.percentile(p, [4, 96], axis=0)
            cx, cy = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2
            # 받침: 판 또는 그 아래에 있는 상자 윗면
            under = [o['top'] for o in solids if o['x0'] - .01 < cx < o['x1'] + .01 and o['y0'] - .01 < cy < o['y1'] + .01 and o['top'] <= lo[2] + .03]
            support = max([floor] + under)
            if lo[2] - support > .03:                   # 떠 있음 = 늘어진 벨트 · 손
                continue
            if hi[2] - support < ITEM_MIN or min(hi[0] - lo[0], hi[1] - lo[1]) < ITEM_MIN:
                continue
            if max(hi[0] - lo[0], hi[1] - lo[1]) > ITEM_MAX:
                continue
            item = dict(x0=float(lo[0]), x1=float(hi[0]), y0=float(lo[1]), y1=float(hi[1]))
            glued = False
            for o in solids:
                gap = max(o['x0'] - item['x1'], item['x0'] - o['x1'], o['y0'] - item['y1'], item['y0'] - o['y1'])
                if gap < .02 and hi[2] > o['top'] - .03 and abs(support - o['bottom']) < .03:
                    glued = True                       # 상자 옆에 붙어 상자 높이까지 = 상자 맞춤 오차
                    break
                if _overlap(item, o) > .3 and abs(support - o['bottom']) < .03 and o.get('kind') in ('side', 'item'):
                    glued = True                       # 이미 옆 짐으로 나온 것
                    break
            if glued:
                continue
            col = np.clip(np.median(c[sel], axis=0) / 255., 0, 1)
            if col.max() < ITEM_DARK:                   # 검은 벨트 (색이 없는 점은 회색 0.47 이라 남는다)
                continue
            item.update(bottom=float(support), top=float(hi[2]), color=col, kind='item')
            found.append(item)
        return found

    def track_items(self, found):
        """작은 물건은 가끔만 잡힌다 -> 자리로 묶어 최근 ITEM_HIST 프레임 중 ITEM_SHOW 번 넘게 보이면 낸다 (중앙값 크기)."""
        for t in self.items:
            t['hits'].append(0)
        for f in found:
            cx, cy = (f['x0'] + f['x1']) / 2, (f['y0'] + f['y1']) / 2
            best = None
            for t in self.items:
                d = math.hypot(cx - t['cx'], cy - t['cy'])
                if d < .05 and abs(f['bottom'] - t['bottom']) < .04 and (best is None or d < best[0]):
                    best = (d, t)
            if best is None:
                t = dict(cx=cx, cy=cy, bottom=f['bottom'], seen=collections.deque(maxlen=ITEM_HIST),
                         hits=collections.deque([0] * (ITEM_HIST - 1), maxlen=ITEM_HIST), shown=False, id=900 + self.next_item)
                t['hits'].append(0)
                self.next_item += 1
                self.items.append(t)
            else:
                t = best[1]
            if t['hits'][-1]:
                continue
            t['hits'][-1] = 1
            t['seen'].append(f)
            t['cx'], t['cy'] = cx * .2 + t['cx'] * .8, cy * .2 + t['cy'] * .8
        out = []
        for t in list(self.items):
            n = sum(t['hits'])
            if n == 0:
                self.items.remove(t)
                continue
            t['shown'] = n >= (ITEM_KEEP if t['shown'] else ITEM_SHOW)
            if not t['shown'] or not t['seen']:
                continue
            seen = list(t['seen'])
            it = dict(seen[-1])
            for k in ('x0', 'x1', 'y0', 'y1', 'bottom', 'top'):
                it[k] = float(np.median([s[k] for s in seen]))
            it['color'] = np.median([s['color'] for s in seen], axis=0)
            it['id'] = t['id']
            it['shape'] = 'box'
            it['held'] = False
            out.append(it)
        return out

    def stable(self, bid, b):
        """층: 최근 결과에서 가장 흔한 층 수를 고르고 그 결과들의 중앙값. 옆 짐: 최근 35 % 넘게 보이면 낸다.
        화면 값은 1 cm 넘게 바뀔 때만 바꾼다."""
        hist = self.hist[bid]
        if len(hist) < 3:
            return self.shown.get(bid)
        h = [x for x in hist if x]
        if len(h) < len(hist) * .4:
            self.shown.pop(bid, None)
            return None
        prev = self.shown.get(bid) or []
        stacks = [[q for q in x if q['kind'] == 'box'] for x in h]
        sig = collections.Counter(len(s) for s in stacks)
        best, n = sig.most_common(1)[0]
        prev_n = sum(q['kind'] == 'box' for q in prev)
        if prev_n and prev_n != best and (n < len(h) * SWITCH or sig.get(prev_n, 0) >= n * .6):
            best = prev_n                                   # 새 층 수가 최근의 SWITCH 넘게 이어져야 바꾼다 (손 · 가림에 안 흔들리게)
        same = [s for s in stacks if len(s) == best]
        med = []
        for j in range(best):
            L = dict(same[0][j])
            for k in ('x0', 'x1', 'y0', 'y1', 'bottom', 'top'):
                L[k] = float(np.median([s[j][k] for s in same]))
            L['color'] = np.median([s[j]['color'] for s in same], axis=0)
            med.append(L)
        # 옆 짐은 잘 안 움직이고 가끔만 잡힌다 -> 런 동안 길게 기억 (최근 SIDE_HIST 프레임 중 3 번 넘게)
        cx = (b['x0'] + b['x1']) / 2
        latest = hist[-1] or []
        for right in (False, True):
            got = [q for q in latest if q['kind'] == 'side' and ((q['x0'] + q['x1']) / 2 > cx) == right]
            sh = self.side_hist[right]
            sh.append(got[0] if got else None)
            sides = [q for q in sh if q]
            had = any(q['kind'] == 'side' and ((q['x0'] + q['x1']) / 2 > cx) == right for q in prev)
            if len(sides) >= (SIDE_KEEP if had else SIDE_SHOW):
                L = dict(sides[-1])
                for k in ('x0', 'x1', 'y0', 'y1', 'bottom', 'top'):
                    L[k] = float(np.median([q[k] for q in sides]))
                L['color'] = np.median([q['color'] for q in sides], axis=0)
                med.append(L)
        if prev and [q['kind'] for q in prev] == [q['kind'] for q in med]:
            for L, P in zip(med, prev):
                for k in ('x0', 'x1', 'y0', 'y1', 'bottom', 'top'):
                    if abs(L[k] - P[k]) <= MOVE:
                        L[k] = P[k]
        self.shown[bid] = med
        top = [L for L in med if L['kind'] == 'box']
        if len(top) >= 2:
            self.carry = (med, top[-1])
        return med
