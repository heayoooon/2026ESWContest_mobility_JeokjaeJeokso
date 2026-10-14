"""카메라 앞면 깊이로 쌓인 화물을 층(상자)별로 나눈다.

cargo_model 은 위에서 보이는 윗면만 맞추므로, 상자 위에 상자를 쌓으면 '윗면 크기 x 바닥까지' 큰 상자 하나가 된다.
카메라는 화물 앞면(카메라 쪽 수직면)도 본다. 그 면에서
  - 앞면이 들어가거나 나온 곳 (위 상자와 아래 상자의 앞면 위치가 다르다)
  - 아래 상자 윗면이 위 상자 옆으로 살짝 보이는 턱 (수평면)
을 찾아 층 경계로 쓴다. 아래 층의 좌우 폭은 그 층 앞면 폭, 깊이(뒤쪽 끝)는 보이지 않으므로 위 상자와 같다고 둔다.
아래 층 옆에 따로 놓인 짐(뽁뽁이로 싼 물병 등)은 '옆 짐' 으로 하나 더 만든다.

출력은 cargo_model 상자와 같은 dict (x0 x1 y0 y1 bottom top color id shape held) + kind ('box' | 'side').
매 프레임 다시 재지만, 화면이 출렁이지 않게 최근 결과의 중앙값으로 고정하고 1 cm 넘게 바뀔 때만 움직인다.
"""
import collections
import math
import time

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


def _rays(intr):
    y, x = np.mgrid[:intr['height'], :intr['width']].astype(np.float32)
    return np.stack(((x - intr['ppx']) / intr['fx'], (y - intr['ppy']) / intr['fy'], np.ones_like(x)), -1)


def _overlap(a, b):
    ix = max(0.0, min(a['x1'], b['x1']) - max(a['x0'], b['x0']))
    iy = max(0.0, min(a['y1'], b['y1']) - max(a['y0'], b['y0']))
    return ix * iy / max(1e-6, (a['x1'] - a['x0']) * (a['y1'] - a['y0']))


class StackLayers:
    def __init__(self):
        self.ray_key = None
        self.side_hist = {k: collections.deque(maxlen=SIDE_HIST) for k in (False, True)}   # 옆 짐은 런이 바뀌어도 기억
        self.items = []          # 작은 물건 추적 (런이 바뀌어도 그대로)
        self.next_item = 0
        self.carry = None        # 마지막으로 낸 층 (런이 바뀌어 다시 잴 동안 한 덩어리로 보이지 않게)
        self.reset()

    def reset(self):
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
        z = top[:, 2]
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
        stamp = (hdr.get('session'), hdr.get('run'))
        if self.stamp is not None and (stamp != self.stamp[0] or (hdr.get('t_run') or 0) < self.stamp[1]):
            self.reset()
        self.stamp = (stamp, hdr.get('t_run') or 0)
        boxes = list(boxes or [])
        if 'depth' not in frame or 'surface' not in hdr or not boxes:
            return boxes, self.floor
        fp, fc, tp, ap, ac = self.cloud(frame)
        solid = [b for b in boxes if b.get('shape') != 'cylinder' and b['top'] - b['bottom'] >= .06]
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
        if ITEMS and self.floor is not None:
            out += self.track_items(self.find_items(ap, ac, out, self.floor))
        self.ms = (time.perf_counter() - t0) * 1000
        return out, self.floor

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
