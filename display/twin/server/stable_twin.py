"""고정 기준 모델 + 지속되는 변화만 반영하는 안정 트윈 (순수 numpy).

카메라와 화물이 같은 카트에 실려 있으므로, 카메라 영상 좌표에서 화물 표면은 원래 고정이다.
  1) 기준: 런 시작 BUILD_N 프레임의 픽셀별 중앙값 깊이 + 평균 색 = 화면에 그리는 기본 형상
  2) 진동 보정: 매 프레임 현재 깊이를 기준에 맞춰 정수 픽셀 이동(±SHIFT px)과 깊이 오프셋을 찾는다
  3) 변화 확정: 기준과 다른 픽셀이 CONFIRM 프레임(0.5 s) 넘게 계속 다를 때만 그 픽셀을 현재 깊이로 바꾼다
  4) 흡수: ADOPT 프레임(3 s) 넘게 이어진 변화는 기준에 넣는다 (예: 떨어진 뒤의 새 화물 상태)
순간 노이즈 · 가림 · 마운트 진동은 형상을 바꾸지 않는다. 확정된 변화는 지어낸 값이 아니라 그 순간의 실측 깊이다.
"""
import numpy as np

BUILD_N = 10        # 기준을 만들 프레임 수 (10 Hz 면 1초)
SHIFT = 8           # 진동 보정 탐색 범위(px). 0.6 m 거리에서 1 px ≈ 3 mm. 거칠게(2px) 찾고 ±1px 다듬는다
FREEZE = .35        # 한 프레임에 기준 픽셀의 이 비율 넘게 다르면 화물 변화가 아니라 카메라(거치대) 흔들림으로 본다
FREEZE_MAX = 15     # 단, 흔들림은 금방 끝난다. 이만큼(1.5 s) 넘게 이어지는 큰 차이는 실제 변화로 보고 판정을 재개한다
CONFIRM = 5         # 이만큼 연속으로 다르면 변화로 확정 (0.5 s)
ADOPT = 30          # 이만큼 이어진 변화는 기준에 흡수 (3 s)
FADE = 30           # 흡수된 자리를 강조하는 프레임 수
MIN_DIFF_MM = 25    # 변화 판정 깊이 차 (거리의 4% 와 큰 쪽)


def neighbours(mask):
    """3x3 이웃 개수 (자기 포함)."""
    p = np.pad(mask.astype(np.uint8), 1)
    h, w = mask.shape
    return sum(p[dy:dy + h, dx:dx + w] for dy in range(3) for dx in range(3))


def shifted(img, dy, dx):
    """out[y, x] = img[y + dy, x + dx], 밖은 0."""
    out = np.zeros_like(img)
    h, w = img.shape[:2]
    ys, yd = (slice(dy, h), slice(0, h - dy)) if dy >= 0 else (slice(0, h + dy), slice(-dy, h))
    xs, xd = (slice(dx, w), slice(0, w - dx)) if dx >= 0 else (slice(0, w + dx), slice(-dx, w))
    out[yd, xd] = img[ys, xs]
    return out


class StableTwin:
    def __init__(self):
        self.reset()

    def reset(self):
        self.buf_d, self.buf_c, self.buf_a = [], [], []
        self.ref = None
        self._had_conf = False
        self.last = None
        self.freeze_run = 0
        self.info = dict(state='building', built=0)

    @property
    def ready(self):
        return self.ref is not None

    def _build(self):
        st = np.stack(self.buf_d)                       # (N, H, W) mm, 0 = 없음
        valid = st > 0
        frac = valid.mean(0)
        srt = np.sort(np.where(valid, st, np.inf), axis=0)
        cnt = valid.sum(0)
        med = np.take_along_axis(srt, np.clip((cnt - 1) // 2, 0, len(st) - 1)[None], 0)[0]
        self.ref = np.where(frac >= .5, med, 0).astype(np.float32)
        cs = np.stack(self.buf_c).astype(np.float32)    # 깊이가 있던 프레임의 색만 평균
        w = valid[..., None].astype(np.float32)
        self.ref_col = (cs * w).sum(0) / np.maximum(w.sum(0), 1)
        self.g0 = np.median(self.buf_a, 0) if self.buf_a else None
        self.cnt = np.zeros(self.ref.shape, np.int16)
        self.fade = np.zeros(self.ref.shape, np.int16)
        self.buf_d, self.buf_c = [], []
        self.ref_pixels = int((self.ref > 0).sum())

    def register(self, d):
        """현재 깊이를 기준에 맞추는 (dy, dx, 깊이 오프셋). 성긴 격자에서 절삭 평균 오차 최소."""
        s, step = SHIFT, 4
        r = self.ref[s:-s:step, s:-s:step]
        rv = r > 0
        need = .3 * rv.sum()

        def err(dy, dx):
            c = d[s + dy:d.shape[0] - s + dy:step, s + dx:d.shape[1] - s + dx:step]
            both = rv & (c > 0)
            if both.sum() < need:
                return np.inf
            return np.minimum(np.abs(c[both] - r[both]), 60).mean() + .3 * (abs(dy) + abs(dx))

        best = min((err(dy, dx), dy, dx) for dy in range(-s, s + 1, 2) for dx in range(-s, s + 1, 2))
        _, by, bx = best
        best = min((err(dy, dx), dy, dx) for dy in range(max(-s, by - 1), min(s, by + 1) + 1)
                   for dx in range(max(-s, bx - 1), min(s, bx + 1) + 1))
        if not np.isfinite(best[0]):
            best = (np.inf, 0, 0)
        _, dy, dx = best
        c = shifted(d, dy, dx)
        both = (self.ref > 0) & (c > 0)
        off = float(np.clip(np.median(c[both] - self.ref[both]), -40, 40)) if both.any() else 0.0
        return dy, dx, off, best[0]

    def update(self, depth, rgb, accel):
        """반환: (보여줄 깊이 mm, 보여줄 색 0..255 float, 형상이 바뀌었나)."""
        d = depth.astype(np.float32)
        if self.ref is None:
            self.buf_d.append(d); self.buf_c.append(rgb)
            if accel:
                self.buf_a.append(np.asarray(accel, np.float32))
            self.info = dict(state='building', built=len(self.buf_d))
            if len(self.buf_d) < BUILD_N:
                return None
            self._build()
            self.info = dict(state='ready')
            self.last = (self.ref, self.ref_col)
            return self.ref, self.ref_col, True
        dy, dx, off, err = self.register(d)
        cur = shifted(d, dy, dx)
        cur = np.where(cur > 0, cur - off, 0)
        col = shifted(rgb, dy, dx)
        ref = self.ref
        thr = np.maximum(MIN_DIFF_MM, .04 * ref)
        both = (ref > 0) & (cur > 0)
        changed = (both & (np.abs(cur - ref) > thr)) | ((ref > 0) & (cur == 0)) | ((ref == 0) & (cur > 0))
        inst = float(changed[ref > 0].mean()) if self.ref_pixels else 0.0
        self.freeze_run = self.freeze_run + 1 if inst > FREEZE else 0
        if 0 < self.freeze_run <= FREEZE_MAX and self.last is not None:   # 거치대 흔들림: 판정을 멈추고 직전 화면 유지
            self.info = dict(self.info, state='ready', shaking=True, shift=(dy, dx), inst_pct=100 * inst)
            return self.last[0], self.last[1], False
        self.cnt = np.where(changed, np.minimum(self.cnt + 1, 1000), np.maximum(self.cnt - 2, 0)).astype(np.int16)
        confirmed = self.cnt >= CONFIRM
        confirmed &= neighbours(confirmed) >= 5           # 외딴 픽셀(가장자리 결측)은 제외
        adopt = confirmed & (self.cnt >= ADOPT)
        geom = bool(confirmed.any() or adopt.any() or self._had_conf)
        if adopt.any():
            ref[adopt] = cur[adopt]
            self.ref_col[adopt] = col[adopt]
            self.cnt[adopt] = 0
            self.fade[adopt] = FADE
            confirmed &= ~adopt
        self._had_conf = bool(confirmed.any())
        self.fade = np.maximum(self.fade - 1, 0)
        show = ref.copy()
        show[confirmed] = cur[confirmed]
        show_col = self.ref_col.copy()
        show_col[confirmed] = col[confirmed] * .45 + np.array([255, 40, 30]) * .55
        f = (self.fade > 0)
        if f.any():
            a = (self.fade[f] / FADE * .5)[:, None]
            show_col[f] = show_col[f] * (1 - a) + np.array([255, 170, 40]) * a
        raw_noise = float(np.median(np.abs(cur[both] - ref[both]))) if both.any() else 0.0
        self.info = dict(state='ready', shift=(dy, dx), offset=off, noise_mm=raw_noise,
                         changed_pct=100.0 * confirmed.sum() / max(self.ref_pixels, 1),
                         pending_pct=100.0 * ((self.cnt > 0) & ~confirmed).sum() / max(self.ref_pixels, 1),
                         adopted=int(adopt.sum()), fading=bool(f.any()))
        self.info['shaking'] = False
        self.info['inst_pct'] = 100 * inst
        self.last = (show, show_col)
        return show, show_col, geom or bool(f.any())


def tilt_rotation(g0, g, R):
    """카트 기울기: 카메라 가속도(위 방향)가 기준 g0 에서 g 로 바뀐 만큼, 월드 좌표 회전 (각도°, 축)."""
    a = np.asarray(g, float); a0 = np.asarray(g0, float)
    a /= np.linalg.norm(a); a0 /= np.linalg.norm(a0)
    axis = np.cross(a, a0)                              # 카트가 Q 만큼 기울면 카메라에서 본 중력은 Q^-1 로 돈다
    s = np.linalg.norm(axis)
    if s < 1e-6:
        return 0.0, np.array([1.0, 0, 0])
    ang = float(np.degrees(np.arctan2(s, np.dot(a, a0))))
    return ang, np.asarray(R, float) @ (axis / s)


class StableHeight:
    """2.5D 높이맵(칸별 높이 m, 빈 칸 NaN)용 같은 규칙: 기준 고정 + 지속 변화만 반영 + 거치대 흔들림 동결."""
    MIN_DIFF_M = .025

    def __init__(self):
        self.reset()

    def reset(self):
        self.buf_h, self.buf_c, self.buf_a = [], [], []
        self.ref = None
        self.last = None
        self.freeze_run = 0
        self.info = dict(state='building', built=0)

    def update(self, h, col, accel):
        if self.ref is not None:
            h = np.where(h < self.floor, np.nan, h).astype(np.float32)
        if self.ref is None:
            self.buf_h.append(h.astype(np.float32)); self.buf_c.append(col.astype(np.float32))
            if accel:
                self.buf_a.append(np.asarray(accel, np.float32))
            self.info = dict(state='building', built=len(self.buf_h))
            if len(self.buf_h) < BUILD_N:
                return None
            st = np.stack(self.buf_h)
            valid = np.isfinite(st)
            srt = np.sort(np.where(valid, st, np.inf), axis=0)
            cnt = valid.sum(0)
            med = np.take_along_axis(srt, np.clip((cnt - 1) // 2, 0, len(st) - 1)[None], 0)[0]
            self.ref = np.where(valid.mean(0) >= .5, med, np.nan).astype(np.float32)
            z = self.ref[np.isfinite(self.ref) & (self.ref > .10)]           # 방바닥 위 10 cm 미만 = 화물이 아닌 잡음
            self.floor = max(.10, float(np.percentile(z, 8))) if len(z) else .10   # 적재면 높이 (이보다 낮은 칸은 버린다)
            self.ref[self.ref < self.floor] = np.nan
            w = valid[..., None].astype(np.float32)
            self.ref_col = (np.stack(self.buf_c) * w).sum(0) / np.maximum(w.sum(0), 1)
            self.g0 = np.median(self.buf_a, 0) if self.buf_a else None
            self.cnt = np.zeros(self.ref.shape, np.int16)
            self.fade = np.zeros(self.ref.shape, np.int16)
            self.ref_cells = int(np.isfinite(self.ref).sum())
            self.buf_h, self.buf_c = [], []
            self.info = dict(state='ready')
            self.last = (self.ref.copy(), self.ref_col.copy())
            return self.last[0], self.last[1], True
        ref = self.ref
        rv, cv = np.isfinite(ref), np.isfinite(h)
        both = rv & cv
        diff = np.where(both, np.abs(np.nan_to_num(h) - np.nan_to_num(ref)), 0)
        # 카메라가 조금 기울면 높이 차가 큰 가장자리가 한두 칸 밀린다 -> 기준의 3x3 이웃 중 가장 가까운 값과 비교
        hp = np.pad(np.where(cv, h, np.nan), 1, constant_values=np.nan)
        rp = np.pad(ref, 1, constant_values=np.nan)
        ny, nx = ref.shape
        near = np.full(ref.shape, np.inf, np.float32)
        for dy in range(3):
            for dx in range(3):
                d = np.abs(h - rp[dy:dy + ny, dx:dx + nx])
                near = np.fmin(near, np.where(np.isfinite(d), d, np.inf))
        rn = np.zeros(ref.shape, bool); cn = np.zeros(ref.shape, bool)
        for dy in range(3):
            for dx in range(3):
                rn |= np.isfinite(rp[dy:dy + ny, dx:dx + nx]); cn |= np.isfinite(hp[dy:dy + ny, dx:dx + nx])
        changed = (cv & rn & (near > self.MIN_DIFF_M)) | (rv & ~cn) | (cv & ~rn)
        inst = float(changed[rv].mean()) if self.ref_cells else 0.0
        self.freeze_run = self.freeze_run + 1 if inst > FREEZE else 0
        if 0 < self.freeze_run <= FREEZE_MAX and self.last is not None:
            self.info = dict(self.info, state='ready', shaking=True, inst_pct=100 * inst)
            return self.last[0], self.last[1], False
        self.cnt = np.where(changed, np.minimum(self.cnt + 1, 1000), np.maximum(self.cnt - 2, 0)).astype(np.int16)
        confirmed = self.cnt >= CONFIRM
        confirmed &= neighbours(confirmed) >= 4
        adopt = confirmed & (self.cnt >= ADOPT)
        if adopt.any():
            ref[adopt] = h[adopt]
            self.ref_col[adopt] = col[adopt]
            self.cnt[adopt] = 0
            self.fade[adopt] = FADE
            confirmed &= ~adopt
            self.ref_cells = int(np.isfinite(ref).sum())
        self.fade = np.maximum(self.fade - 1, 0)
        show = ref.copy(); show[confirmed] = h[confirmed]
        show_col = self.ref_col.copy()
        show_col[confirmed] = col[confirmed] * .45 + np.array([255, 40, 30]) * .55
        f = self.fade > 0
        if f.any():
            a = (self.fade[f] / FADE * .5)[:, None]
            show_col[f] = show_col[f] * (1 - a) + np.array([255, 170, 40]) * a
        self.info = dict(state='ready', shaking=False, inst_pct=100 * inst,
                         noise_mm=float(np.median(diff[both]) * 1000) if both.any() else 0.0,
                         changed_pct=100.0 * confirmed.sum() / max(self.ref_cells, 1),
                         pending_pct=100.0 * ((self.cnt > 0) & ~confirmed).sum() / max(self.ref_cells, 1),
                         shift=(0, 0))
        geom = bool(confirmed.any() or adopt.any() or f.any() or getattr(self, '_had', False))
        self._had = bool(confirmed.any() or f.any())
        self.last = (show, show_col)
        return show, show_col, geom
