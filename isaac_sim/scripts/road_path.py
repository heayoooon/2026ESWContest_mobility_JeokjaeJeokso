"""Closed four-lane road loop: centreline sampled by arc length with heading and curvature.

The loop is a rounded rectangle (LOOP_LENGTH x LOOP_WIDTH between centre-line extents, LOOP_RADIUS corners; 400 x
260 m, 60 m since 2026-11-01 - the user found 200 x 120 m too cramped to drive freely), about 1220 m per lap, traversed counter-clockwise (interior on the +d side). Since 2026-11-01 it runs round a paved test area: all of
the interior and an APRON outside the loop are asphalt, so the truck can turn off the loop at any moment (the S-bend
on the top straight was dropped so the interior stays convex and the apron ribbon cannot fold).
Frenet coordinates: s = distance along the centreline (wraps), d = lateral offset (+ = left of travel).
"""
import math

import numpy as np

LANES = 4
LANE_W = 3.5
WIDTH = LANES * LANE_W          # 14 m of asphalt
SHOULDER = 1.5
RIGHT_LANE = -LANE_W * 1.5      # centre of the outer right lane (right-hand traffic)
INNER_RIGHT_LANE = -LANE_W * .5
LOOP_LENGTH, LOOP_WIDTH, LOOP_RADIUS = 400., 260., 60.
APRON = 40.                     # paved strip outside the loop's outer edge (m): room to turn right off the loop
PAVED_OUT = WIDTH / 2 + APRON   # paved half-width on the outer side of the centreline


def _line(p0, p1, step):
    n = max(2, int(math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / step))
    t = np.linspace(0, 1, n, endpoint=False)
    return np.column_stack([p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t])


def _arc(c, r, a0, a1, step):
    n = max(2, int(abs(a1 - a0) * r / step))
    a = np.linspace(a0, a1, n, endpoint=False)
    return np.column_stack([c[0] + r * np.cos(a), c[1] + r * np.sin(a)])


def raw_centreline(length=LOOP_LENGTH, width=LOOP_WIDTH, radius=LOOP_RADIUS, step=.5, wiggle=0.):
    hx, hy, R = length / 2, width / 2, radius
    parts = [_line((-hx + R, -hy), (hx - R, -hy), step),
             _arc((hx - R, -hy + R), R, -np.pi / 2, 0, step),
             _line((hx, -hy + R), (hx, hy - R), step),
             _arc((hx - R, hy - R), R, 0, np.pi / 2, step)]
    # Top straight (travelling -x) with an S-bend that starts and ends tangent to the straight.
    n = int((length - 2 * R) / step)
    u = np.linspace(0, 1, n, endpoint=False)
    xs = hx - R - u * (length - 2 * R)
    ys = hy + wiggle * np.sin(2 * np.pi * u) * np.sin(np.pi * u) ** 2
    parts += [np.column_stack([xs, ys]),
              _arc((-hx + R, hy - R), R, np.pi / 2, np.pi, step),
              _line((-hx, hy - R), (-hx, -hy + R), step),
              _arc((-hx + R, -hy + R), R, np.pi, 3 * np.pi / 2, step)]
    return np.concatenate(parts)


class Road:
    def __init__(self, step=.5, **kw):
        pts = raw_centreline(step=step, **kw)
        seg = np.diff(np.vstack([pts, pts[:1]]), axis=0)
        cum = np.concatenate([[0.], np.cumsum(np.hypot(seg[:, 0], seg[:, 1]))])
        self.length = float(cum[-1])
        self.s = np.arange(0., self.length, step)
        closed = np.vstack([pts, pts[:1]])
        self.x = np.interp(self.s, cum, closed[:, 0])
        self.y = np.interp(self.s, cum, closed[:, 1])
        dx = np.gradient(np.concatenate([self.x[-3:], self.x, self.x[:3]]))[3:-3]
        dy = np.gradient(np.concatenate([self.y[-3:], self.y, self.y[:3]]))[3:-3]
        self.theta = np.unwrap(np.arctan2(dy, dx))
        k = np.gradient(np.concatenate([self.theta[-3:] - 2 * np.pi, self.theta, self.theta[:3] + 2 * np.pi]))[3:-3] / step
        self.kappa = np.convolve(np.pad(k, 8, mode='wrap'), np.ones(17) / 17, mode='same')[8:-8]
        self.step = step
        L, Wd, R = (kw.get(k, d) for k, d in (('length', LOOP_LENGTH), ('width', LOOP_WIDTH), ('radius', LOOP_RADIUS)))
        self.start_s = (L - 2 * R) / 2                                         # middle of the bottom straight
        arc = math.pi * R / 2                                                  # where each corner begins
        self.corner_s = [L - 2 * R, L - 2 * R + arc + Wd - 2 * R, 2 * (L - 2 * R) + 2 * arc + Wd - 2 * R,
                         2 * (L - 2 * R) + 3 * arc + 2 * (Wd - 2 * R)]
        self._coarse = np.column_stack([self.x, self.y])[::4]                  # for distance queries (2 m)
        self._coarse_theta = self.theta[::4]

    # ------------------------------------------------------------------ queries
    def index(self, s):
        return (np.asarray(s) % self.length) / self.step

    def pose(self, s):
        """(x, y, heading rad, curvature) at arc length s (wraps)."""
        i = self.index(s)
        i0 = int(i) % len(self.s)
        i1 = (i0 + 1) % len(self.s)
        u = float(i - int(i))
        th0, th1 = self.theta[i0], self.theta[i1]
        if th1 - th0 > np.pi:
            th1 -= 2 * np.pi
        elif th0 - th1 > np.pi:
            th1 += 2 * np.pi
        return (self.x[i0] + (self.x[i1] - self.x[i0]) * u, self.y[i0] + (self.y[i1] - self.y[i0]) * u,
                th0 + (th1 - th0) * u, self.kappa[i0] + (self.kappa[i1] - self.kappa[i0]) * u)

    def project(self, x, y):
        """World point -> (s, d, heading rad) of the nearest centreline point (d + = inside the loop)."""
        j = int(np.argmin((self._coarse[:, 0] - x) ** 2 + (self._coarse[:, 1] - y) ** 2))
        idx = np.arange(4 * j - 6, 4 * j + 7) % len(self.s)
        k = int(idx[np.argmin((self.x[idx] - x) ** 2 + (self.y[idx] - y) ** 2)])
        th = float(self.theta[k])
        dx, dy = x - self.x[k], y - self.y[k]
        return (float(self.s[k]) + math.cos(th) * dx + math.sin(th) * dy) % self.length, -math.sin(th) * dx + math.cos(th) * dy, th

    def frenet_to_xy(self, s, d):
        x, y, th, _ = self.pose(s)
        return np.array([x - math.sin(th) * d, y + math.cos(th) * d]), th

    def offsets(self, s_values, d_values):
        """Vectorised Frenet -> world for arrays of (s, d)."""
        i = (np.asarray(s_values) % self.length) / self.step
        i0 = i.astype(int) % len(self.s)
        th = self.theta[i0]
        return np.column_stack([self.x[i0] - np.sin(th) * d_values, self.y[i0] + np.cos(th) * d_values])

    def distance(self, X, Y, chunk=20000):
        """Distance from arbitrary points to the centreline, and signed side (+ = inside the loop)."""
        X, Y = np.asarray(X, float).ravel(), np.asarray(Y, float).ravel()
        out = np.empty(len(X))
        side = np.empty(len(X))
        px, py = self._coarse[:, 0], self._coarse[:, 1]
        for a in range(0, len(X), chunk):
            xs, ys = X[a:a + chunk], Y[a:a + chunk]
            d2 = (xs[:, None] - px[None, :]) ** 2 + (ys[:, None] - py[None, :]) ** 2
            j = d2.argmin(1)
            out[a:a + chunk] = np.sqrt(d2[np.arange(len(xs)), j])
            th = self._coarse_theta[j]
            side[a:a + chunk] = np.sign(-np.sin(th) * (xs - px[j]) + np.cos(th) * (ys - py[j]))
        return out, side

    def bounds(self, margin=0.):
        return (self.x.min() - margin, self.x.max() + margin, self.y.min() - margin, self.y.max() + margin)
