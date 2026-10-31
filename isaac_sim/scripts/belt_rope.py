"""Over-the-top lashing belts as ropes: geometry of a taut belt pulled over the cargo between two anchors.

A belt lies in a vertical plane across the plate (rig x = BELT_X[i]); its ends are anchored beside the plate
(A on the tension-cell side y = -ANCHOR_Y, B on the other side y = +ANCHOR_Y, at plate height). Pulled taut it
follows the upper convex hull of the cargo's cross-section in that plane, so its length grows when the cargo
lifts, tips or slides sideways. Tension is that of a rope: preload plus stiffness x stretch, never negative -
a slack belt carries nothing and a belt can only pull. On the cargo it acts where it leaves the cargo, along
the free legs towards the anchors (down and inwards).
"""
import math

import numpy as np

BELT_X = (-.18, .18)          # front belt c1 (N side), rear belt c2 (S side), rig frame (m, real cart)
ANCHOR_Y = .225               # anchors just outside the plate edge
ANCHOR_Z = .20                # belt hooks on the side of the plate, below its top (.225): always the lowest points,
                             # so a cargo pushed past an anchor still wraps the belt over its corner
WIDTH = .032                  # webbing width (m)
CARGO_HALF = (.29, .19, .28)  # sandbox cargo body half extents (m, real cart); centre seats at z = .515


def anchors():
    return np.array([-ANCHOR_Y, ANCHOR_Z]), np.array([ANCHOR_Y, ANCHOR_Z])


def _hull(points):
    """Convex hull (Andrew's monotone chain), counter-clockwise, no repeated end point."""
    pts = sorted(set(map(tuple, np.round(points, 6))))
    if len(pts) <= 2:
        return [np.array(p) for p in pts]
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return [np.array(p) for p in lower[:-1] + upper[:-1]]


def upper_path(points_yz, a=None, b=None):
    """Taut path A -> over the cargo -> B in the belt plane: the arc of the convex hull of {A, B, cargo points}
    from A to B that passes over the top (it wraps corners that stick out past an anchor, too).
    Returns an (n, 2) array (y, z) from A to B; n == 2 means the belt touches nothing."""
    a, b = (np.asarray(a, float), np.asarray(b, float)) if a is not None else anchors()
    pts = np.asarray(points_yz, float).reshape(-1, 2)
    hull = _hull(np.vstack([pts, a, b]))
    key = lambda q: tuple(np.round(q, 6))
    keys = [key(q) for q in hull]
    if key(a) not in keys or key(b) not in keys:
        return np.array([a, b])
    ia, ib = keys.index(key(a)), keys.index(key(b))
    n = len(hull)
    ccw = [hull[(ia + k) % n] for k in range((ib - ia) % n + 1)]
    cw = [hull[(ia - k) % n] for k in range((ia - ib) % n + 1)]
    top = lambda arc: max((q[1] for q in arc[1:-1]), default=-1e9)
    path = ccw if top(ccw) >= top(cw) else cw
    if len(path) > 2 and top(path) <= max(a[1], b[1]) + 1e-4:
        return np.array([a, b])                    # nothing above the anchors: the belt lies flat
    return np.array(path)


def length(path):
    return float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))


def box_section(centre, rotation, half, x):
    """(y, z) corners of a box (rig frame centre, 3x3 rotation, half extents) if the belt plane x crosses it."""
    signs = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)], float)
    corners = np.asarray(centre) + (signs * np.asarray(half)) @ np.asarray(rotation).T
    if corners[:, 0].min() <= x <= corners[:, 0].max():
        return corners[:, 1:]
    return np.zeros((0, 2))


def seat_length(x, cargo_y=0.):
    """Belt length with the cargo seated (no rotation) - the length at which the belt carries its preload."""
    return length(upper_path(box_section((0., cargo_y, .515), np.eye(3), CARGO_HALF, x)))


def leg_forces(path, tension):
    """[(point_yz, force_yz)] the belt applies to the cargo: at the first and last contact point, along the
    free legs towards the anchors. Empty when the belt has no contact."""
    if len(path) <= 2 or tension <= 0:
        return []
    out = []
    for contact, anchor in ((path[1], path[0]), (path[-2], path[-1])):
        leg = anchor - contact
        n = np.linalg.norm(leg)
        if n > 1e-6:
            out.append((contact, tension * leg / n))
    return out


def ribbon(path, x, width=WIDTH, lift=.004):
    """Points / quads of a flat webbing strip along a path in the belt plane x (rig frame)."""
    pts = []
    for y, z in path:
        pts.append((x - width / 2, y, z + lift))
        pts.append((x + width / 2, y, z + lift))
    quads = [(2 * k, 2 * k + 1, 2 * k + 3, 2 * k + 2) for k in range(len(path) - 1)]
    return np.array(pts, np.float32), quads
