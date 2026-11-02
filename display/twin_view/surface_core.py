"""Measured RGB-D surfaces. Pure numpy; no invented back faces or dependencies."""
import numpy as np


def rays(intr):
    y, x = np.mgrid[:intr['height'], :intr['width']].astype(np.float32)
    return np.stack(((x-intr['ppx'])/intr['fx'], (y-intr['ppy'])/intr['fy'], np.ones_like(x)), -1)


def surface_indices(shape):
    ny, nx = shape
    y, x = np.mgrid[:ny-1, :nx-1]
    a = (y*nx+x).ravel()
    return np.stack((np.stack((a,a+1,a+nx),1), np.stack((a+1,a+nx+1,a+nx),1)),1).reshape(-1,3).astype(np.uint32)


def valid_triangles(points, valid, triangles, threshold=0.025):
    p = points.reshape(-1,3)
    triangles = triangles[valid.ravel()[triangles].all(1)]
    keep = np.ones(len(triangles), dtype=bool)
    # Check actual 3D edge lengths, not world height: vertical box faces are real.
    for a,b in ((0,1),(1,2),(2,0)):
        delta = p[triangles[:,a]]-p[triangles[:,b]]
        keep &= np.einsum('ij,ij->i',delta,delta) < threshold**2
    return np.ascontiguousarray(triangles[keep].ravel())


class DepthSmoother:
    """Smooth only sub-centimetre jitter; removal/motion is immediate. No hole persistence."""
    def __init__(self):
        self.previous = None

    def update(self, depth):
        out = depth.astype(np.float32)
        if self.previous is not None and self.previous.shape == out.shape:
            stable = (out > 0) & (self.previous > 0) & (np.abs(out-self.previous) < 8)
            out[stable] = .65*out[stable]+.35*self.previous[stable]
        self.previous = out
        return out
