"""디지털트윈 처리 코어 (순수 numpy — PC/라즈베리파이 공용, pyrealsense2 불필요).

입력: depth 프레임(uint16, 1 unit = depth_scale m), depth intrinsics(dict), IMU accel 평균
출력: 월드좌표 3D 점, 2.5D 높이맵, (비교용) 3D TSDF 볼륨
"""
import numpy as np


def world_rotation(accel_mean):
    """카메라좌표(x右, y下, z前) -> 월드좌표(z=위). 정지 상태 accel 은 '위' 방향을 가리킨다."""
    up = np.asarray(accel_mean, float); up /= np.linalg.norm(up)
    x = np.array([1.0, 0, 0]) - up[0] * up
    x /= np.linalg.norm(x)
    return np.stack([x, np.cross(up, x), up])        # 행벡터 = 월드 축 (x, y, up)


def deproject(depth_u16, intr, scale, max_range=3.0):
    """픽셀 (u,v,d) -> 카메라좌표 (X,Y,Z)[m].  X=(u-cx)Z/fx, Y=(v-cy)Z/fy.  D435 depth 는 왜곡계수 0."""
    h, w = depth_u16.shape
    u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    z = depth_u16.astype(np.float32) * scale
    valid = (z > 0) & (z < max_range)
    x = (u - intr["ppx"]) * z / intr["fx"]
    y = (v - intr["ppy"]) * z / intr["fy"]
    return np.stack([x[valid], y[valid], z[valid]], 1), valid


class HeightMapper:
    """고정 카메라용 2.5D 높이맵. 픽셀별 광선(ray)을 월드좌표로 미리 계산해 두고
    매 프레임은 depth 곱셈 + 격자 max 만 한다."""

    def __init__(self, intr, scale, R, bounds, cell=0.01, ref_h=0.0, max_range=3.0):
        h, w = intr["height"], intr["width"]
        u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
        rays_cam = np.stack([(u - intr["ppx"]) / intr["fx"], (v - intr["ppy"]) / intr["fy"],
                             np.ones_like(u)], -1).reshape(-1, 3)
        self.rays = (rays_cam @ np.asarray(R, np.float32).T).astype(np.float32)   # (N,3) 월드 광선
        self.scale, self.cell, self.ref_h, self.max_range = scale, cell, ref_h, max_range
        (self.x0, self.x1), (self.y0, self.y1) = bounds
        self.nx = int(np.ceil((self.x1 - self.x0) / cell))
        self.ny = int(np.ceil((self.y1 - self.y0) / cell))

    def __call__(self, depth_u16):
        z = depth_u16.reshape(-1).astype(np.float32) * self.scale
        ok = (z > 0) & (z < self.max_range)
        p = self.rays[ok] * z[ok, None]                           # 월드좌표 점
        ix = ((p[:, 0] - self.x0) / self.cell).astype(np.int32)
        iy = ((p[:, 1] - self.y0) / self.cell).astype(np.int32)
        inb = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        flat = iy[inb] * self.nx + ix[inb]
        hm = np.full(self.nx * self.ny, -np.inf, np.float32)
        np.maximum.at(hm, flat, p[inb, 2] - self.ref_h)           # 셀별 최대 높이
        hm[np.isinf(hm)] = np.nan
        return hm.reshape(self.ny, self.nx)


def voxel_downsample(pts, voxel=0.01):
    """3D 점을 voxel 격자로 줄임 (3D 포인트클라우드 경로 비교용)."""
    k = np.floor(pts / voxel).astype(np.int32)
    _, idx = np.unique(k, axis=0, return_index=True)
    return pts[idx]


class TSDF:
    """아주 단순한 TSDF 누적 (3D 재구성 경로의 비용 측정용). Open3D ScalableTSDF 와 같은 계열 연산."""

    def __init__(self, intr, scale, origin, size, voxel, trunc=0.04):
        n = np.round(np.asarray(size) / voxel).astype(int)
        g = np.stack(np.meshgrid(*[np.arange(k, dtype=np.float32) for k in n], indexing="ij"), -1)
        self.pts = (g.reshape(-1, 3) * voxel + np.asarray(origin, np.float32))   # 카메라좌표 voxel 중심
        self.tsdf = np.ones(len(self.pts), np.float32)
        self.w = np.zeros(len(self.pts), np.float32)
        self.intr, self.scale, self.trunc = intr, scale, trunc
        z = self.pts[:, 2]
        self.u = np.round(self.pts[:, 0] * intr["fx"] / z + intr["ppx"]).astype(np.int32)
        self.v = np.round(self.pts[:, 1] * intr["fy"] / z + intr["ppy"]).astype(np.int32)
        self.inimg = (self.u >= 0) & (self.u < intr["width"]) & (self.v >= 0) & (self.v < intr["height"])
        self.n = n

    def integrate(self, depth_u16):
        m = self.inimg
        d = depth_u16[self.v[m], self.u[m]].astype(np.float32) * self.scale
        sdf = d - self.pts[m, 2]
        upd = (d > 0) & (sdf > -self.trunc)
        idx = np.flatnonzero(m)[upd]
        t = np.minimum(1.0, sdf[upd] / self.trunc)
        w = self.w[idx]
        self.tsdf[idx] = (self.tsdf[idx] * w + t) / (w + 1)
        self.w[idx] = np.minimum(w + 1, 50)
