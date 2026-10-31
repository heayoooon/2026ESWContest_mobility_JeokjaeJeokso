"""Animated mesh of the D435i 2.5-D height map (one frame of twin.tws / live_cache npz).

Rig-local placement (see prepare_live_runs.py): grid y (camera forward, N->S) -> rig +x,
grid x -> rig +y, heights are mm above the room floor with the plate top 208 mm above it; the deck
top is at z = .225 in the rig frame.
"""
import json

import numpy as np
from pxr import Sdf, UsdGeom, UsdShade, Vt

from forest_scene import bind


def vertex_colour_material(stage, path='/World/Materials/MeasuredSurface'):
    """UsdPreviewSurface whose diffuse colour comes from the mesh's displayColor primvar
    (RTX ignores displayColor on an unbound mesh)."""
    mat = UsdShade.Material.Define(stage, path)
    reader = UsdShade.Shader.Define(stage, path + '/Reader')
    reader.CreateIdAttr('UsdPrimvarReader_float3')
    reader.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('displayColor')
    reader.CreateInput('fallback', Sdf.ValueTypeNames.Float3).Set((.5, .5, .5))
    out = reader.CreateOutput('result', Sdf.ValueTypeNames.Float3)
    shader = UsdShade.Shader.Define(stage, path + '/Shader')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(out)
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.85)
    shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(0.)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    return mat

PLATE_ABOVE_FLOOR = .208
DECK_TOP = .225
GRID_Y_TO_RIG_X = -.266     # grid y = 0.05 m sits 21.6 cm in front of the N cell row (x = -.196)
NODATA = -30000
DECK_COLOUR = (.50, .53, .52)


class MeasuredSurface:
    def __init__(self, stage, path, grid, mat=None):
        self.nx, self.ny, self.cell = int(grid['nx']), int(grid['ny']), float(grid['cell'])
        self.x0, self.y0 = float(grid['x0']), float(grid['y0'])
        gx = self.x0 + (np.arange(self.nx) + .5) * self.cell
        gy = self.y0 + (np.arange(self.ny) + .5) * self.cell
        self.rig_x = np.repeat(gy + GRID_Y_TO_RIG_X, self.nx)       # point index = iy * nx + ix
        self.rig_y = np.tile(gx, self.ny)
        self.mesh = UsdGeom.Mesh.Define(stage, path)
        ix, iy = np.meshgrid(np.arange(self.nx - 1), np.arange(self.ny - 1), indexing='xy')
        a = iy * self.nx + ix
        quads = np.stack([a, a + self.nx, a + self.nx + 1, a + 1], -1).reshape(-1, 4)   # +x then +y: normal +z
        self.mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4] * len(quads)))
        self.mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(quads.ravel().tolist()))
        self.mesh.CreateSubdivisionSchemeAttr('none')
        self.mesh.CreateDoubleSidedAttr(True)
        self.points = self.mesh.CreatePointsAttr()
        self.colour = UsdGeom.PrimvarsAPI(self.mesh).CreatePrimvar('displayColor', Sdf.ValueTypeNames.Color3fArray,
                                                                   UsdGeom.Tokens.vertex)
        bind(self.mesh.GetPrim(), mat if mat is not None else vertex_colour_material(stage))
        self.prim = self.mesh.GetPrim()
        self.last = None
        self.update(np.full((self.ny, self.nx), NODATA, np.int16), np.zeros((self.ny, self.nx, 3), np.uint8))

    def update(self, height_mm, colour, fill=None):
        """height_mm: (ny, nx) int16 with NODATA holes; colour: (ny, nx, 3) uint8."""
        h = np.asarray(height_mm, np.float32)
        valid = h > NODATA
        z = np.where(valid, h / 1000. - PLATE_ABOVE_FLOOR + DECK_TOP, DECK_TOP + .002)
        z = np.clip(z, DECK_TOP + .002, DECK_TOP + 1.2)
        points = np.stack([self.rig_x, self.rig_y, z.ravel()], -1).astype(np.float32)
        self.points.Set(Vt.Vec3fArray.FromNumpy(points))
        rgb = np.asarray(colour, np.float32) / 255.
        rgb = np.where(valid[..., None], rgb, np.array(DECK_COLOUR, np.float32))
        self.colour.Set(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(rgb.reshape(-1, 3), np.float32)))
        self.last = (height_mm, colour)
        self.top = float(z.max())

    def cargo_top(self):
        """Highest measured point (rig z), for fitting the straps over the real cargo."""
        return getattr(self, 'top', DECK_TOP)


class DepthSurface:
    """Textured surface rebuilt every frame from the D435i depth image (424x240, visible cargo only)
    and the depth-aligned colour JPEG: much finer than the 1 cm height map.

    Pixels are decimated by `step`; each kept pixel is a vertex whose position is the depth ray in the
    camera-world frame (same frame as the height-map grid). Pixels without depth are projected onto the
    plate plane and clamped to the ROI, so the fixed quad topology never shows holes."""

    def __init__(self, stage, path, camera, grid, step=2, mat=None):
        intr = camera['intr']
        self.step = step
        self.vs = np.arange(0, intr['height'], step)
        self.us = np.arange(0, intr['width'], step)
        self.set_camera(camera, grid)
        nu, nv = len(self.us), len(self.vs)
        self.mesh = UsdGeom.Mesh.Define(stage, path)
        iu, iv = np.meshgrid(np.arange(nu - 1), np.arange(nv - 1), indexing='xy')
        a = iv * nu + iu
        # grid x -> rig +y and grid y -> rig +x: (iu, iv) -> (iu, iv+1) -> (iu+1, iv+1) -> (iu+1, iv) faces +z
        quads = np.stack([a, a + nu, a + nu + 1, a + 1], -1).reshape(-1, 4)
        self.mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4] * len(quads)))
        self.mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(quads.ravel().tolist()))
        self.mesh.CreateSubdivisionSchemeAttr('none')
        self.mesh.CreateDoubleSidedAttr(True)
        self.points = self.mesh.CreatePointsAttr()
        self.colour = UsdGeom.PrimvarsAPI(self.mesh).CreatePrimvar('displayColor', Sdf.ValueTypeNames.Color3fArray,
                                                                   UsdGeom.Tokens.vertex)
        bind(self.mesh.GetPrim(), mat if mat is not None else vertex_colour_material(stage))
        self.prim = self.mesh.GetPrim()
        self.top = DECK_TOP
        self.update(np.zeros((intr['height'], intr['width']), np.uint16), None)

    def set_camera(self, camera, grid):
        """Per-run camera calibration (intrinsics, world rotation, floor reference)."""
        intr = camera['intr']
        R = np.asarray(camera['R'], np.float32)
        self.ref_h = float(camera['ref_h'])
        u, v = np.meshgrid(self.us.astype(np.float32), self.vs.astype(np.float32))
        rays = np.stack([(u - intr['ppx']) / intr['fx'], (v - intr['ppy']) / intr['fy'], np.ones_like(u)], -1)
        self.rays = (rays.reshape(-1, 3) @ R.T).astype(np.float32)          # camera-world rays (N, 3)
        plate_z = self.ref_h + PLATE_ABOVE_FLOOR
        t_plate = plate_z / np.where(np.abs(self.rays[:, 2]) < 1e-3, -1e-3, self.rays[:, 2])
        self.plate_hit = self.rays * np.clip(t_plate, 0., 5.)[:, None]
        x0, x1 = float(grid['x0']), float(grid['x0']) + grid['nx'] * float(grid['cell'])
        y0, y1 = float(grid['y0']), float(grid['y0']) + grid['ny'] * float(grid['cell'])
        self.plate_hit[:, 0] = np.clip(self.plate_hit[:, 0], x0, x1)
        self.plate_hit[:, 1] = np.clip(self.plate_hit[:, 1], y0, y1)
        self.plate_hit[:, 2] = plate_z
        self.roi = (x0, x1, y0, y1)

    def update(self, depth_mm, jpeg, blend=False):
        """blend: depth_mm is the next frame of the same recording, so average it with the previous one where
        the surface did not really move (the D435i's frame-to-frame noise made the cargo wobble)."""
        import cv2
        # 2026-11-01 (user: "the cargo looks warped, fix it a little"): median 5 instead of 3, flying-pixel removal
        # and the temporal blend below.
        depth = cv2.medianBlur(np.ascontiguousarray(depth_mm, np.uint16), 5).astype(np.float32)
        # Depth edges mix cargo and plate into in-between depths that stretch slanted sheets from the cargo top to
        # the plate; drop pixels whose 3x3 neighbourhood spans > 25 mm, so the hole filling gives vertical sides.
        has = depth > 0
        k = np.ones((3, 3), np.uint8)
        span = cv2.dilate(np.where(has, depth, 0.), k) - cv2.erode(np.where(has, depth, 65535.), k)
        depth[has & (span > 25.)] = 0.
        prev = getattr(self, 'prev_depth', None)
        if blend and prev is not None and prev.shape == depth.shape:
            both = (depth > 0) & (prev > 0) & (np.abs(depth - prev) < 20.)
            depth = np.where(both, .5 * depth + .5 * prev, depth)
        self.prev_depth = depth
        sub = depth[self.vs][:, self.us]
        d = sub.reshape(-1).astype(np.float32) / 1000.
        valid = d > 0
        p = self.rays * d[:, None]
        if valid.any() and not valid.all():
            # Pixels without depth take the x/y of the nearest cargo pixel at plate height, so the
            # cargo silhouette gets vertical sides instead of a tent stretched to the plate behind it.
            # Nearest pixel above the deck (not a plate pixel) since 2026-11-01: halves the slanted sheets again.
            zv = p[:, 2] - self.ref_h - PLATE_ABOVE_FLOOR + DECK_TOP
            src = valid & (zv > DECK_TOP + .01)
            if not src.any():
                src = valid
            _, labels = cv2.distanceTransformWithLabels((~src).reshape(sub.shape).astype(np.uint8), cv2.DIST_L2, 3,
                                                        labelType=cv2.DIST_LABEL_PIXEL)
            nearest = np.flatnonzero(src)[labels.reshape(-1) - 1]
            p[~valid, :2] = p[nearest[~valid], :2]
            p[~valid, 2] = self.ref_h + PLATE_ABOVE_FLOOR
        elif not valid.any():
            p = self.plate_hit.copy()
        z = p[:, 2] - self.ref_h - PLATE_ABOVE_FLOOR + DECK_TOP
        z = np.clip(z, DECK_TOP + .002, DECK_TOP + 1.2)
        pts = np.stack([p[:, 1] + GRID_Y_TO_RIG_X, p[:, 0], z], -1).astype(np.float32)
        self.points.Set(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(pts)))
        rgb = np.tile(np.array(DECK_COLOUR, np.float32), (len(d), 1))
        if jpeg is not None:
            img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                img = img[self.vs][:, self.us].reshape(-1, 3)[:, ::-1].astype(np.float32) / 255.
                img = img ** 2.2                                   # sRGB JPEG -> linear shading colour
                rgb = np.where(valid[:, None], img, rgb)
        self.colour.Set(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(rgb, np.float32)))
        self.top = float(z[valid].max()) if valid.any() else DECK_TOP
        self.cargo_points = pts[valid & (z > DECK_TOP + .01)]   # rig frame, for draping the belts

    def cargo_top(self):
        return self.top


def load_cache(path):
    data = np.load(path)
    grid = json.loads(str(data['grid']))
    stats = json.loads(str(data['stats']))
    out = {'t': data['t'], 'height': data['height'], 'colour': data['colour'], 'jpeg': data['jpeg'],
           'jpeg_offsets': data['jpeg_offsets'], 'grid': grid, 'stats': stats}
    if 'depth' in data.files:
        out['depth'] = data['depth']
        out['camera'] = json.loads(str(data['camera']))
    return out


def jpeg_bytes(cache, index):
    o = cache['jpeg_offsets']
    return cache['jpeg'][o[index]:o[index + 1]].tobytes()
