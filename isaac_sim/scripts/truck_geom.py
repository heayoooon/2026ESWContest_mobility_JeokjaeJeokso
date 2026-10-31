"""Mesh generators for a more realistic vehicle: filleted boxes, lathe (revolved) solids such as tyres
and rims, and profiles swept along the road (guardrails). All meshes carry smooth per-vertex normals
where the surface is curved and world/object UVs for tiling materials.
"""
import math

import numpy as np
from pxr import Gf, Sdf, UsdGeom, Vt

from forest_scene import bind


def _fix_winding(points, counts, indices, normals):
    """Reverse any face whose geometric normal disagrees with its vertices' shading normals."""
    out = []
    pos = 0
    for c in counts:
        face = list(indices[pos:pos + c])
        pos += c
        if c >= 3 and normals is not None:
            a, b, d = points[face[0]], points[face[1]], points[face[2 if c == 3 else 2]]
            geo = np.cross(b - a, d - a)
            if np.dot(geo, normals[face].mean(0)) < 0:
                face.reverse()
        out.extend(face)
    return out


def _mesh(stage, path, points, counts, indices, mat, normals=None, uv=None, double_sided=False):
    points = np.asarray(points, float)
    if normals is not None:
        indices = _fix_winding(points, counts, indices, np.asarray(normals, float))
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(points, np.float32)))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([int(c) for c in counts]))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([int(i) for i in indices]))
    mesh.CreateSubdivisionSchemeAttr('none')
    if normals is not None:
        mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(normals, np.float32)))
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    if uv is not None:
        st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        st.Set(Vt.Vec2fArray.FromNumpy(np.ascontiguousarray(uv, np.float32)))
    if double_sided:
        mesh.CreateDoubleSidedAttr(True)
    bind(mesh.GetPrim(), mat)
    return mesh.GetPrim()


def rounded_box(stage, path, lo, hi, radius, mat, segments=4, uv_scale=1.):
    """Box from min/max corners with filleted edges (radius clamped to the smallest half-extent).

    Each of the six faces is a grid; vertices are pushed out from the inner (shrunken) box by the
    fillet radius so edges and corners become quarter-cylinders and eighth-spheres."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    size = hi - lo
    r = float(min(radius, size.min() / 2 - 1e-4))
    centre = (lo + hi) / 2
    inner = size / 2 - r
    n = max(2, int(segments))
    # Face grid parameter: dense near the edges (where the fillet lives), sparse in the flat middle.
    edge = np.linspace(0., 1., n + 1) ** 1.6
    t = np.unique(np.concatenate([-1 + edge * (2 * r / size.max()) * 1.0, 1 - edge * (2 * r / size.max()),
                                  np.linspace(-1, 1, 5)]))
    pts, nrm, uv, counts, idx = [], [], [], [], []
    axes = [(0, 1, 2), (1, 2, 0), (2, 0, 1)]
    for axis, (a, b, c) in enumerate(axes):
        for sign in (-1., 1.):
            base = len(pts)
            m = len(t)
            for i in range(m):
                for j in range(m):
                    p = np.zeros(3)
                    p[a] = sign
                    p[b] = t[i]
                    p[c] = t[j]
                    # Point on the unit cube face -> box surface: clamp to the inner box, push out by r.
                    q = p * size / 2
                    core = np.clip(q, -inner, inner)
                    d = q - core
                    norm = np.linalg.norm(d)
                    if norm < 1e-9:
                        d = np.zeros(3); d[a] = sign; norm = 1.
                    normal = d / norm
                    pts.append(centre + core + normal * r)
                    nrm.append(normal)
                    uv.append(((centre[b] + t[i] * size[b] / 2) / uv_scale, (centre[c] + t[j] * size[c] / 2) / uv_scale))
            for i in range(m - 1):
                for j in range(m - 1):
                    k = base + i * m + j
                    counts.append(4); idx.extend([k, k + m, k + m + 1, k + 1])   # winding fixed in _mesh
    return _mesh(stage, path, np.array(pts), counts, idx, mat, np.array(nrm), np.array(uv))


def lathe(stage, path, profile, mat, segments=64, radius_fn=None, uv_scale=1.):
    """Solid of revolution about the local z axis. profile = [(r, z), ...] from one end to the other;
    radius_fn(angle_index, segments, ring_index) -> multiplier lets tread blocks modulate the radius."""
    profile = np.asarray(profile, float)
    m = len(profile)
    pts, nrm, uv, counts, idx = [], [], [], [], []
    # Profile normals (2-D) from neighbouring segments.
    tang = np.gradient(profile, axis=0)
    pn = np.column_stack([tang[:, 1], -tang[:, 0]])
    pn /= np.maximum(np.linalg.norm(pn, axis=1, keepdims=True), 1e-9)
    for k in range(segments + 1):
        a = 2 * math.pi * (k % segments) / segments
        ca, sa = math.cos(a), math.sin(a)
        for i, (r, z) in enumerate(profile):
            rr = r * (radius_fn(k % segments, segments, i) if radius_fn else 1.)
            pts.append((rr * ca, rr * sa, z))
            nrm.append((pn[i, 0] * ca, pn[i, 0] * sa, pn[i, 1]))
            uv.append((a * r / uv_scale, z / uv_scale))
    for k in range(segments):
        for i in range(m - 1):
            p0 = k * m + i
            p1 = (k + 1) * m + i
            counts.append(4); idx.extend([p0, p1, p1 + 1, p0 + 1])
    # Caps where the profile does not reach the axis (cap normal along -z at the start, +z at the end).
    for end, ring in ((0, 0), (1, m - 1)):
        if profile[ring, 0] > 1e-6:
            cap_n = (0., 0., -1.) if profile[0, 1] < profile[-1, 1] else (0., 0., 1.)
            if end == 1:
                cap_n = tuple(-v for v in cap_n)
            c = len(pts)
            pts.append((0., 0., profile[ring, 1])); nrm.append(cap_n); uv.append((0, 0))
            for k in range(segments):
                p0 = k * m + ring
                p1 = (k + 1) * m + ring
                cp = len(pts); pts.append(pts[p0]); nrm.append(cap_n); uv.append(uv[p0])
                cq = len(pts); pts.append(pts[p1]); nrm.append(cap_n); uv.append(uv[p1])
                counts.append(3); idx.extend([c, cp, cq])
    return _mesh(stage, path, np.array(pts), counts, idx, mat, np.array(nrm), np.array(uv))


def tyre_profile(radius, width, rim_radius, shoulder=.035, sidewall_bulge=.012):
    """(r, z) profile of a truck tyre from bead to bead (z along the axle)."""
    hw = width / 2
    return [(rim_radius, -hw + .02), (rim_radius + .03, -hw + .005), (radius - .10, -hw + .0),
            (radius - .03, -hw + sidewall_bulge), (radius - .006, -hw + shoulder), (radius, -hw + shoulder + .02),
            (radius, 0.), (radius, hw - shoulder - .02), (radius - .006, hw - shoulder), (radius - .03, hw - sidewall_bulge),
            (radius - .10, hw), (rim_radius + .03, hw - .005), (rim_radius, hw - .02)]


def tread_modulation(blocks=48, groove_depth=.008, radius=.52):
    """Radius multiplier making transverse tread blocks around the circumference."""
    def fn(k, segments, ring):
        if ring not in (5, 6, 7):          # only the tread rings
            return 1.
        block = (k * blocks // segments) % 2
        return 1. - (groove_depth / radius if block else 0.)
    return fn


def rim_profile(radius, width, hub_radius=.14, dish=.06):
    """Chrome rim: outer lip, dished centre with a hub."""
    hw = width / 2
    return [(hub_radius, -hw + dish), (radius - .10, -hw + dish * .4), (radius - .03, -hw + .01), (radius, -hw),
            (radius, hw), (radius - .03, hw - .01), (radius - .10, hw - dish * .4), (hub_radius, hw - dish)]


def sweep_profile(stage, path, road, d, profile, mat, s_ranges=None, step=1., uv_scale=1.):
    """Sweep a 2-D cross-section (dy toward +d, dz up) along the road at lateral offset d."""
    profile = np.asarray(profile, float)
    m = len(profile)
    tang = np.gradient(profile, axis=0)
    pn = np.column_stack([tang[:, 1], -tang[:, 0]])
    pn /= np.maximum(np.linalg.norm(pn, axis=1, keepdims=True), 1e-9)
    pts, nrm, uv, counts, idx = [], [], [], [], []
    for a, b in (s_ranges if s_ranges is not None else [(0., road.length)]):
        ss = np.arange(a, b + 1e-6, step)
        if len(ss) < 2:
            continue
        base = len(pts)
        for s in ss:
            x, y, th, _ = road.pose(s)
            nx, ny = -math.sin(th), math.cos(th)      # left normal
            for i, (dy, dz) in enumerate(profile):
                pts.append((x + nx * (d + dy), y + ny * (d + dy), dz))
                nrm.append((nx * pn[i, 0], ny * pn[i, 0], pn[i, 1]))
                uv.append((s / uv_scale, (dy + dz) / uv_scale))
        for k in range(len(ss) - 1):
            for i in range(m - 1):
                p0 = base + k * m + i
                p1 = base + (k + 1) * m + i
                counts.append(4); idx.extend([p0, p0 + 1, p1 + 1, p1])
    return _mesh(stage, path, np.array(pts), counts, idx, mat, np.array(nrm), np.array(uv), double_sided=True)


def cylinder_between(stage, path, a, b, radius, mat, segments=12):
    """Thin cylinder from point a to point b (wires, pipes)."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    length = float(np.linalg.norm(d))
    prim = lathe(stage, path, [(radius, 0.), (radius, length)], mat, segments)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, a)))
    quat = Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.Vec3d(*map(float, d / max(length, 1e-9)))).GetQuat()
    xf.AddOrientOp().Set(Gf.Quatf(quat))
    return prim
