"""USD geometry for the forest test road, instrumented cart and operator.

Realistic mode (default when assets/nvidia exists, see fetch_nvidia_assets.py):
textured terrain and road, NVIDIA vegetation, HDRI sky and a skinned operator whose
skeleton is posed procedurally (leg gait + two-bone IK keeping both hands on the grip).
Without those assets the original procedural primitives are used.
"""
import math
import random
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, UsdSkel, Vt

import road_path

NVIDIA = Path(__file__).resolve().parents[1] / 'assets' / 'nvidia'
S3 = 'omniverse-content-production.s3-us-west-2.amazonaws.com'
TREES = {  # prototype: (relative weight, min scale, max scale); native units are centimetres
    # Each tree already instances 74-405 branch meshes internally, so the counts below
    # are kept modest for interactive frame rates on a 10 GB RTX 3080.
    'Red_Oak': (.10, .40, .62), 'Sugar_Maple': (.24, .70, 1.0), 'White_Ash': (.26, .85, 1.15),
    'Red_Maple': (.20, .80, 1.10), 'Yellow_Pine': (.20, .42, .60)}
TREE_ROWS = 2                                            # roadside rows per side along the ~560 m loop
COVER_NEAR, COVER_FAR, LITTER, ROCKS = 2200, 900, 1500, 220
# Barberry is deliberately excluded: its nested instancers turn the RTX frame black (NaN).
GROUND_COVER = ['Grass_Short_A', 'Grass_Short_B', 'Grass_Short_C', 'Switchgrass']
DEBRIS = ['oakfall1', 'oakfall2', 'maplefall1']
GRIP_X, GRIP_Y, GRIP_Z = .36, .205, .94  # cart-frame handle grip (see ForestScene.cart)


def veg(*parts):
    return NVIDIA / 'vegetation' / S3 / 'Assets' / 'Vegetation' / Path(*parts)


def base_mdl(group, name):
    return NVIDIA / 'materials' / S3 / 'Materials' / '2023_1' / 'Base' / group / f'{name}.mdl'


CHARACTER = (NVIDIA / 'people' / S3 / 'Assets/Isaac/4.2/Isaac/People/Characters/male_adult_construction_03'
             / 'male_adult_construction_03.usd')
SKY_CLOUDY = NVIDIA / 'sky' / 'kloofendal_48d_partly_cloudy_4k.hdr'
# clear noon sky (2026-11-01, user request "sunny weather"); the cloudy one stays as the fallback
SKY_CLEAR = NVIDIA / 'sky_clear' / 'noon_grass_4k.hdr'
SKY = SKY_CLEAR if SKY_CLEAR.exists() else SKY_CLOUDY


def realistic_assets_available():
    needed = [CHARACTER, SKY, base_mdl('Natural', 'Asphalt')] + [veg('Trees', f'{n}.usd') for n in TREES]
    return all(p.exists() for p in needed)


def pose(prim, position=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1)):
    xf = UsdGeom.Xformable(prim)
    ops = xf.GetOrderedXformOps()
    if not ops:
        ops = [xf.AddTranslateOp(), xf.AddRotateXYZOp(), xf.AddScaleOp()]
    values = (Gf.Vec3d(*map(float, position)), Gf.Vec3f(*map(float, rotation)), Gf.Vec3f(*map(float, scale)))
    for op, value in zip(ops, values):
        op.Set(value)


def material(stage, name, color, rough=.7, metallic=0):
    mat = UsdShade.Material.Define(stage, '/World/Materials/' + name)
    shader = UsdShade.Shader.Define(stage, mat.GetPath().AppendChild('Shader'))
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(rough)
    shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(metallic)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    return mat


def mdl_material(stage, name, mdl_path, inputs=None):
    """Material backed by one of the collected NVIDIA Base MDL presets."""
    mat = UsdShade.Material.Define(stage, '/World/Materials/' + name)
    shader = UsdShade.Shader.Define(stage, mat.GetPath().AppendChild('Shader'))
    shader.SetSourceAsset(Sdf.AssetPath(str(mdl_path).replace('\\', '/')), 'mdl')
    shader.SetSourceAssetSubIdentifier(Path(mdl_path).stem, 'mdl')
    for key, (type_name, value) in (inputs or {}).items():
        shader.CreateInput(key, type_name).Set(value)
    out = shader.CreateOutput('out', Sdf.ValueTypeNames.Token)
    for render_context in ('mdl', ''):
        (mat.CreateSurfaceOutput(render_context) if render_context else mat.CreateSurfaceOutput()).ConnectToSource(out)
    return mat


def bind(prim, mat):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)


def shape(stage, path, kind, pos, size, mat, rotation=(0, 0, 0), collision=False):
    cls = {'box': UsdGeom.Cube, 'sphere': UsdGeom.Sphere, 'cylinder': UsdGeom.Cylinder, 'capsule': UsdGeom.Capsule}[kind]
    geo = cls.Define(stage, path)
    if kind == 'box':
        geo.CreateSizeAttr(1.)
    elif kind == 'sphere':
        geo.CreateRadiusAttr(1.)
    else:
        geo.CreateRadiusAttr(1.)
        geo.CreateHeightAttr(1.)
    pose(geo.GetPrim(), pos, rotation, size)
    bind(geo.GetPrim(), mat)
    if collision:
        UsdPhysics.CollisionAPI.Apply(geo.GetPrim())
    return geo.GetPrim()


def link(stage, path, a, b, radius, mat):
    a, b = np.array(a), np.array(b)
    delta = b - a
    length = float(np.linalg.norm(delta))
    prim = shape(stage, path, 'sphere', (a + b) / 2, (radius, radius, length / 2 + radius), mat)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, (a + b) / 2)))
    quat = Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.Vec3d(*map(float, delta / max(length, 1e-6)))).GetQuat()
    xf.AddOrientOp().Set(Gf.Quatf(quat))
    xf.AddScaleOp().Set(Gf.Vec3f(radius, radius, length / 2 + radius))
    return prim


def move_link(prim, a, b, radius):
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b-a
    length = np.linalg.norm(d)
    ops = UsdGeom.Xformable(prim).GetOrderedXformOps()
    ops[0].Set(Gf.Vec3d(*((a+b)/2)))
    ops[1].Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0,0,1), Gf.Vec3d(*(d/max(length,1e-6)))).GetQuat()))
    ops[2].Set(Gf.Vec3f(radius, radius, float(length/2+radius)))


ROAD_EDGE = road_path.WIDTH / 2 + road_path.SHOULDER      # asphalt + gravel shoulder half-width (8.5 m)


def terrain_height(x, y, dist):
    """Ground height (m) from the distance to the road centreline: flat road and shoulders,
    a shallow ditch just past the shoulder, then a rolling forest floor."""
    x, y, dist = (np.asarray(a, float) for a in (x, y, dist))
    ditch = -.25 * np.exp(-((dist - (ROAD_EDGE + .9)) / .55) ** 2)
    rise = np.clip((dist - (ROAD_EDGE + .4)) / 4.5, 0, 1)
    rise = rise * rise * (3 - 2 * rise)
    rolling = (.55 * np.sin(.085 * x + 1.3) * np.cos(.11 * y + .4) + .3 * np.sin(.25 * x + .21 * y + 2.1)
               + .15 * np.sin(.6 * x - .5 * y) + .25 * np.sin(.045 * x + .07 * y))
    return np.where(dist < ROAD_EDGE + .3, 0., ditch + rise * (.3 + .03 * (dist - ROAD_EDGE) + .35 * rolling))


def mesh_from_quads(stage, path, points, quads, mat, uv, subsets=None, face_key=None):
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(points, np.float32)))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4] * len(quads)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(np.asarray(quads).ravel().tolist()))
    mesh.CreateSubdivisionSchemeAttr('none')
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set(Vt.Vec2fArray.FromNumpy(np.ascontiguousarray(uv, np.float32)))
    bind(mesh.GetPrim(), mat)
    if subsets:
        for name, (lo, hi, smat) in subsets.items():
            faces = np.nonzero((face_key >= lo) & (face_key < hi))[0]
            subset = UsdGeom.Subset.CreateGeomSubset(mesh, name, UsdGeom.Tokens.face, Vt.IntArray(faces.tolist()),
                                                     'materialBind', UsdGeom.Tokens.partition)
            bind(subset.GetPrim(), smat)
    return mesh


def grid_mesh(stage, path, xs, ys, mat, uv_tile, height, subsets=None, dist_fn=None):
    """Quad grid mesh with world-space UVs; height(x, y, dist) with dist from dist_fn(x, y)."""
    gx, gy = np.meshgrid(xs, ys, indexing='ij')
    dist = dist_fn(gx, gy).reshape(gx.shape) if dist_fn else np.zeros_like(gx)
    gz = height(gx, gy, dist)
    nx, ny = len(xs), len(ys)
    points = np.stack([gx, gy, gz], -1).reshape(-1, 3)
    idx = np.arange(nx * ny).reshape(nx, ny)
    # Counter-clockwise seen from above (xs and ys ascending), so faces point up.
    quads = np.stack([idx[:-1, :-1], idx[1:, :-1], idx[1:, 1:], idx[:-1, 1:]], -1).reshape(-1, 4)
    face_key = dist.reshape(-1)[quads].mean(1)
    return mesh_from_quads(stage, path, points, quads, mat, points[:, :2] / uv_tile, subsets, face_key)


def ribbon(stage, path, road, d0, d1, mat, z=0., uv_tile=3.5, step=1., s_ranges=None):
    """Strip following the road between lateral offsets d0..d1 (s along u, d along v)."""
    pts, quads, uv = [], [], []
    ranges = s_ranges if s_ranges is not None else [(0., road.length)]
    for a, b in ranges:
        ss = np.arange(a, b + 1e-6, step)
        if len(ss) < 2:
            continue
        left = road.offsets(ss, np.full(len(ss), d0))
        right = road.offsets(ss, np.full(len(ss), d1))
        base = len(pts)
        for k, s in enumerate(ss):
            pts.append((left[k, 0], left[k, 1], z)); uv.append((s / uv_tile, d0 / uv_tile))
            pts.append((right[k, 0], right[k, 1], z)); uv.append((s / uv_tile, d1 / uv_tile))
        for k in range(len(ss) - 1):
            i = base + 2 * k
            quads.append((i, i + 1, i + 3, i + 2) if d1 > d0 else (i + 1, i, i + 2, i + 3))
    return mesh_from_quads(stage, path, np.array(pts), np.array(quads), mat, np.array(uv))


def mat_to_quat(m):
    """Unit quaternion (w, x, y, z) of a column-convention rotation matrix."""
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.) * 2
        return (.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s)
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1. + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        return ((m[2, 1] - m[1, 2]) / s, .25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s)
    if m[1, 1] > m[2, 2]:
        s = math.sqrt(1. + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        return ((m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, .25 * s, (m[1, 2] + m[2, 1]) / s)
    s = math.sqrt(1. + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    return ((m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, .25 * s)


def axis_rotation(axis, angle):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + s * k + (1 - c) * k @ k


def rotation_between(a, b):
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if c < -.9999:
        perp = np.cross(a, [1, 0, 0] if abs(a[0]) < .9 else [0, 1, 0])
        return axis_rotation(perp, math.pi)
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k / (1 + c)


def two_bone_joint(root, target, l1, l2, pole):
    """Middle joint (knee/elbow) position for a two-bone chain reaching for target."""
    d_vec = target - root
    d = float(np.clip(np.linalg.norm(d_vec), abs(l1 - l2) + 1e-4, l1 + l2 - 1e-4))
    u = d_vec / max(np.linalg.norm(d_vec), 1e-6)
    w = pole - np.dot(pole, u) * u
    w = w / max(np.linalg.norm(w), 1e-6)
    cos_a = np.clip((l1 * l1 + d * d - l2 * l2) / (2 * l1 * d), -1, 1)
    return root + u * l1 * cos_a + w * l1 * math.sqrt(1 - cos_a * cos_a), root + u * d


class SkinnedOperator:
    """NVIDIA character driven by a procedural pose written to a UsdSkel.Animation.

    Skeleton space: metres, Z up, the character faces -Y and its left side is +X.
    The operator prim's local frame faces +X, so the body is rotated +90 deg about Z."""

    def __init__(self, stage, root='/World/Operator', outfit='operator'):
        self.prim = UsdGeom.Xform.Define(stage, root).GetPrim()
        pose(self.prim)
        body = UsdGeom.Xform.Define(stage, root + '/Body').GetPrim()
        pose(body, rotation=(0, 0, 90))
        char = stage.DefinePrim(root + '/Body/Character', 'Xform')
        char.GetReferences().AddReference(str(CHARACTER), '/Root/male_adult_construction_03')
        self.dress(stage, char, outfit)
        skel_prim = next(p for p in Usd.PrimRange(char) if p.IsA(UsdSkel.Skeleton))
        skel = UsdSkel.Skeleton(skel_prim)
        joints = skel.GetJointsAttr().Get()
        self.parents = list(UsdSkel.Topology(joints).GetParentIndices())
        rest = [np.array(m) for m in skel.GetRestTransformsAttr().Get()]
        self.rest_R = np.array([m[:3, :3].T for m in rest])
        self.rest_t = np.array([m[3, :3] for m in rest])
        n = len(joints)
        self.rest_Rg = np.empty((n, 3, 3))
        self.rest_tg = np.empty((n, 3))
        for j, p in enumerate(self.parents):
            self.rest_Rg[j] = self.rest_R[j] if p < 0 else self.rest_Rg[p] @ self.rest_R[j]
            self.rest_tg[j] = self.rest_t[j] if p < 0 else self.rest_Rg[p] @ self.rest_t[j] + self.rest_tg[p]
        name = {str(jt).split('/')[-1]: i for i, jt in enumerate(joints)}
        self.j = name
        self.legs = {side: (name[f'{s}_Thigh'], name[f'{s}_Calf'], name[f'{s}_Foot']) for side, s in ((1, 'L'), (-1, 'R'))}
        self.arms = {side: (name[f'{s}_Upperarm'], name[f'{s}_Forearm'], name[f'{s}_Hand']) for side, s in ((1, 'L'), (-1, 'R'))}
        self.spine = [name[k] for k in ('Waist', 'Spine01', 'Spine02')]
        # Grip: curl the four fingers toward the palm (palm faces -Z in the T pose).
        self.curl = {}
        for side, s in ((1, 'L'), (-1, 'R')):
            for finger in ('Index', 'Mid', 'Ring', 'Pinky'):
                for k, angle in ((1, 45), (2, 70), (3, 45)):
                    j = name.get(f'{s}_{finger}{k}')
                    if j is not None:
                        axis_local = self.rest_Rg[j].T @ np.array([0., side, 0.])
                        self.curl[j] = axis_rotation(axis_local, math.radians(angle))
        # Cap following the head joint; Body frame == skeleton space.
        cap_colour = (.018, .019, .022) if outfit == 'operator' else (.06, .10, .30)
        cap_mat = material(stage, 'CapFabric' + outfit.title(), cap_colour, .85)
        self.head = name['Head']
        self.cap = []
        for part, center, radii in (('CapCrown', (0, .028, 1.712), (.099, .106, .082)),
                                    ('CapBrim', (0, -.088, 1.712), (.082, .075, .007))):
            prim = UsdGeom.Sphere.Define(stage, f'{root}/Body/{part}').GetPrim()
            UsdGeom.Sphere(prim).CreateRadiusAttr(1.)
            xf = UsdGeom.Xformable(prim)
            ops = (xf.AddTranslateOp(), xf.AddOrientOp(), xf.AddScaleOp())
            ops[2].Set(Gf.Vec3f(*radii))
            bind(prim, cap_mat)
            self.cap.append((ops, np.array(center) - self.rest_tg[self.head]))
        self.anim = UsdSkel.Animation.Define(stage, root + '/Pose')
        self.anim.CreateJointsAttr(joints)
        self.anim.CreateScalesAttr(Vt.Vec3hArray([Gf.Vec3h(1, 1, 1)] * n))
        self.rotations = self.anim.CreateRotationsAttr()
        self.translations = self.anim.CreateTranslationsAttr()
        UsdSkel.BindingAPI.Apply(skel_prim).CreateAnimationSourceRel().SetTargets([self.anim.GetPath()])
        self.walk(0., 0., True)

    OUTFITS = {
        # Reference video: black top, grey trousers.
        'operator': (('opaque__fabric__shirt', (.028, .03, .034)), ('opaque__fabric__safetyvest', (.02, .021, .024)),
                     ('retro__reflective__safetyvest', (.02, .021, .024)), ('opaque__fabric__jeans', (.52, .53, .55)),
                     ('opaque__leather__workboots', (.04, .04, .045))),
        # Parcel courier: navy shirt and vest, dark trousers.
        'delivery': (('opaque__fabric__shirt', (.10, .16, .42)), ('opaque__fabric__safetyvest', (.07, .10, .28)),
                     ('retro__reflective__safetyvest', (.07, .10, .28)), ('opaque__fabric__jeans', (.16, .17, .20)),
                     ('opaque__leather__workboots', (.05, .05, .055))),
    }

    @classmethod
    def dress(cls, stage, char, outfit='operator'):
        """Retint the construction worker's clothes; no hard hat or ear defenders."""
        looks = char.GetPath().AppendChild('Looks')
        for look, tint in cls.OUTFITS[outfit]:
            shader = UsdShade.Shader(stage.GetPrimAtPath(looks.AppendChild(look).AppendChild('Shader')))
            if shader:
                shader.CreateInput('diffuse_tint', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*tint))
        for prim in Usd.PrimRange(char):
            if prim.GetName() in ('opaque__plastic__hardhat', 'opaque__plastic__earprotection'):
                UsdGeom.Imageable(prim).MakeInvisible()

    def sit(self, seat, steer_deg=0.):
        """Driving pose. seat (operator frame: x forward, y left, z up, origin = root prim):
        hip_z, knee (x, z), pedal (x, z), wheel (x, z), wheel_r, wheel_tilt_deg, hands_deg.
        The steering wheel rotation moves both hands around the rim."""
        R = self.rest_R.copy()
        t = self.rest_t.copy()
        Rg = np.empty_like(self.rest_Rg)
        tg = np.empty_like(self.rest_tg)
        hip = self.j['Hip']
        hip_offset = np.array([0., -seat.get('hip_x', .0), seat['hip_z'] - self.rest_tg[hip][2]])
        thighs = (self.legs[1][0], self.legs[-1][0])
        # Hands on the rim: grip angle from the top, rotated with the wheel (operator frame -> skel).
        tilt = math.radians(seat['wheel_tilt_deg'])
        wx, wz = seat['wheel'][0], seat['wheel'][1]
        wy = seat.get('wheel_y', 0.)
        hands = {}
        for side in (1, -1):
            a = math.radians(side * seat['hands_deg'] + steer_deg)
            # rim point in the wheel plane (u = lateral, w = up along the tilted plane)
            u, w = seat['wheel_r'] * math.sin(a), seat['wheel_r'] * math.cos(a)
            fwd = wx - w * math.sin(tilt)
            up = wz + w * math.cos(tilt)
            hands[side] = np.array([wy + u, -fwd, up])          # skel: X=left, Y=-forward, Z=up
        pedals = {side: np.array([side * .14, -seat['pedal'][0], seat['pedal'][1]]) for side in (1, -1)}
        knees = {side: np.array([side * .14, -seat['knee'][0], seat['knee'][1]]) for side in (1, -1)}
        lean = math.radians(seat.get('lean_deg', 8.))
        self._sit_targets = {}
        for j, p in enumerate(self.parents):
            if j in self.curl:
                R[j] = R[j] @ self.curl[j]
            if p < 0:
                Rg[j], tg[j] = R[j], t[j]
            else:
                Rg[j] = Rg[p] @ R[j]
                tg[j] = Rg[p] @ t[j] + tg[p]
            if j == hip:
                tg[j] = tg[j] + hip_offset
                self._set(j, p, Rg[j], R, Rg, t, tg)
            elif j in self.spine:
                self._set(j, p, axis_rotation((1, 0, 0), -lean / 3) @ Rg[j], R, Rg, t, tg)
            elif j in thighs:
                side = 1 if j == self.legs[1][0] else -1
                calf, foot = self.legs[side][1], self.legs[side][2]
                knee, reach = two_bone_joint(tg[j], pedals[side], np.linalg.norm(self.rest_t[calf]),
                                             np.linalg.norm(self.rest_t[foot]), knees[side] - tg[j])
                self._aim(j, p, calf, knee, R, Rg, t, tg)
                self._sit_targets[calf] = reach
            elif j in (self.legs[1][1], self.legs[-1][1]):
                self._aim(j, p, self.legs[1 if j == self.legs[1][1] else -1][2], self._sit_targets[j], R, Rg, t, tg)
            elif j in (self.legs[1][2], self.legs[-1][2]):
                self._set(j, p, axis_rotation((1, 0, 0), math.radians(-25)) @ self.rest_Rg[j], R, Rg, t, tg)
            elif j in (self.arms[1][0], self.arms[-1][0]):
                side = 1 if j == self.arms[1][0] else -1
                fore, hand = self.arms[side][1], self.arms[side][2]
                wrist = hands[side] + .06 * (tg[j] - hands[side]) / np.linalg.norm(tg[j] - hands[side])
                elbow, reach = two_bone_joint(tg[j], wrist, np.linalg.norm(self.rest_t[fore]),
                                              np.linalg.norm(self.rest_t[hand]), np.array([side * .6, .3, -1.]))
                self._sit_targets[fore] = reach
                self._aim(j, p, fore, elbow, R, Rg, t, tg)
            elif j in (self.arms[1][1], self.arms[-1][1]):
                self._aim(j, p, self.arms[1 if j == self.arms[1][1] else -1][2], self._sit_targets[j], R, Rg, t, tg)
        self.rotations.Set(Vt.QuatfArray([Gf.Quatf(*mat_to_quat(r)) for r in R]))
        self.translations.Set(Vt.Vec3fArray.FromNumpy(t.astype(np.float32)))
        delta = Rg[self.head] @ self.rest_Rg[self.head].T
        quat = Gf.Quatf(*mat_to_quat(delta))
        for ops, offset in self.cap:
            ops[0].Set(Gf.Vec3d(*map(float, tg[self.head] + delta @ offset)))
            ops[1].Set(quat)

    @staticmethod
    def offset(pull):
        """Operator root position along the cart X axis so the grip is within reach."""
        return GRIP_X + (.28 if pull else .42)

    def walk(self, phase, moving, pull=True):
        m = min(abs(moving), 1.)
        R = self.rest_R.copy()
        t = self.rest_t.copy()
        Rg = np.empty_like(self.rest_Rg)
        tg = np.empty_like(self.rest_tg)
        # Grip targets in skeleton space (operator frame: x forward, y left -> skel X=y, Y=-x).
        grip_f = (GRIP_X - self.offset(pull)) if pull else (self.offset(pull) - GRIP_X)
        grips = {side: np.array([side * GRIP_Y, -grip_f, GRIP_Z]) for side in (1, -1)}
        lean = math.radians(5 if pull else 9)
        hip_offset = np.array([-.018 * m * math.cos(phase), 0., -.035 + .014 * m * (abs(math.cos(phase)) - 1)])
        twist = math.radians(5) * m * math.sin(phase)
        thighs = (self.legs[1][0], self.legs[-1][0])
        targets = {}
        for side, (thigh, calf, foot) in self.legs.items():
            angle = phase + (0 if side > 0 else math.pi)
            stride = math.sin(angle) * .21 * m
            lift = max(0., math.cos(angle)) * .085 * m
            targets[thigh] = self.rest_tg[foot] + np.array([0., -stride, lift])
        hip = self.j['Hip']
        for j, p in enumerate(self.parents):
            if j in self.curl:
                R[j] = R[j] @ self.curl[j]
            if p < 0:
                Rg[j], tg[j] = R[j], t[j]
            else:
                Rg[j] = Rg[p] @ R[j]
                tg[j] = Rg[p] @ t[j] + tg[p]
            if j == hip:
                tg[j] = tg[j] + hip_offset
                self._set(j, p, axis_rotation((0, 0, 1), -twist * .6) @ Rg[j], R, Rg, t, tg)
            elif j in self.spine:
                self._set(j, p, axis_rotation((1, 0, 0), lean / 3) @ axis_rotation((0, 0, 1), twist / 3) @ Rg[j], R, Rg, t, tg)
            elif j in thighs:  # thigh: knee forward
                side = 1 if j == self.legs[1][0] else -1
                calf = self.legs[side][1]
                foot = self.legs[side][2]
                knee, reach = two_bone_joint(tg[j], targets[j], np.linalg.norm(self.rest_t[calf]),
                                             np.linalg.norm(self.rest_t[foot]), np.array([side * .15, -1., 0.]))
                targets[calf] = reach
                self._aim(j, p, calf, knee, R, Rg, t, tg)
            elif j in (self.legs[1][1], self.legs[-1][1]):  # calf
                self._aim(j, p, self.legs[1 if j == self.legs[1][1] else -1][2], targets[j], R, Rg, t, tg)
            elif j in (self.legs[1][2], self.legs[-1][2]):  # foot stays level with the ground
                self._set(j, p, self.rest_Rg[j], R, Rg, t, tg)
            elif j in (self.arms[1][0], self.arms[-1][0]):  # upper arm: elbow back/out
                side = 1 if j == self.arms[1][0] else -1
                fore, hand = self.arms[side][1], self.arms[side][2]
                wrist = grips[side] + .07 * (tg[j] - grips[side]) / np.linalg.norm(tg[j] - grips[side])
                pole = np.array([side * 1., .25, -.35]) if pull else np.array([side * .7, .9, -.5])
                elbow, reach = two_bone_joint(tg[j], wrist, np.linalg.norm(self.rest_t[fore]),
                                              np.linalg.norm(self.rest_t[hand]), pole)
                targets[fore] = reach
                self._aim(j, p, fore, elbow, R, Rg, t, tg)
            elif j in (self.arms[1][1], self.arms[-1][1]):  # forearm
                self._aim(j, p, self.arms[1 if j == self.arms[1][1] else -1][2], targets[j], R, Rg, t, tg)
        rotations = [Gf.Quatf(*mat_to_quat(r)) for r in R]
        self.rotations.Set(Vt.QuatfArray(rotations))
        self.translations.Set(Vt.Vec3fArray.FromNumpy(t.astype(np.float32)))
        delta = Rg[self.head] @ self.rest_Rg[self.head].T
        quat = Gf.Quatf(*mat_to_quat(delta))
        for ops, offset in self.cap:
            ops[0].Set(Gf.Vec3d(*map(float, tg[self.head] + delta @ offset)))
            ops[1].Set(quat)

    def _set(self, j, p, new_global, R, Rg, t, tg):
        parent_R = Rg[p] if p >= 0 else np.eye(3)
        parent_t = tg[p] if p >= 0 else np.zeros(3)
        Rg[j] = new_global
        R[j] = parent_R.T @ new_global
        t[j] = parent_R.T @ (tg[j] - parent_t)

    def _aim(self, j, p, child, point, R, Rg, t, tg):
        current = Rg[j] @ self.rest_t[child]
        self._set(j, p, rotation_between(current, point - tg[j]) @ Rg[j], R, Rg, t, tg)


class ForestScene:
    def __init__(self, stage, realistic=None):
        self.stage = stage
        self.realistic = realistic_assets_available() if realistic is None else realistic
        self.mats = {key: material(stage, key, color, rough, metal) for key, color, rough, metal in [
            ('asphalt', (.075,.082,.083), .95, 0), ('white', (.8,.81,.72), .8, 0),
            ('yellow', (.9,.64,.09), .7, 0), ('soil', (.12,.17,.075), 1.,0),
            ('steel', (.08,.09,.10), .3,.8), ('silver', (.5,.53,.52), .35,.6),
            ('rubber', (.018,.02,.023), .9,0), ('red', (.6,.045,.035), .45,.15),
            ('sensor', (.03,.57,.72), .3,.3), ('green', (.12,.7,.24), .4,0),
            ('skin', (.60,.40,.29), .8,0), ('hoodie', (.026,.032,.042), .95,0),
            ('pants', (.42,.44,.47), .95,0), ('bark', (.20,.16,.105), .95,0),
            ('box', (.58,.43,.27), .95,0), ('tape', (.74,.65,.42), .7,0),
            ('paint_white', (.78,.78,.74), .55, 0), ('paint_yellow', (.86,.6,.08), .55, 0),
            ('powder_coat', (.022,.023,.026), .42, .35)]}
        self.environment = UsdGeom.Xform.Define(stage, '/World/Environment').GetPrim()
        pose(self.environment)
        if self.realistic:
            self.real_materials()
            self.real_road()
            self.real_forest()
            self.road_furniture()
            self.real_sky()
        else:
            self.road()
            self.forest()
            dome = UsdLux.DomeLight.Define(stage, '/World/Sky')
            dome.CreateIntensityAttr(650)
            dome.CreateColorAttr(Gf.Vec3f(.72,.83,1.))
            sun = UsdLux.DistantLight.Define(stage, '/World/Sun')
            sun.CreateIntensityAttr(2800)
            sun.CreateAngleAttr(.8)
            pose(sun.GetPrim(), rotation=(25,-35,-20))

    # ------------------------------------------------------------------ realistic scene
    def real_materials(self):
        c3 = Sdf.ValueTypeNames.Color3f
        fl = Sdf.ValueTypeNames.Float
        self.mats['asphalt_pbr'] = mdl_material(self.stage, 'AsphaltPBR', base_mdl('Natural', 'Asphalt'), {
            'diffuse_tint': (c3, Gf.Vec3f(.78, .78, .8)), 'reflection_roughness_constant': (fl, .62)})
        self.mats['gravel_pbr'] = mdl_material(self.stage, 'GravelPBR', base_mdl('Natural', 'Soil_Rocky'), {
            'diffuse_tint': (c3, Gf.Vec3f(.62, .6, .58))})
        self.mats['grass_pbr'] = mdl_material(self.stage, 'GrassPBR', base_mdl('Natural', 'Grass_Countryside'), {
            'diffuse_tint': (c3, Gf.Vec3f(.82, .9, .72))})
        # forest floor: grass too (the brown mulch read as bare sand from above), a shade darker than the verge
        self.mats['grass_floor_pbr'] = mdl_material(self.stage, 'GrassFloorPBR', base_mdl('Natural', 'Grass_Countryside'), {
            'diffuse_tint': (c3, Gf.Vec3f(.58, .70, .46))})
        self.mats['litter_pbr'] = mdl_material(self.stage, 'LeafLitterPBR', base_mdl('Natural', 'Mulch_Brown'), {
            'diffuse_tint': (c3, Gf.Vec3f(.42, .33, .24))})
        self.mats['dirt_pbr'] = mdl_material(self.stage, 'DirtPBR', base_mdl('Natural', 'Dirt'))

    def real_road(self):
        """Closed four-lane loop (road_path.Road): asphalt and shoulder ribbons, markings, posts, terrain."""
        s, env, m = self.stage, '/World/Environment', self.mats
        road = self.road = road_path.Road()
        W = road_path.WIDTH
        x0, x1, y0, y1 = road.bounds(70.)
        # One flat invisible collider under everything keeps the original contact behaviour.
        prim = shape(s, f'{env}/Ground', 'box', ((x0 + x1) / 2, (y0 + y1) / 2, -.5), (x1 - x0, y1 - y0, 1.), m['asphalt'], collision=True)
        UsdGeom.Imageable(prim).MakeInvisible()
        ribbon(s, f'{env}/RoadSurface', road, -W / 2, W / 2, m['asphalt_pbr'], z=.004, uv_tile=3.5)
        # Grass shoulders (2026-11-01): the coarse terrain dips into the ditch across the shoulder, and the gravel
        # showing through it looked like brown bare triangles beside the road.
        ribbon(s, f'{env}/ShoulderLeft', road, W / 2, W / 2 + road_path.SHOULDER, m['grass_pbr'], z=-.006, uv_tile=1.6)
        ribbon(s, f'{env}/ShoulderRight', road, -W / 2 - road_path.SHOULDER, -W / 2, m['grass_pbr'], z=-.006, uv_tile=1.6)
        # Markings: double yellow centre line, dashed white lane lines, solid white edge lines.
        for name, d in (('CenterLineA', -.14), ('CenterLineB', .14)):
            ribbon(s, f'{env}/{name}', road, d - .06, d + .06, m['paint_yellow'], z=.010, uv_tile=1.)
        for name, d in (('EdgeLeft', W / 2 - .30), ('EdgeRight', -W / 2 + .30)):
            ribbon(s, f'{env}/{name}', road, d - .075, d + .075, m['paint_white'], z=.010, uv_tile=1.)
        dashes = [(a, a + 3.) for a in np.arange(0., road.length - 3., 9.)]
        for name, d in (('LaneLineLeft', road_path.LANE_W), ('LaneLineRight', -road_path.LANE_W)):
            ribbon(s, f'{env}/{name}', road, d - .075, d + .075, m['paint_white'], z=.010, uv_tile=1., s_ranges=dashes)
        # Delineator posts every 12 m on both shoulders.
        for k, sv in enumerate(np.arange(0., road.length, 12.)):
            for side in (-1, 1):
                (px, py), th = road.frenet_to_xy(sv, side * (W / 2 + 2.0))
                root = f'{env}/Post_{side+1}_{k}'
                shape(s, root, 'box', (px, py, .43), (.10, .10, .85), m['paint_white'], (0, 0, math.degrees(th)))
                shape(s, root + '/Reflector', 'box', (0, -side * .505, .25), (.8, .04, .18), m['yellow'])
        # Terrain grid (2 m) over the loop; distance to the centreline shapes ditch, verge and forest floor.
        xs = np.arange(x0, x1 + 1, 2.)
        ys = np.arange(y0, y1 + 1, 2.)
        dist_fn = lambda gx, gy: road.distance(gx, gy)[0]
        grid_mesh(s, f'{env}/Terrain', xs, ys, m['grass_pbr'], 2.2, terrain_height,
                  subsets={'Verge': (0, ROAD_EDGE + 8., m['grass_pbr']), 'Floor': (ROAD_EDGE + 8., 1e9, m['grass_floor_pbr'])},
                  dist_fn=dist_fn)

    def road_furniture(self):
        """Realism extras along the loop: tyre-wear tracks in every lane, W-beam guardrails with posts on both
        shoulders, utility poles with three wires inside the loop, speed-limit and curve-warning signs."""
        import materials_pbr as M
        from truck_geom import cylinder_between, lathe, sweep_profile
        from truck_porter import decal, decal_material
        s, env, m, road = self.stage, '/World/Environment', self.mats, self.road
        W = road_path.WIDTH
        # Wheel tracks: the same asphalt, darker and smoother, as opaque strips (blended overlays reflect the sky).
        wear = mdl_material(s, 'TyreWear', base_mdl('Natural', 'Asphalt'), {
            'diffuse_tint': (Sdf.ValueTypeNames.Color3f, Gf.Vec3f(.52, .52, .55)),
            'reflection_roughness_constant': (Sdf.ValueTypeNames.Float, .5)})
        for lane, centre in enumerate((-1.5, -.5, .5, 1.5)):
            for k, off in enumerate((-.95, .95)):
                d = centre * road_path.LANE_W + off
                ribbon(s, f'{env}/Wear{lane}_{k}', road, d - .28, d + .28, wear, z=.006, uv_tile=3.5)
        rail = M.pbr(s, 'Guardrail', (.72, .74, .76), .42, .85)
        post = M.pbr(s, 'GuardrailPost', (.5, .52, .54), .5, .7)
        w_beam = [(0., .42), (-.03, .46), (.05, .52), (-.02, .59), (.05, .66), (-.03, .72), (0., .76)]
        for side in (-1, 1):
            d = side * (ROAD_EDGE + .35)
            profile = [(side * dy, dz) for dy, dz in w_beam]
            sweep_profile(s, f'{env}/Guardrail{side+1}', road, d, profile, rail, step=1., uv_scale=1.)
            for k, sv in enumerate(np.arange(0., road.length, 2.)):
                (px, py), th = road.frenet_to_xy(sv, d + side * .06)
                shape(s, f'{env}/GuardPost{side+1}_{k}', 'box', (px, py, .38), (.07, .12, .76), post, (0, 0, math.degrees(th)))
        concrete = M.pbr(s, 'PoleConcrete', (.58, .58, .56), .85, 0.)
        wire = M.pbr(s, 'PowerWire', (.06, .06, .06), .6, .2)
        tops = []
        for k, sv in enumerate(np.arange(10., road.length, 45.)):
            (px, py), th = road.frenet_to_xy(sv, ROAD_EDGE + 3.2)
            z0 = float(self.ground_z(np.array([px]), np.array([py]))[0])
            root = f'{env}/Pole{k}'
            pole = UsdGeom.Xform.Define(s, root).GetPrim()
            pose(pole, (px, py, z0 - .3), (0, 0, math.degrees(th)))
            lathe(s, root + '/Post', [(.17, 0.), (.15, 4.), (.12, 9.8)], concrete, 20)
            shape(s, root + '/CrossArm', 'box', (0, 0, 9.3), (.12, 2.0, .12), concrete)
            for j, y in enumerate((-.8, 0., .8)):
                shape(s, root + f'/Insulator{j}', 'cylinder', (0, y, 9.45), (.05, .05, .18), m['plastic'] if 'plastic' in m else concrete)
            tops.append(np.array([px, py, z0 - .3 + 9.52]) + np.array([-math.sin(th), math.cos(th), 0.])[None, :] * np.array([[-.8], [0.], [.8]]))
        for k in range(len(tops)):
            a, b = tops[k], tops[(k + 1) % len(tops)]
            for j in range(3):
                mid = (a[j] + b[j]) / 2 - np.array([0, 0, .9])     # sag
                cylinder_between(s, f'{env}/Wire{k}_{j}a', a[j], mid, .014, wire, 8)
                cylinder_between(s, f'{env}/Wire{k}_{j}b', mid, b[j], .014, wire, 8)
        signs = {'speed': decal_material(s, 'SignSpeed', NVIDIA.parents[0] / 'truck' / 'sign_speed60.png'),
                 'left': decal_material(s, 'SignCurveLeft', NVIDIA.parents[0] / 'truck' / 'sign_curve_left.png')}
        sign_post = M.pbr(s, 'SignPost', (.55, .56, .58), .45, .8)
        placements = [(road.start_s + 45., 'speed')] + [(sv, 'left') for sv in (100., 203., 386., 489.)]
        for k, (sv, kind) in enumerate(placements):
            (px, py), th = road.frenet_to_xy(sv, -(ROAD_EDGE + 1.3))
            tx, ty = math.cos(th), math.sin(th)
            nx, ny = -math.sin(th), math.cos(th)
            shape(s, f'{env}/SignPost{k}', 'cylinder', (px, py, 1.3), (.04, .04, 2.6), sign_post)
            decal(s, f'{env}/Sign{k}', (px - tx * .03, py - ty * .03, 2.35), (-nx * .42, -ny * .42, 0), (0, 0, .42), signs[kind])
            shape(s, f'{env}/SignBack{k}', 'box', (px + tx * .005, py + ty * .005, 2.35), (.01, .84, .84), sign_post, (0, 0, math.degrees(th)))

    def ground_z(self, x, y):
        """Terrain height at world points (arrays), via the road distance field."""
        x, y = np.asarray(x, float), np.asarray(y, float)
        dist, _ = self.road.distance(x, y)
        return terrain_height(x.ravel(), y.ravel(), dist).reshape(x.shape)

    def _instancer(self, path, names, folder_of, entries):
        """PointInstancer whose prototypes reference collected NVIDIA assets (cm -> m)."""
        inst = UsdGeom.PointInstancer.Define(self.stage, path)
        protos = []
        for name in names:
            proto = UsdGeom.Xform.Define(self.stage, f'{path}/Prototypes/{name}').GetPrim()
            pose(proto, scale=(.01, .01, .01))
            proto.GetReferences().AddReference(str(folder_of(name)))
            protos.append(proto.GetPath())
        inst.CreatePrototypesRel().SetTargets(protos)
        positions, scales, orients, ids = zip(*entries)
        inst.CreatePositionsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in positions]))
        inst.CreateScalesAttr(Vt.Vec3fArray([Gf.Vec3f(s, s, s) for s in scales]))
        inst.CreateOrientationsAttr(Vt.QuathArray([Gf.Quath(Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw).GetQuat()) for yaw in orients]))
        inst.CreateProtoIndicesAttr(Vt.IntArray(list(ids)))
        return inst

    def real_forest(self):
        """Trees in rows along both sides of the loop plus a sparser scatter inside and outside it;
        grass, leaf litter and rocks in Frenet offsets from the road."""
        rng = random.Random(124)
        road = self.road
        names = list(TREES)
        weights = [TREES[n][0] for n in names]
        light = [names.index('White_Ash'), names.index('Yellow_Pine')]
        trees = []
        for side in (-1, 1):
            for row in range(TREE_ROWS):
                spacing = 7.5 + row * 1.5
                for sv in np.arange(rng.uniform(0, spacing), road.length, spacing):
                    d = side * (ROAD_EDGE + 3.5 + row * 4.8 + rng.uniform(-1.4, 1.4))
                    k = rng.choices(range(len(names)), weights)[0]
                    while row == 0 and names[k] == 'Red_Oak':   # wide low crowns would hang over the road
                        k = rng.choices(range(len(names)), weights)[0]
                    (x, y), _ = road.frenet_to_xy(sv + rng.uniform(-2, 2), d)
                    trees.append((x, y, rng.uniform(*TREES[names[k]][1:]), rng.uniform(0, 360), k))
        # Scatter further away (inside and outside the loop) so the forest does not end abruptly.
        x0, x1, y0, y1 = road.bounds(55.)
        cand = np.column_stack([np.random.RandomState(7).uniform(x0, x1, 1600), np.random.RandomState(8).uniform(y0, y1, 1600)])
        dist, _ = road.distance(cand[:, 0], cand[:, 1])
        for (x, y), dd in zip(cand[dist > ROAD_EDGE + 3.5 + TREE_ROWS * 4.8 + 2.], dist[dist > ROAD_EDGE + 3.5 + TREE_ROWS * 4.8 + 2.]):
            if rng.random() < .22:
                k = rng.choice(light) if rng.random() < .5 else rng.choices(range(len(names)), weights)[0]
                trees.append((x, y, rng.uniform(*TREES[names[k]][1:]), rng.uniform(0, 360), k))
        xs = np.array([t[0] for t in trees]); ys = np.array([t[1] for t in trees])
        zs = self.ground_z(xs, ys) - .05
        # Trees contain their own branch PointInstancers; nesting those inside another
        # PointInstancer drops the leaves in RTX, so each tree is an instanceable reference.
        UsdGeom.Xform.Define(self.stage, '/World/Environment/Trees')
        for i, ((x, y, scale, yaw, k), z) in enumerate(zip(trees, zs)):
            tree = UsdGeom.Xform.Define(self.stage, f'/World/Environment/Trees/{names[k]}_{i:03d}').GetPrim()
            pose(tree, (x, y, float(z)), (0, 0, yaw), (scale * .01,) * 3)
            tree.GetReferences().AddReference(str(veg('Trees', f'{names[k]}.usd')))
            tree.SetInstanceable(True)
        self.tree_count = len(trees)

        def frenet_entries(count, d_lo, d_hi, scale_fn, proto_fn, z_off):
            sv = np.array([rng.uniform(0, road.length) for _ in range(count)])
            dv = np.array([rng.choice((-1, 1)) * rng.uniform(d_lo, d_hi) for _ in range(count)])
            xy = road.offsets(sv, dv)
            z = terrain_height(xy[:, 0], xy[:, 1], np.abs(dv)) + z_off
            return [((float(xy[i, 0]), float(xy[i, 1]), float(z[i])), scale_fn(k), rng.uniform(0, 360), k)
                    for i, k in enumerate(proto_fn() for _ in range(count))]
        cover_scale = {0: (.8, 1.3), 1: (.8, 1.4), 2: (.9, 1.6), 3: (.35, .6)}
        near = frenet_entries(COVER_NEAR, ROAD_EDGE + .6, ROAD_EDGE + 7., lambda k: rng.uniform(*cover_scale[k]),
                              lambda: rng.choices(range(4), [.14, .40, .38, .08])[0], -.01)
        far = frenet_entries(COVER_FAR, ROAD_EDGE + 7., ROAD_EDGE + 24., lambda k: rng.uniform(*cover_scale[k]),
                             lambda: rng.choices(range(4), [.06, .42, .46, .06])[0], -.01)
        self._instancer('/World/Environment/GroundCover', GROUND_COVER, lambda n: veg('Shrub', f'{n}.usd'), near + far)
        # No leaf-litter patches since 2026-11-01: the brown mats beside the road read as bare ground
        # (user request "grass where it is missing beside the road").
        rocks = frenet_entries(ROCKS, ROAD_EDGE + .6, ROAD_EDGE + 20., lambda k: rng.uniform(.5, 2.2),
                               lambda: rng.randrange(5), -.03)
        self._instancer('/World/Environment/Rocks', [f'rock_small_{i:02d}' for i in range(1, 6)],
                        lambda n: veg('Rocks', f'{n}.usda'), rocks)

    def real_sky(self):
        dome = UsdLux.DomeLight.Define(self.stage, '/World/Sky')
        # sunny day (2026-11-01): brighter, slightly bluer sky light and a stronger, whiter sun
        dome.CreateIntensityAttr(1050)
        dome.CreateColorAttr(Gf.Vec3f(.90, .95, 1.))
        dome.CreateTextureFileAttr(Sdf.AssetPath(str(SKY).replace('\\', '/')))
        dome.CreateTextureFormatAttr(UsdLux.Tokens.latlong)
        sun = UsdLux.DistantLight.Define(self.stage, '/World/Sun')
        sun.CreateIntensityAttr(4300)
        sun.CreateAngleAttr(.53)
        sun.CreateColorAttr(Gf.Vec3f(1., .97, .91))
        pose(sun.GetPrim(), rotation=(38, -28, -25))

    @staticmethod
    def render_settings():
        """RTX real-time settings for an outdoor look; applied after the app has started."""
        return {
            '/rtx/ambientOcclusion/enabled': True,
            '/rtx/ambientOcclusion/rayLength': 1.5,
            # Translucent glass (UsdPreviewSurface opacity < 1) instead of binary cutouts.
            '/rtx/raytracing/fractionalCutoutOpacity': True,
            '/rtx/indirectDiffuse/enabled': True,
            '/rtx/directLighting/sampledLighting/enabled': True,
            '/rtx/post/dlss/execMode': 2,              # DLSS quality
            '/rtx/reflections/maxRoughness': .6,
            # Light atmospheric haze for depth on the long straights.
            '/rtx/fog/enabled': True,
            '/rtx/fog/fogColor': [.80, .87, .96],     # clear day: only a faint distant haze
            '/rtx/fog/fogColorIntensity': 1.,
            '/rtx/fog/fogStartDist': 250.,
            '/rtx/fog/fogEndDist': 1500.,
            '/rtx/fog/fogDistanceDensity': .03,
            '/rtx/fog/fogStartHeight': 0.,
            '/rtx/fog/fogHeightDensity': .0,
        }

    @staticmethod
    def path_tracing_settings():
        """Offline-quality settings used for HQ exports (progressive accumulation)."""
        return {'/rtx/rendermode': 'PathTracing', '/rtx/pathtracing/spp': 8, '/rtx/pathtracing/totalSpp': 512,
                '/rtx/pathtracing/maxBounces': 6, '/rtx/pathtracing/clampSpp': 0}

    # ------------------------------------------------------------------ procedural fallback
    def road(self):
        for name, pos, scale, mat in [
            ('Road', (0,0,-.08),(100,5.8,.16),'asphalt'),
            ('Earth',(0,0,-.22),(110,42,.24),'soil'),
            ('CenterLineA',(0,-.07,.006),(100,.06,.012),'yellow'),
            ('CenterLineB',(0,.07,.006),(100,.06,.012),'yellow'),
            ('EdgeLeft',(0,2.68,.006),(100,.08,.012),'white'),
            ('EdgeRight',(0,-2.68,.006),(100,.08,.012),'white')]:
            shape(self.stage, '/World/Environment/'+name, 'box', pos, scale, self.mats[mat], collision=name in ('Road','Earth'))
        for side in (-1, 1):
            for x in range(-45,46,6):
                root = f'/World/Environment/Post_{side+1}_{x+50}'
                shape(self.stage,root,'box',(x,side*3.1,.43),(.10,.10,.85),self.mats['white'])
                shape(self.stage,root+'/Reflector','box',(0,-side*.505,.25),(.8,.04,.18),self.mats['yellow'])

    def forest(self):
        rng = random.Random(124)
        UsdGeom.Xform.Define(self.stage,'/World/TreeLibrary')
        leaves = [material(self.stage, f'Leaf{i}', c) for i,c in enumerate([
            (.10,.23,.045),(.19,.34,.065),(.30,.40,.085),(.12,.29,.08),(.26,.36,.055)])]
        # Five reusable broadleaf crowns. Elliptical leaf meshes, not billboard spheres.
        prototypes = []
        for variant in range(5):
            root = f'/World/TreeLibrary/Tree{variant}'
            UsdGeom.Xform.Define(self.stage,root)
            height = rng.uniform(5.5,8.)
            shape(self.stage,root+'/Trunk','cylinder',(0,0,height*.38),(.15,.15,height*.76),self.mats['bark'])
            points, indices, counts = [],[],[]
            for branch in range(12):
                angle = branch*2.399
                z = height*(.40+.035*branch)
                endpoint = (math.cos(angle)*1.5,math.sin(angle)*1.5,z+.85)
                link(self.stage,root+f'/Branch{branch}',(0,0,z),endpoint,.055,self.mats['bark'])
            for leaf in range(1800):
                u = rng.uniform(-1,1)
                a = rng.uniform(0,math.tau)
                rad = rng.random()**(1/3)
                center = np.array([math.sqrt(1-u*u)*math.cos(a)*2.2*rad,
                                   math.sqrt(1-u*u)*math.sin(a)*2.2*rad,
                                   height*.74 + u*height*.34*rad])
                normal = np.array([rng.uniform(-1,1),rng.uniform(-1,1),rng.uniform(.2,1)])
                normal /= np.linalg.norm(normal)
                tangent = np.cross(normal,[1,0,0]); tangent /= np.linalg.norm(tangent)
                other = np.cross(normal,tangent)
                length = rng.uniform(.12,.28)
                base = len(points)
                for k in range(6):
                    theta = k*math.tau/6
                    points.append(tuple(center+tangent*math.cos(theta)*length+other*math.sin(theta)*length*.48))
                counts.append(6); indices.extend(range(base,base+6))
            mesh = UsdGeom.Mesh.Define(self.stage,root+'/Leaves')
            mesh.CreatePointsAttr(points);mesh.CreateFaceVertexCountsAttr(counts);mesh.CreateFaceVertexIndicesAttr(indices)
            mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True)
            bind(mesh.GetPrim(),leaves[variant])
            prototypes.append(root)
        inst = UsdGeom.PointInstancer.Define(self.stage,'/World/Environment/Trees')
        inst.CreatePrototypesRel().SetTargets(prototypes)
        positions, scales, rotations, variants = [],[],[],[]
        for side in (-1,1):
            for row in range(4):
                for i in range(27):
                    positions.append(Gf.Vec3f(-49+i*3.8+rng.uniform(-1.3,1.3),side*(5.+row*3.4+rng.uniform(-.6,.6)),0))
                    s=rng.uniform(.75,1.3);scales.append(Gf.Vec3f(s,s,s))
                    rotations.append(Gf.Quath(Gf.Rotation(Gf.Vec3d(0,0,1),rng.uniform(0,360)).GetQuat()))
                    variants.append(rng.randrange(5))
        inst.CreatePositionsAttr(positions);inst.CreateScalesAttr(scales)
        inst.CreateOrientationsAttr(rotations);inst.CreateProtoIndicesAttr(variants)
        # Prototype roots are relocated below ground without affecting instancer-relative geometry.
        pose(self.stage.GetPrimAtPath('/World/TreeLibrary'),(0,0,-100))

    # ------------------------------------------------------------------ cart and operator
    def rig(self, root, frame=None):
        """Instrumented plate: frame, deck (collider), 4 load cells, 2 tension cells, IMU, straps.

        Rig frame: N (front, belt c1) row of cells at x=-.196, S row at +.196; NW at y=+.125."""
        s, m = self.stage, self.mats
        frame = frame or (m['powder_coat'] if self.realistic else m['steel'])
        shape(s,root+'/Frame','box',(0,0,.135),(.70,.48,.045),frame)
        shape(s,root+'/Deck','box',(0,0,.205),(.65,.43,.04),m['silver'],collision=True)
        for name,x,y in [('NW',-.196,.125),('NE',-.196,-.125),('SW',.196,.125),('SE',.196,-.125)]:
            cell=shape(s,root+'/LoadCell_'+name,'box',(x,y,.172),(.075,.035,.025),m['sensor'])
            cell.CreateAttribute('sensor:capacity_kg',Sdf.ValueTypeNames.Float,custom=True).Set(5.)
        for i,x in enumerate((-.18,.18)):
            cell=shape(s,root+f'/TensionCell_{i+1}','box',(x,-.22,.27),(.09,.035,.025),m['green'])
            cell.CreateAttribute('sensor:capacity_N',Sdf.ValueTypeNames.Float,custom=True).Set(98.0665)
        shape(s,root+'/IMUHousing','box',(.25,.12,.25),(.04,.03,.015),m['sensor'])
        # Two over-the-top lashing belts as flexible webbing (belt_rope.py): a thin strip from the anchor beside
        # the plate, over the cargo, to the other anchor, re-draped every frame on the cargo's actual shape.
        import belt_rope
        self.straps=[]
        self.strap_paths=[None,None]
        for i,x in enumerate(belt_rope.BELT_X):
            mesh=UsdGeom.Mesh.Define(s,root+f'/Belt{i}')
            mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True)
            bind(mesh.GetPrim(),m['rubber'])
            self.straps.append(mesh.GetPrim())
            # the tension cell sits in the belt on the anchor side, like the real cart
        self.strap_top = None
        self.fit_straps(.78)

    def drape_belt(self, i, path):
        """Lay belt i along a path [(y, z), ...] in its plane (rig frame)."""
        import belt_rope
        path = np.asarray(path, float)
        old = self.strap_paths[i]
        if old is not None and len(old) == len(path) and np.abs(old - path).max() < .0015:
            return
        self.strap_paths[i] = path
        pts, quads = belt_rope.ribbon(path, belt_rope.BELT_X[i])
        mesh = UsdGeom.Mesh(self.straps[i])
        mesh.CreatePointsAttr().Set(Vt.Vec3fArray.FromNumpy(pts))
        mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray([4] * len(quads)))
        mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(np.asarray(quads).ravel().tolist()))

    def fit_straps(self, top_z, half_width=.19):
        """Drape both belts over a flat-topped cargo of height top_z (rig z), e.g. before any measurement."""
        import belt_rope
        top_z = float(max(.30, top_z))
        if self.strap_top is not None and abs(top_z - self.strap_top) < .004:
            return
        self.strap_top = top_z
        section = [(-half_width, top_z), (half_width, top_z), (-half_width, .23), (half_width, .23)]
        for i in range(2):
            self.drape_belt(i, belt_rope.upper_path(section))

    def drape_on_points(self, points, band=.012):
        """Drape both belts over measured cargo points (rig frame, (n, 3)) - the D435i surface in replay."""
        import belt_rope
        points = np.asarray(points, float).reshape(-1, 3)
        for i, x in enumerate(belt_rope.BELT_X):
            near = points[np.abs(points[:, 0] - x) < band]
            self.drape_belt(i, belt_rope.upper_path(near[:, 1:] if len(near) else np.zeros((0, 2))))

    def cart(self, root):
        s, m = self.stage, self.mats
        frame = m['powder_coat'] if self.realistic else m['steel']
        self.rig(root, frame)
        for i, (x,y) in enumerate([(-.27,-.205),(-.27,.205),(.27,-.205),(.27,.205)]):
            shape(s,root+f'/Wheel{i}','cylinder',(x,y,.075),(.075,.075,.045),m['rubber'],(90,0,0))
            shape(s,root+f'/Hub{i}','cylinder',(x,y*1.10,.075),(.036,.036,.049),m['red'],(90,0,0))
        for side in (-1,1):
            link(s,root+f'/Handle{side+1}',(.29,side*GRIP_Y,.17),(GRIP_X,side*GRIP_Y,GRIP_Z),.019,frame)
        link(s,root+'/Grip',(GRIP_X,-GRIP_Y,GRIP_Z),(GRIP_X,GRIP_Y,GRIP_Z),.022,frame)
        link(s,root+'/CameraMast',(.31,.19,.22),(.18,.19,1.03),.012,frame)
        shape(s,root+'/D435iHousing','box',(.18,.13,1.03),(.026,.09,.025),m['silver'])

    def operator(self):
        if self.realistic:
            self.skinned = SkinnedOperator(self.stage)
            self.person = self.skinned.prim
            return
        self.skinned = None
        self._procedural_operator()

    def driver(self, root, seat):
        """Seated delivery driver (realistic assets only). seat = truck.DRIVER layout dict."""
        if not self.realistic:
            self.skinned = None
            self.person = UsdGeom.Xform.Define(self.stage, root).GetPrim()
            return None
        self.skinned = SkinnedOperator(self.stage, root, outfit='delivery')
        self.person = self.skinned.prim
        pose(self.person, seat['root'])
        self.skinned.sit(seat, 0.)
        return self.skinned

    def operator_offset(self, pull):
        """Operator root distance along cart X (the realistic body needs the grip in reach)."""
        return SkinnedOperator.offset(pull) if self.skinned else .79

    def operator_yaw(self, pull):
        """Extra yaw so the operator faces travel (pull) or the cart (push)."""
        if self.skinned:
            return 0 if pull else 180
        return 180 if pull else 0

    def walk(self, phase, moving, pull=True):
        if self.skinned:
            self.skinned.walk(phase, moving, pull)
        else:
            self._procedural_walk(phase, moving, pull)

    def _procedural_operator(self):
        s,m=self.stage,self.mats
        self.person=UsdGeom.Xform.Define(s,'/World/Operator').GetPrim();pose(self.person)
        root='/World/Operator'
        shape(s,root+'/Torso','sphere',(0,0,1.23),(.15,.235,.29),m['hoodie'])
        shape(s,root+'/Hood','sphere',(.035,0,1.47),(.12,.17,.12),m['hoodie'])
        shape(s,root+'/Neck','cylinder',(-.01,0,1.53),(.065,.065,.1),m['skin'])
        shape(s,root+'/Head','sphere',(-.025,0,1.66),(.10,.087,.125),m['skin'])
        shape(s,root+'/Hair','sphere',(-.005,0,1.70),(.097,.091,.10),m['hoodie'])
        shape(s,root+'/Cap','sphere',(-.03,0,1.75),(.112,.10,.05),m['hoodie'])
        shape(s,root+'/CapBrim','sphere',(-.13,0,1.737),(.10,.105,.011),m['hoodie'])
        shape(s,root+'/Nose','sphere',(-.123,0,1.665),(.018,.021,.025),m['skin'])
        self.limbs={}
        self.arms={}
        for side in (-1,1):
            y=side*.22
            elbow=(-.20,y,1.13);hand=(-.43,side*.205,.94)
            self.arms[side]=[
                link(s,root+f'/UpperArm{side+1}',(-.025,y,1.39),elbow,.065,m['hoodie']),
                link(s,root+f'/Forearm{side+1}',elbow,hand,.052,m['hoodie']),
                shape(s,root+f'/Hand{side+1}','sphere',hand,(.047,.037,.031),m['skin'])]
            self.limbs[side]=[
                link(s,root+f'/Thigh{side+1}',(0,side*.1,.98),(0,side*.1,.54),.092,m['pants']),
                link(s,root+f'/Shin{side+1}',(0,side*.1,.54),(0,side*.1,.10),.074,m['pants']),
                shape(s,root+f'/Shoe{side+1}','sphere',(-.05,side*.1,.05),(.145,.066,.047),m['rubber'])]

    def _procedural_walk(self, phase, moving, pull=True):
        moving = abs(moving)
        for side,(upper,lower,handprim) in self.arms.items():
            direction=1 if pull else -1
            elbow=(direction*.20,side*.22,1.13)
            hand=(direction*.43,side*.205,.94)
            move_link(upper,(-.025,side*.22,1.39),elbow,.065)
            move_link(lower,elbow,hand,.052)
            pose(handprim,hand,scale=(.047,.037,.031))
        for side,(thigh,shin,shoe) in self.limbs.items():
            angle=phase+(math.pi if side<0 else 0)
            stride=math.sin(angle)*.22*moving
            lift=max(0,math.cos(angle))*.095*moving
            hip=(0,side*.11,.96)
            knee=(stride*.4-.035,side*.11,.52+lift*.4)
            ankle=(stride,side*.11,.09+lift)
            move_link(thigh,hip,knee,.088);move_link(shin,knee,ankle,.072)
            pose(shoe, (stride-.055,side*.11,.045+lift),scale=(.145,.066,.045))


def add_scan(stage, path, parent, height=.56):
    asset=Usd.Stage.Open(str(path))
    bbox=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render']).ComputeWorldBound(asset.GetPseudoRoot()).ComputeAlignedRange()
    lo,hi=np.array(bbox.GetMin()),np.array(bbox.GetMax())
    scale=height/max(hi[2]-lo[2],.001)
    root=UsdGeom.Xform.Define(stage,parent).GetPrim();pose(root)
    ref=stage.DefinePrim(parent+'/Mesh','Xform');ref.GetReferences().AddReference(str(path))
    # Wrap the referenced transform so imported operations remain untouched.
    pose(root,((lo[1]+hi[1])*.5*scale,-(lo[0]+hi[0])*.5*scale,-lo[2]*scale-.28),rotation=(0,0,90),scale=(scale,scale,scale))
    for prim in Usd.PrimRange(ref):
        if prim.GetTypeName().endswith('Light') or prim.IsA(UsdGeom.Camera):
            prim.SetActive(False)
    return root
