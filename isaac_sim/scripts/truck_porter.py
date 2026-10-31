"""Procedural 1-ton cab-over flatbed truck (Porter-class) built from USD primitives.

Truck frame: x forward, y left, z up, origin on the ground at the wheelbase centre.
Rough dimensions follow a Hyundai Porter II long-bed: overall ~5.0 x 1.74 x 2.05 m, wheelbase 2.65 m,
bed floor 0.80 m above the ground, dual rear wheels. The instrumented plate from the hand cart sits on
the bed behind the cab (RIG below); its physics is unchanged.
"""
import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdShade

from forest_scene import material, pose, shape

WHEEL_R = .33
BED_TOP = .80            # bed floor top (m above ground)
BED_X = (-2.10, .80)     # bed floor extent along x
BED_W = 1.63
CAB_X = (.90, 2.45)
CAB_W = 1.70
CAB_FLOOR = .62
ROOF = 2.05
AXLE_F, AXLE_R = 1.325, -1.325
# Instrumented plate: rig-local frame rotated 180 deg about z so its N (front) row faces the cab.
# 'scale' enlarges the *appearance* of the plate, belts, cells and measured cargo to suit the truck bed;
# every force, mass, COG distance and judgement still uses the hand-cart's real geometry (the twin
# divides positions by this scale before any sensor arithmetic).
RIG = {'x': -.50, 'y': 0., 'z': BED_TOP - .1125 * 2.0, 'yaw': 180., 'scale': 2.0}
# Driver layout in the operator frame (x forward, y left, z up; origin = seat root on the cab floor).
DRIVER = {'root': (1.12, .46, CAB_FLOOR), 'hip_z': .47, 'hip_x': .0, 'knee': (.36, .50), 'pedal': (.62, .10),
          'wheel': (.40, .70), 'wheel_r': .19, 'wheel_tilt_deg': 28., 'hands_deg': 55., 'lean_deg': 10.}
# D435i on a mast above the cab guard looking down and back over the (scaled) cargo (truck frame).
CARGO_TOP_Z = BED_TOP + .825 * RIG['scale']            # typical measured stack: plate + 0.6 m boxes
CAMERA = {'pos': (BED_X[1] - .02, 0., CARGO_TOP_Z + .55),
          'pitch_deg': math.degrees(math.atan2(CARGO_TOP_Z + .55 - (BED_TOP + .4 * RIG['scale']), BED_X[1] - RIG['x']))}
RISK_COLOURS = {0: (.1, .9, .25), 1: (1., .85, .1), 2: (1., .45, .05), 3: (1., .08, .05)}


def glass_material(stage, name):
    mat = UsdShade.Material.Define(stage, '/World/Materials/' + name)
    shader = UsdShade.Shader.Define(stage, mat.GetPath().AppendChild('Shader'))
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.30, .40, .48))
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.05)
    shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(0.)
    shader.CreateInput('opacity', Sdf.ValueTypeNames.Float).Set(.28)
    shader.CreateInput('ior', Sdf.ValueTypeNames.Float).Set(1.5)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    return mat


def emissive_material(stage, name, colour, strength=6.):
    mat = UsdShade.Material.Define(stage, '/World/Materials/' + name)
    shader = UsdShade.Shader.Define(stage, mat.GetPath().AppendChild('Shader'))
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*colour))
    emissive = shader.CreateInput('emissiveColor', Sdf.ValueTypeNames.Color3f)
    emissive.Set(Gf.Vec3f(*[c * strength for c in colour]))
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.3)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    return mat, emissive


ASSETS = Path(__file__).resolve().parents[1] / 'assets' / 'truck'


def decal_material(stage, name, texture):
    """UsdPreviewSurface with an RGBA texture: colour from RGB, cut-out from alpha."""
    mat = UsdShade.Material.Define(stage, '/World/Materials/' + name)
    st = UsdShade.Shader.Define(stage, mat.GetPath().AppendChild('Primvar'))
    st.CreateIdAttr('UsdPrimvarReader_float2')
    st.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
    st_out = st.CreateOutput('result', Sdf.ValueTypeNames.Float2)
    tex = UsdShade.Shader.Define(stage, mat.GetPath().AppendChild('Texture'))
    tex.CreateIdAttr('UsdUVTexture')
    tex.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(str(texture).replace('\\', '/'))
    tex.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(st_out)
    tex.CreateInput('wrapS', Sdf.ValueTypeNames.Token).Set('clamp')
    tex.CreateInput('wrapT', Sdf.ValueTypeNames.Token).Set('clamp')
    rgb = tex.CreateOutput('rgb', Sdf.ValueTypeNames.Float3)
    alpha = tex.CreateOutput('a', Sdf.ValueTypeNames.Float)
    shader = UsdShade.Shader.Define(stage, mat.GetPath().AppendChild('Shader'))
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
    shader.CreateInput('opacity', Sdf.ValueTypeNames.Float).ConnectToSource(alpha)
    shader.CreateInput('opacityThreshold', Sdf.ValueTypeNames.Float).Set(.4)
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.35)
    shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(.6)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    return mat


def decal(stage, path, centre, u_axis, v_axis, mat):
    """Textured quad: centre +- u_axis (texture right) +- v_axis (texture up); faces u x v."""
    c, u, v = (np.asarray(a, float) for a in (centre, u_axis, v_axis))
    pts = [c - u - v, c + u - v, c + u + v, c - u + v]
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in pts])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateSubdivisionSchemeAttr('none')
    mesh.CreateDoubleSidedAttr(True)
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    from forest_scene import bind
    bind(mesh.GetPrim(), mat)
    return mesh.GetPrim()


def box(stage, path, lo, hi, mat, rotation=(0, 0, 0)):
    """Axis-aligned box from min/max corners."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return shape(stage, path, 'box', (lo + hi) / 2, hi - lo, mat, rotation)


def round_cylinder(stage, path, radius, height, mat, segments=48, r_inner=0.):
    """Smooth cylinder mesh (axis z, centred) - UsdGeom.Cylinder tessellates visibly coarse in RTX.
    r_inner > 0 makes a ring (rim lip) instead of a solid disc."""
    a = np.linspace(0, 2 * np.pi, segments, endpoint=False)
    ring = np.stack([np.cos(a), np.sin(a)], -1)
    hz = height / 2
    pts, counts, idx = [], [], []
    top = np.column_stack([ring * radius, np.full(segments, hz)])
    bot = np.column_stack([ring * radius, np.full(segments, -hz)])
    pts.extend(top.tolist()); pts.extend(bot.tolist())
    for i in range(segments):          # side quads, outward normals
        j = (i + 1) % segments
        counts.append(4); idx.extend([i, j, segments + j, segments + i])
    if r_inner > 0:
        tin = np.column_stack([ring * r_inner, np.full(segments, hz)])
        bin_ = np.column_stack([ring * r_inner, np.full(segments, -hz)])
        o = len(pts); pts.extend(tin.tolist()); pts.extend(bin_.tolist())
        for i in range(segments):
            j = (i + 1) % segments
            counts.append(4); idx.extend([i, o + i, o + j, j])                       # top annulus
            counts.append(4); idx.extend([segments + j, o + segments + j, o + segments + i, segments + i])  # bottom
            counts.append(4); idx.extend([o + j, o + segments + j, o + segments + i, o + i])   # inner wall
    else:
        counts.append(segments); idx.extend(range(segments))                          # top cap (ccw from +z)
        counts.append(segments); idx.extend(reversed(range(segments, 2 * segments)))  # bottom cap
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in pts])
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.CreateSubdivisionSchemeAttr('none')
    from forest_scene import bind
    bind(mesh.GetPrim(), mat)
    return mesh.GetPrim()


class Truck:
    def __init__(self, stage, root, mats):
        s = stage
        self.stage, self.root = s, root
        m = dict(mats)
        m.update(paint=material(s, 'TruckPaint', (.86, .87, .88), .32, .05),
                 paint_dark=material(s, 'TruckPaintDark', (.16, .17, .19), .5, .1),
                 plastic=material(s, 'TruckPlastic', (.03, .03, .032), .75, 0),
                 chrome=material(s, 'TruckChrome', (.80, .81, .82), .18, .95),
                 bed=material(s, 'TruckBed', (.10, .11, .12), .62, .35),
                 steel=material(s, 'TruckSteel', (.28, .29, .30), .45, .6),
                 tyre=material(s, 'TruckTyre', (.02, .02, .022), .9, 0),
                 rim=material(s, 'TruckRim', (.72, .73, .74), .3, .8),
                 interior=material(s, 'TruckInterior', (.12, .12, .13), .8, 0),
                 seat=material(s, 'TruckSeat', (.18, .19, .22), .9, 0),
                 plate=material(s, 'TruckPlate', (.9, .9, .88), .5, 0),
                 glass=glass_material(s, 'TruckGlass'))
        m['headlight'], _ = emissive_material(s, 'TruckHeadlight', (1., .95, .85), 4.)
        m['taillight'], _ = emissive_material(s, 'TruckTaillight', (1., .05, .03), 5.)
        m['amber'], _ = emissive_material(s, 'TruckAmber', (1., .55, .05), 4.)
        m['beacon'], self.beacon_emissive = emissive_material(s, 'TruckBeacon', RISK_COLOURS[0], 8.)
        self.m = m
        UsdGeom.Xform.Define(s, root)
        self.wheels = []
        self.chassis()
        self.cab()
        self.flatbed()
        self.running_gear()
        self.lights()
        self.beacon_level = 0

    # ------------------------------------------------------------------ parts
    def chassis(self):
        s, m, r = self.stage, self.m, self.root
        for side in (-1, 1):
            box(s, f'{r}/Rail{side+1}', (-2.3, side * .40 - .05, .46), (2.3, side * .40 + .05, .56), m['steel'])
        box(s, f'{r}/Underbody', (.95, -.78, .42), (2.4, .78, CAB_FLOOR + .01), m['paint_dark'])
        box(s, f'{r}/BumperFront', (2.42, -.87, .50), (2.54, .87, .70), m['paint_dark'])
        box(s, f'{r}/BumperRear', (-2.28, -.72, .45), (-2.16, .72, .55), m['steel'])
        box(s, f'{r}/Plate', (-2.29, -.17, .56), (-2.28, .17, .64), m['plate'])
        box(s, f'{r}/FuelTank', (-.6, .5, .40), (.5, .80, .62), m['steel'])

    def cab(self):
        s, m, r = self.stage, self.m, self.root
        x0, x1 = CAB_X
        w = CAB_W / 2
        box(s, f'{r}/CabLower', (x0 + .02, -w, CAB_FLOOR), (x1, w, 1.17), m['paint'])          # doors, lower body
        box(s, f'{r}/Bonnet', (2.08, -w + .02, 1.15), (x1, w - .02, 1.21), m['paint'])
        box(s, f'{r}/Roof', (x0 + .06, -w, 1.97), (2.02, w, ROOF), m['paint'])
        box(s, f'{r}/CabFloor', (x0 + .02, -w + .02, CAB_FLOOR), (x1 - .3, w - .02, CAB_FLOOR + .04), m['interior'])
        # Rear wall with a window opening (four strips around a glass pane).
        box(s, f'{r}/RearWallLow', (x0, -w, 1.15), (x0 + .06, w, 1.38), m['paint'])
        box(s, f'{r}/RearWallTop', (x0, -w, 1.86), (x0 + .06, w, 1.99), m['paint'])
        for side in (-1, 1):
            box(s, f'{r}/RearWallSide{side+1}', (x0, min(side * w, side * .58), 1.38), (x0 + .06, max(side * w, side * .58), 1.86), m['paint'])
        box(s, f'{r}/RearWindow', (x0 + .025, -.58, 1.38), (x0 + .035, .58, 1.86), m['glass'])
        # Side walls: B pillar, sill, top rail, door glass.
        for side in (-1, 1):
            y0, y1 = sorted((side * (w - .03), side * w))
            box(s, f'{r}/Sill{side+1}', (x0 + .06, y0, 1.15), (2.06, y1, 1.31), m['paint'])
            box(s, f'{r}/TopRail{side+1}', (x0 + .06, y0, 1.92), (2.02, y1, 1.98), m['paint'])
            box(s, f'{r}/BPillar{side+1}', (x0 + .06, y0, 1.31), (x0 + .14, y1, 1.92), m['paint'])
            box(s, f'{r}/DoorGlass{side+1}', (x0 + .14, side * (w - .018) - .004, 1.31), (2.02, side * (w - .018) + .004, 1.92), m['glass'])
            # Mirror on an arm
            box(s, f'{r}/MirrorArm{side+1}', (2.0, min(side * w, side * (w + .22)), 1.52), (2.04, max(side * w, side * (w + .22)), 1.56), m['plastic'])
            box(s, f'{r}/Mirror{side+1}', (1.98, min(side * (w + .14), side * (w + .30)), 1.40), (2.06, max(side * (w + .14), side * (w + .30)), 1.66), m['plastic'])
            box(s, f'{r}/MirrorGlass{side+1}', (1.975, min(side * (w + .15), side * (w + .29)), 1.42), (1.98, max(side * (w + .15), side * (w + .29)), 1.64), m['chrome'])
            box(s, f'{r}/DoorHandle{side+1}', (1.40, min(side * w, side * (w + .02)), 1.22), (1.55, max(side * w, side * (w + .02)), 1.25), m['chrome'])
        # Raked windshield and A pillars: top edge further back than the bottom edge.
        tilt = -17.
        length = .82
        centre = (1.995, 0, 1.59)
        shape(s, f'{r}/Windshield', 'box', centre, (.03, 1.56, length), m['glass'], (0, tilt, 0))
        for side in (-1, 1):
            shape(s, f'{r}/APillar{side+1}', 'box', (centre[0], side * .80, centre[2]), (.07, .08, length), m['paint'], (0, tilt, 0))
        box(s, f'{r}/Cowl', (2.02, -w + .02, 1.17), (2.12, w - .02, 1.23), m['paint_dark'])
        # Interior
        box(s, f'{r}/Dash', (1.80, -w + .06, 1.17), (2.08, w - .06, 1.42), m['interior'])
        for side, yc in ((1, .46), (-1, -.46)):
            box(s, f'{r}/SeatCushion{side+1}', (.98, yc - .26, .98), (1.44, yc + .26, 1.08), m['seat'])
            box(s, f'{r}/SeatBack{side+1}', (.98, yc - .26, 1.08), (1.08, yc + .26, 1.68), m['seat'])
            box(s, f'{r}/Headrest{side+1}', (.99, yc - .12, 1.70), (1.07, yc + .12, 1.84), m['seat'])
        # Steering column and wheel; the wheel spins about its own axis inside a tilted xform.
        wx, wz = DRIVER['wheel']
        centre = (DRIVER['root'][0] + wx, DRIVER['root'][1], DRIVER['root'][2] + wz)
        tilt = DRIVER['wheel_tilt_deg']
        column = UsdGeom.Xform.Define(s, f'{r}/SteeringColumn').GetPrim()
        pose(column, centre, (0, 90 - tilt, 0))
        shape(s, f'{r}/SteeringColumn/Shaft', 'cylinder', (0, 0, .12), (.02, .02, .26), m['plastic'])
        self.wheel_xf = UsdGeom.Xform.Define(s, f'{r}/SteeringColumn/Wheel').GetPrim()
        pose(self.wheel_xf)
        rr = DRIVER['wheel_r']
        round_cylinder(s, f'{r}/SteeringColumn/Wheel/Rim', rr, .035, m['plastic'], 40, r_inner=rr - .035)
        round_cylinder(s, f'{r}/SteeringColumn/Wheel/Hub', .06, .05, m['plastic'], 24)
        for k in range(3):
            a = math.radians(90 + k * 120)
            shape(s, f'{r}/SteeringColumn/Wheel/Spoke{k}', 'box', (math.cos(a) * rr * .5, math.sin(a) * rr * .5, 0),
                  (rr * .9, .03, .025), m['plastic'], (0, 0, math.degrees(a)))
        # Cab guard (headache rack) at the front of the bed with the camera mast on top.
        gx = BED_X[1] - .06
        top = BED_TOP + 1.12
        for side in (-1, 1):
            box(s, f'{r}/GuardPost{side+1}', (gx, side * .72 - .03, BED_TOP), (gx + .06, side * .72 + .03, top), m['steel'])
        box(s, f'{r}/GuardTop', (gx, -.75, top - .06), (gx + .06, .75, top), m['steel'])
        box(s, f'{r}/GuardMesh', (gx + .02, -.69, BED_TOP + .30), (gx + .03, .69, top - .06), m['paint_dark'])
        for k in range(5):
            z = BED_TOP + .30 + k * .2
            box(s, f'{r}/GuardBar{k}', (gx + .005, -.69, z - .015), (gx + .055, .69, z + .015), m['steel'])
        cam = CAMERA['pos']
        box(s, f'{r}/CameraMast', (cam[0] - .025, -.025, top), (cam[0] + .025, .025, cam[2] - .02), m['steel'])
        box(s, f'{r}/CameraMastBrace', (gx, -.02, top - .5), (cam[0] + .02, .02, top + .02), m['steel'])
        box(s, f'{r}/D435iHousing', (cam[0] - .013, -.045, cam[2] - .0125), (cam[0] + .013, .045, cam[2] + .0125), m['silver'])

    def flatbed(self):
        s, m, r = self.stage, self.m, self.root
        x0, x1 = BED_X
        w = BED_W / 2
        box(s, f'{r}/BedFloor', (x0, -w, BED_TOP - .04), (x1, w, BED_TOP), m['bed'])
        box(s, f'{r}/BedFrame', (x0, -w + .08, .56), (x1, w - .08, BED_TOP - .04), m['paint_dark'])
        for side in (-1, 1):
            y0, y1 = sorted((side * (w - .03), side * w))
            box(s, f'{r}/SideGate{side+1}', (x0, y0, BED_TOP), (x1, y1, BED_TOP + .33), m['paint'])
            for k, x in enumerate(np.linspace(x0 + .02, x1 - .06, 4)):
                box(s, f'{r}/GatePost{side+1}_{k}', (x, y0 - .01 * side, BED_TOP), (x + .04, y1 + .01 * side, BED_TOP + .34), m['paint_dark'])
        box(s, f'{r}/Tailgate', (x0, -w, BED_TOP), (x0 + .03, w, BED_TOP + .33), m['paint'])
        box(s, f'{r}/TailgateRail', (x0 - .01, -w, BED_TOP + .31), (x0 + .04, w, BED_TOP + .34), m['paint_dark'])
        for side in (-1, 1):   # mud flaps behind the rear wheels
            box(s, f'{r}/MudFlap{side+1}', (AXLE_R - .55, side * .68 - .12, .30), (AXLE_R - .53, side * .68 + .12, .60), m['plastic'])

    def running_gear(self):
        s, m, r = self.stage, self.m, self.root
        for name, x, ys in (('F', AXLE_F, (-.70, .70)), ('R', AXLE_R, (-.80, -.57, .57, .80))):
            shape(s, f'{r}/Axle{name}', 'cylinder', (x, 0, WHEEL_R), (.045, .045, 1.7), m['steel'], (90, 0, 0))
            for k, y in enumerate(ys):
                prim = UsdGeom.Xform.Define(s, f'{r}/Wheel{name}{k}').GetPrim()
                pose(prim, (x, y, WHEEL_R), (90, 0, 0))
                round_cylinder(s, f'{r}/Wheel{name}{k}/Tyre', WHEEL_R, .20, m['tyre'], 56, r_inner=.215)
                round_cylinder(s, f'{r}/Wheel{name}{k}/Rim', .215, .205, m['rim'], 40)
                round_cylinder(s, f'{r}/Wheel{name}{k}/Hub', .085, .225, m['steel'], 24)
                for b in range(6):
                    a = b * 60.
                    shape(s, f'{r}/Wheel{name}{k}/Bolt{b}', 'cylinder',
                          (math.cos(math.radians(a)) * .12, math.sin(math.radians(a)) * .12, .0), (.012, .012, .232), m['plastic'])
                self.wheels.append(prim)
        for side in (-1, 1):   # front mudguards
            box(s, f'{r}/MudGuardF{side+1}', (AXLE_F - .42, side * .70 - .13, .60), (AXLE_F + .42, side * .70 + .13, .66), m['paint_dark'])

    def lights(self):
        s, m, r = self.stage, self.m, self.root
        x1 = CAB_X[1]
        for side in (-1, 1):
            box(s, f'{r}/Headlight{side+1}', (x1 - .005, min(side * .44, side * .74), .86), (x1 + .012, max(side * .44, side * .74), 1.02), m['headlight'])
            box(s, f'{r}/Indicator{side+1}', (x1 - .005, min(side * .44, side * .74), .76), (x1 + .010, max(side * .44, side * .74), .84), m['amber'])
            box(s, f'{r}/Taillight{side+1}', (BED_X[0] - .012, min(side * .56, side * .78), .60), (BED_X[0] + .005, max(side * .56, side * .78), .72), m['taillight'])
        box(s, f'{r}/Grille', (x1 - .005, -.40, .78), (x1 + .010, .40, 1.03), m['plastic'])
        for k in range(5):   # chrome grille bars
            z = .80 + k * .05
            box(s, f'{r}/GrilleBar{k}', (x1 + .010, -.40, z), (x1 + .018, .40, z + .015), m['chrome'])
        for k, y in enumerate((-.45, 0., .45)):   # roof marker lamps
            box(s, f'{r}/RoofLamp{k}', (1.92, y - .05, ROOF), (2.0, y + .05, ROOF + .04), m['amber'])
        self.beacon = shape(s, f'{r}/RiskBeacon', 'sphere', (1.45, 0, ROOF + .09), (.075, .075, .075), m['beacon'])
        shape(s, f'{r}/RiskBeaconBase', 'cylinder', (1.45, 0, ROOF + .02), (.07, .07, .04), m['plastic'])
        self.badges()

    def badges(self):
        """Hyundai badge on the grille, HYUNDAI lettering on the tailgate, PORTER II on the doors."""
        s, r = self.stage, self.root
        if not (ASSETS / 'hyundai_badge.png').exists():
            return
        badge = decal_material(s, 'DecalBadge', ASSETS / 'hyundai_badge.png')
        hyundai = decal_material(s, 'DecalHyundai', ASSETS / 'hyundai_text.png')
        porter = decal_material(s, 'DecalPorter', ASSETS / 'porter_text.png')
        plate = decal_material(s, 'DecalPlate', ASSETS / 'plate_text.png')
        x1 = CAB_X[1]
        # grille badge faces +x: texture right = -y (seen from the front, right is -y), up = +z
        decal(s, f'{r}/BadgeFront', (x1 + .022, 0, 1.065), (0, -.11, 0), (0, 0, .07), badge)
        box(s, f'{r}/BadgeMount', (x1 + .010, -.12, .99), (x1 + .018, .12, 1.14), self.m['paint'])
        # tailgate lettering faces -x: a viewer behind the truck has -y on their right
        decal(s, f'{r}/LetteringRear', (BED_X[0] - .002, 0, BED_TOP + .165), (0, -.42, 0), (0, 0, .105), hyundai)
        decal(s, f'{r}/BadgeRear', (BED_X[0] - .002, -.60, BED_TOP + .165), (0, -.085, 0), (0, 0, .053), badge)
        decal(s, f'{r}/PlateText', (-2.292, 0, .60), (0, -.16, 0), (0, 0, .04), plate)
        for side in (-1, 1):   # door lettering: a viewer beside the left door (+y) has -x on their right
            y = side * (CAB_W / 2 + .002)
            decal(s, f'{r}/LetteringDoor{side+1}', (1.30, y, 1.10), (-side * .17, 0, 0), (0, 0, .0425), porter)

    # ------------------------------------------------------------------ runtime
    def set_beacon(self, level):
        level = 0 if level is None else int(max(0, min(3, level)))
        if level != self.beacon_level:
            self.beacon_level = level
            self.beacon_emissive.Set(Gf.Vec3f(*[c * (8. if level else 3.) for c in RISK_COLOURS[level]]))

    def steer(self, angle_deg):
        pose(self.wheel_xf, rotation=(0, 0, -angle_deg))
        self.road_steer = max(-35., min(35., angle_deg / 14.))   # steering ratio ~14:1

    def roll_wheels(self, distance_m):
        """Roll every wheel for a travelled distance (positive = forward); front wheels also steer.

        USD rotateXYZ applies X, then Y, then Z about the parent axes: X(90) tilts the cylinder onto the
        axle (y), Y(spin) rolls it about the axle, Z(yaw) steers it about the vertical."""
        spin = math.degrees(distance_m / WHEEL_R) % 360
        yaw = getattr(self, 'road_steer', 0.)
        for prim in self.wheels:
            ops = UsdGeom.Xformable(prim).GetOrderedXformOps()
            ops[1].Set(Gf.Vec3f(90, spin, yaw if prim.GetName().startswith('WheelF') else 0.))
