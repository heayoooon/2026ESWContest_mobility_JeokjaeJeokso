"""Procedural Hyundai XCIENT-class heavy cargo truck (8x4, high-roof cab-over).

Truck frame: x forward, y left, z up, origin on the ground under the bed front.
Proportions follow The New XCIENT 8x4 cargo: ~11.5 m long, 2.5 m wide, 3.65 m cab roof, 520 mm wheel
radius, two steer axles under the cab and a rear tandem with dual wheels, flatbed 1.35 m above the ground.
Bodywork is built from filleted boxes and lathe solids (truck_geom) with MDL materials (materials_pbr):
clear-coat paint, chrome, glass with refraction, textured rubber and painted steel.
The hand cart's instrumented plate sits on the bed behind the cab (RIG; drawn RIG['scale'] times larger,
sensor arithmetic unchanged). Decals use the Hyundai logo the user supplied (assets/truck/*.png).
Earlier models: truck_porter.py (1-ton Porter), truck_xcient_v1.py (boxy first pass).
"""
import math

import numpy as np
from pxr import Gf, UsdGeom, UsdPhysics

import materials_pbr as M
from forest_scene import material, pose, shape
from truck_geom import lathe, rim_profile, rounded_box, tread_modulation, tyre_profile
from truck_porter import ASSETS, RISK_COLOURS, box, decal, decal_material

WHEEL_R = .52
WHEEL_W = .30
RIM_R = .33
BED_TOP = 1.35
BED_X = (-6.0, 1.45)
BED_W = 2.40
CAB_X = (1.65, 4.15)
CAB_W = 2.50
CAB_FLOOR = 1.45
ROOF = 3.65
AXLES = {'F1': 3.15, 'F2': 1.40, 'R1': -3.30, 'R2': -4.65}
FRONT_Y = (-1.0, 1.0)
REAR_Y = (-1.06, -.74, .74, 1.06)
BODY = (.58, .035, .045)           # XCIENT red (clear-coat paint)
CENTER_Z = 2.2                      # point the chase camera looks at
VIEW_DISTANCE = 16.                 # default camera distance
# Instrumented plate on the bed behind the cab, N (front, belt c1) row toward the cab.
RIG = {'x': -.55, 'y': 0., 'z': BED_TOP - .1125 * 3.2, 'yaw': 180., 'scale': 3.2}
DRIVER = {'root': (2.95, .80, CAB_FLOOR), 'hip_z': .50, 'hip_x': .0, 'knee': (.38, .52), 'pedal': (.66, .12),
          'wheel': (.46, .66), 'wheel_r': .23, 'wheel_tilt_deg': 42., 'hands_deg': 55., 'lean_deg': 8.}
CARGO_TOP_Z = BED_TOP + .825 * RIG['scale']
CAMERA = {'pos': (BED_X[1] - .05, 0., CARGO_TOP_Z + .6),
          'pitch_deg': math.degrees(math.atan2(CARGO_TOP_Z + .6 - (BED_TOP + .4 * RIG['scale']), BED_X[1] - .05 - RIG['x']))}


def rbox(stage, path, lo, hi, mat, radius=.02):
    return rounded_box(stage, path, lo, hi, radius, mat, segments=3)


class Truck:
    def __init__(self, stage, root, mats):
        s = stage
        self.stage, self.root = s, root
        m = dict(mats)
        m.update(paint=M.car_paint(s, 'TruckPaint', BODY),
                 paint_dark=M.pbr(s, 'TruckPaintDark', (.09, .095, .105), .45, .2, round_edges=.01),
                 plastic=M.pbr(s, 'TruckPlastic', (.022, .023, .025), .62, 0., round_edges=.008),
                 chrome=M.chrome(s, 'TruckChrome'),
                 chrome_dull=M.chrome(s, 'TruckChromeDull', .3, (.7, .71, .73)),
                 interior=M.pbr(s, 'TruckInterior', (.11, .11, .12), .8, 0.),
                 seat=M.pbr(s, 'TruckSeat', (.15, .16, .19), .9, 0.),
                 plate=M.pbr(s, 'TruckPlate', (.92, .92, .9), .5, 0.),
                 glass=M.glass(s, 'TruckGlass'),
                 lamp_glass=M.glass(s, 'TruckLampGlass', (.85, .9, .95), .18, .03))
        m['tyre'] = (M.base_material(s, 'TruckTyre', 'Plastics', 'Rubber_Textured', tint=(.16, .16, .17), scale=.25)
                     if M.available('Plastics', 'Rubber_Textured') else M.pbr(s, 'TruckTyre', (.02, .02, .022), .85))
        m['rim'] = M.chrome(s, 'TruckRim', .2, (.86, .87, .89))
        m['bed'] = (M.base_material(s, 'TruckBed', 'Metals', 'Steel_Carbon', tint=(.35, .36, .38), roughness=.55, scale=.5)
                    if M.available('Metals', 'Steel_Carbon') else M.pbr(s, 'TruckBed', (.10, .11, .12), .6, .35))
        m['steel'] = (M.base_material(s, 'TruckSteel', 'Metals', 'Steel_Carbon', tint=(.5, .51, .53), roughness=.45, scale=.4)
                      if M.available('Metals', 'Steel_Carbon') else M.pbr(s, 'TruckSteel', (.26, .27, .28), .45, .6))
        m['tank'] = (M.base_material(s, 'TruckTank', 'Metals', 'Steel_Stainless', roughness=.25, scale=.4)
                     if M.available('Metals', 'Steel_Stainless') else m['chrome_dull'])
        m['headlight'], _ = M.emissive(s, 'TruckHeadlight', (1., .96, .9), 600., (.8, .8, .78))
        m['drl'], _ = M.emissive(s, 'TruckDRL', (.95, .98, 1.), 900.)
        m['taillight'], _ = M.emissive(s, 'TruckTaillight', (1., .05, .03), 500., (.5, .05, .03))
        m['amber'], _ = M.emissive(s, 'TruckAmber', (1., .55, .05), 400., (.6, .3, .05))
        m['beacon'], self.beacon_shader = M.emissive(s, 'TruckBeacon', RISK_COLOURS[0], 300., (.4, .4, .4))
        self.m = m
        UsdGeom.Xform.Define(s, root)
        self.wheels = []
        self.wheel_home = []   # wheel centre (x, y, z) in the truck frame
        self.axles = []         # (axle prim, centre)
        self.suspended = (0., 0.)
        self.chassis()
        self.cab()
        self.flatbed()
        self.running_gear()
        self.lights()
        self.badges()
        self.beacon_level = 0

    # ------------------------------------------------------------------ parts
    def chassis(self):
        s, m, r = self.stage, self.m, self.root
        for side in (-1, 1):
            rbox(s, f'{r}/Rail{side+1}', (-6.3, side * .45 - .06, .85), (4.3, side * .45 + .06, 1.12), m['steel'], .012)
        for k, x in enumerate((-5.6, -4.0, -2.0, 0., 1.0, 2.4, 3.8)):
            box(s, f'{r}/CrossMember{k}', (x - .05, -.45, .88), (x + .05, .45, 1.08), m['steel'])
        # Fuel tank (lathe with rounded ends, right side), battery and tool boxes (left), guards.
        tank = UsdGeom.Xform.Define(s, f'{r}/FuelTank').GetPrim()
        pose(tank, (-1.4, -.95, .78), (0, 90, 0))
        lathe(s, f'{r}/FuelTank/Body', [(.0, -1.1), (.26, -1.1), (.33, -1.04), (.34, -1.0), (.34, 1.0), (.33, 1.04), (.26, 1.1), (.0, 1.1)],
              m['tank'], 48)
        for k, x in enumerate((-2.1, -.7)):
            box(s, f'{r}/TankStrap{k}', (x - .03, -1.31, .42), (x + .03, -.59, 1.14), m['steel'])
        rbox(s, f'{r}/BatteryBox', (-2.3, .55, .50), (-1.1, 1.15, 1.08), m['paint_dark'], .03)
        rbox(s, f'{r}/ToolBox', (-.8, .55, .55), (.3, 1.15, 1.08), m['paint_dark'], .03)
        for side in (-1, 1):
            rbox(s, f'{r}/SideGuard{side+1}', (-2.9, side * 1.16 - .03, .62), (0.9, side * 1.16 + .03, .70), m['steel'], .012)
            rbox(s, f'{r}/SideGuard{side+1}B', (-6.0, side * 1.16 - .03, .62), (-5.4, side * 1.16 + .03, .70), m['steel'], .012)
        rbox(s, f'{r}/RearUnderrun', (-6.45, -1.1, .50), (-6.3, 1.1, .64), m['steel'], .02)
        box(s, f'{r}/Plate', (-6.46, -.26, .70), (-6.45, .26, .82), m['plate'])
        for side in (-1, 1):   # rear fenders over the tandem and mud flaps
            rbox(s, f'{r}/RearFender{side+1}', (AXLES['R2'] - .75, min(side * .68, side * 1.22), WHEEL_R + .52),
                 (AXLES['R1'] + .75, max(side * .68, side * 1.22), WHEEL_R + .62), m['paint_dark'], .04)
            box(s, f'{r}/MudFlap{side+1}', (AXLES['R2'] - .75, min(side * .72, side * 1.2), .25),
                (AXLES['R2'] - .72, max(side * .72, side * 1.2), .85), m['plastic'])
        rbox(s, f'{r}/UnderCab', (CAB_X[0], -1.0, .95), (CAB_X[1] - .2, 1.0, CAB_FLOOR), m['paint_dark'], .03)
        rbox(s, f'{r}/AirCleaner', (CAB_X[0] - .5, -.9, BED_TOP + .3), (CAB_X[0] - .1, -.4, BED_TOP + 1.3), m['paint_dark'], .05)

    def cab(self):
        s, m, r = self.stage, self.m, self.root
        x0, x1 = CAB_X
        w = CAB_W / 2
        # Shell in filleted blocks that leave the glasshouse open: sleeper block, lower body (doors + grille
        # area), roof slab, A and B pillars. Glass panes close the openings so the interior stays visible.
        rounded_box(s, f'{r}/CabSleeper', (x0, -w, 1.0), (x0 + .92, w, ROOF), .16, m['paint'], segments=5)
        rounded_box(s, f'{r}/CabLower', (x0 + .9, -w, 1.0), (x1, w, 2.12), .10, m['paint'], segments=4)
        rounded_box(s, f'{r}/CabRoof', (x0 + .9, -w, 3.02), (x1, w, ROOF), .12, m['paint'], segments=4)
        for side in (-1, 1):
            rbox(s, f'{r}/APillar{side+1}', (x1 - .14, min(side * (w - .14), side * w), 2.0), (x1, max(side * (w - .14), side * w), 3.12), m['paint'], .03)
            rbox(s, f'{r}/BPillar{side+1}', (x0 + .88, min(side * (w - .06), side * w), 2.0), (x0 + 1.0, max(side * (w - .06), side * w), 3.12), m['paint'], .02)
        # Window recesses: dark trim boxes just outside the shell with the glass pane in front.
        for side in (-1, 1):
            box(s, f'{r}/DoorGlass{side+1}', (x0 + 1.0, min(side * (w - .03), side * (w - .024)), 2.12), (x1 - .14, max(side * (w - .03), side * (w - .024)), 3.02), m['glass'])
            box(s, f'{r}/DoorHandle{side+1}', (x0 + 1.0, min(side * (w + .005), side * (w + .03)), 1.75), (x0 + 1.18, max(side * (w + .005), side * (w + .03)), 1.79), m['chrome'])
            box(s, f'{r}/DoorSeam{side+1}', (x0 + .90, min(side * (w + .004), side * (w + .006)), 1.05), (x0 + .915, max(side * (w + .004), side * (w + .006)), 3.3), m['plastic'])
            for k, z in enumerate((.55, .98)):
                rbox(s, f'{r}/Step{side+1}_{k}', (x0 + 1.05, min(side * .95, side * (w - .02)), z), (x0 + 1.85, max(side * .95, side * (w - .02)), z + .06), m['paint_dark'], .015)
            rbox(s, f'{r}/StepBox{side+1}', (x0 + 1.0, min(side * .95, side * (w - .1)), .45), (x0 + 1.9, max(side * .95, side * (w - .1)), 1.0), m['paint_dark'], .03)
            # Mirror: two arms from the A pillar, tall housing with twin glass
            box(s, f'{r}/MirrorArm{side+1}', (x1 - .3, min(side * w, side * (w + .38)), 2.95), (x1 - .24, max(side * w, side * (w + .38)), 3.01), m['plastic'])
            box(s, f'{r}/MirrorArmB{side+1}', (x1 - .3, min(side * w, side * (w + .38)), 2.35), (x1 - .24, max(side * w, side * (w + .38)), 2.41), m['plastic'])
            rbox(s, f'{r}/Mirror{side+1}', (x1 - .36, min(side * (w + .26), side * (w + .40)), 2.25), (x1 - .18, max(side * (w + .26), side * (w + .40)), 3.05), m['plastic'], .035)
            box(s, f'{r}/MirrorGlass{side+1}', (x1 - .37, min(side * (w + .27), side * (w + .39)), 2.28), (x1 - .36, max(side * (w + .27), side * (w + .39)), 3.02), m['chrome'])
        # Windshield between the A pillars; chrome cowl strip below.
        box(s, f'{r}/Windshield', (x1 - .05, -w + .14, 2.12), (x1 - .035, w - .14, 3.02), m['glass'])
        box(s, f'{r}/Cowl', (x1 - .02, -w + .05, 2.06), (x1 + .03, w - .05, 2.12), m['chrome'])
        rbox(s, f'{r}/SunVisor', (x1 - .05, -w + .05, 3.08), (x1 + .32, w - .05, 3.30), m['paint'], .04)
        box(s, f'{r}/SunVisorLip', (x1 + .28, -w + .05, 3.06), (x1 + .34, w - .05, 3.32), m['plastic'])
        # Interior: dash, two seats, bunk; steering wheel on a tilted column (spins about its own axis).
        rbox(s, f'{r}/Dash', (x1 - .55, -w + .1, CAB_FLOOR + .55), (x1 - .1, w - .1, 2.08), m['interior'], .04)
        box(s, f'{r}/Bunk', (x0 + .06, -w + .1, CAB_FLOOR + .6), (x0 + .8, w - .1, CAB_FLOOR + .75), m['seat'])
        box(s, f'{r}/CabFloor', (x0 + .05, -w + .05, CAB_FLOOR), (x1 - .1, w - .05, CAB_FLOOR + .04), m['interior'])
        for side, yc in ((1, .80), (-1, -.80)):
            rbox(s, f'{r}/SeatCushion{side+1}', (DRIVER['root'][0] - .14, yc - .28, CAB_FLOOR + .36), (DRIVER['root'][0] + .34, yc + .28, CAB_FLOOR + .48), m['seat'], .04)
            rbox(s, f'{r}/SeatBack{side+1}', (DRIVER['root'][0] - .16, yc - .28, CAB_FLOOR + .48), (DRIVER['root'][0] - .04, yc + .28, CAB_FLOOR + 1.15), m['seat'], .04)
            rbox(s, f'{r}/Headrest{side+1}', (DRIVER['root'][0] - .15, yc - .13, CAB_FLOOR + 1.17), (DRIVER['root'][0] - .05, yc + .13, CAB_FLOOR + 1.32), m['seat'], .03)
        wx, wz = DRIVER['wheel']
        centre = (DRIVER['root'][0] + wx, DRIVER['root'][1], DRIVER['root'][2] + wz)
        tilt = DRIVER['wheel_tilt_deg']
        column = UsdGeom.Xform.Define(s, f'{r}/SteeringColumn').GetPrim()
        pose(column, centre, (0, 90 - tilt, 0))
        shape(s, f'{r}/SteeringColumn/Shaft', 'cylinder', (0, 0, .14), (.025, .025, .3), m['plastic'])
        self.wheel_xf = UsdGeom.Xform.Define(s, f'{r}/SteeringColumn/Wheel').GetPrim()
        pose(self.wheel_xf)
        rr = DRIVER['wheel_r']
        lathe(s, f'{r}/SteeringColumn/Wheel/Rim', [(rr - .035, -.02), (rr, -.02), (rr, .02), (rr - .035, .02)], m['plastic'], 40)
        lathe(s, f'{r}/SteeringColumn/Wheel/Hub', [(0., -.025), (.07, -.025), (.07, .025), (0., .025)], m['plastic'], 24)
        for k in range(3):
            a = math.radians(90 + k * 120)
            shape(s, f'{r}/SteeringColumn/Wheel/Spoke{k}', 'box', (math.cos(a) * rr * .5, math.sin(a) * rr * .5, 0),
                  (rr * .9, .035, .028), m['plastic'], (0, 0, math.degrees(a)))
        # Cab guard behind the cab with the D435i mast; roof corner lamps like the reference photo.
        gx = BED_X[1] - .08
        top = BED_TOP + 2.0
        for side in (-1, 1):
            rbox(s, f'{r}/GuardPost{side+1}', (gx, side * 1.05 - .04, BED_TOP), (gx + .08, side * 1.05 + .04, top), m['steel'], .01)
        rbox(s, f'{r}/GuardTop', (gx, -1.1, top - .08), (gx + .08, 1.1, top), m['steel'], .01)
        box(s, f'{r}/GuardMesh', (gx + .03, -1.0, BED_TOP + .35), (gx + .045, 1.0, top - .08), m['paint_dark'])
        for k in range(7):
            z = BED_TOP + .35 + k * .24
            box(s, f'{r}/GuardBar{k}', (gx + .005, -1.0, z - .02), (gx + .075, 1.0, z + .02), m['steel'])
        cam = CAMERA['pos']
        # The camera looks back and down at the cargo (-x): housing and mast sit behind the lens (+x, cab side).
        # With the mast under the lens its top filled the lower half of the D435i view and of the simulated depth.
        box(s, f'{r}/CameraMast', (cam[0] + .01, -.03, top), (cam[0] + .07, .03, cam[2] - .0125), m['steel'])
        box(s, f'{r}/CameraMastBrace', (gx, -.025, top - .6), (cam[0] + .07, .025, top + .02), m['steel'])
        box(s, f'{r}/D435iHousing', (cam[0] + .002, -.045, cam[2] - .0125), (cam[0] + .028, .045, cam[2] + .0125), m['silver'])
        for side in (-1, 1):
            shape(s, f'{r}/RoofLampDome{side+1}', 'sphere', (x1 - .35, side * (w - .2), ROOF + .11), (.11, .11, .11), m['chrome'])
            shape(s, f'{r}/RoofLampBase{side+1}', 'cylinder', (x1 - .35, side * (w - .2), ROOF + .03), (.09, .09, .06), m['plastic'])

    def flatbed(self):
        s, m, r = self.stage, self.m, self.root
        x0, x1 = BED_X
        w = BED_W / 2
        box(s, f'{r}/BedFloor', (x0, -w, BED_TOP - .05), (x1, w, BED_TOP), m['bed'])
        rbox(s, f'{r}/BedFrame', (x0, -w + .1, 1.12), (x1, w - .1, BED_TOP - .05), m['paint_dark'], .02)
        for side in (-1, 1):
            y0, y1 = sorted((side * (w - .04), side * w))
            rbox(s, f'{r}/SideGate{side+1}', (x0, y0, BED_TOP), (x1, y1, BED_TOP + .5), m['paint'], .012)
            for k, x in enumerate(np.linspace(x0 + .03, x1 - .08, 6)):
                rbox(s, f'{r}/GatePost{side+1}_{k}', (x, y0 - .012 * side, BED_TOP), (x + .05, y1 + .012 * side, BED_TOP + .52), m['paint_dark'], .01)
            rbox(s, f'{r}/GateRail{side+1}', (x0, y0 - .01 * side, BED_TOP + .47), (x1, y1 + .01 * side, BED_TOP + .52), m['paint_dark'], .01)
        rbox(s, f'{r}/Tailgate', (x0, -w, BED_TOP), (x0 + .04, w, BED_TOP + .5), m['paint'], .012)
        rbox(s, f'{r}/TailgateRail', (x0 - .01, -w, BED_TOP + .47), (x0 + .05, w, BED_TOP + .52), m['paint_dark'], .01)
        # Invisible colliders of the load bed (part of the kinematic truck): cargo that slides off the instrumented
        # plate lands on the bed floor and stops at the side gates, the tailgate or the cab guard instead of
        # falling through the truck onto the road.
        gx = BED_X[1] - .08
        for name, lo, hi in (('Floor', (x0, -w, BED_TOP - .05), (x1, w, BED_TOP)),
                             ('Left', (x0, w - .04, BED_TOP), (x1, w, BED_TOP + .52)),
                             ('Right', (x0, -w, BED_TOP), (x1, -w + .04, BED_TOP + .52)),
                             ('Tail', (x0, -w, BED_TOP), (x0 + .04, w, BED_TOP + .52)),
                             ('Guard', (gx, -w, BED_TOP), (gx + .08, w, BED_TOP + 2.0))):
            lo, hi = np.array(lo, float), np.array(hi, float)
            cube = UsdGeom.Cube.Define(s, f'{r}/BedCollider{name}')
            cube.CreateSizeAttr(1.)
            xf = UsdGeom.Xformable(cube)
            xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, (lo + hi) / 2)))
            xf.AddScaleOp().Set(Gf.Vec3f(*map(float, hi - lo)))
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            cube.CreatePurposeAttr('guide')            # never rendered

    def running_gear(self):
        s, m, r = self.stage, self.m, self.root
        tread = tread_modulation(blocks=56, groove_depth=.009, radius=WHEEL_R)
        for name, x in AXLES.items():
            ys = FRONT_Y if name.startswith('F') else REAR_Y
            lathe_axle = UsdGeom.Xform.Define(s, f'{r}/Axle{name}').GetPrim()
            self.axles.append((lathe_axle, (x, 0., WHEEL_R)))
            pose(lathe_axle, (x, 0, WHEEL_R), (90, 0, 0))
            lathe(s, f'{r}/Axle{name}/Tube', [(.07, -1.1), (.07, 1.1)], m['steel'], 24)
            for k, y in enumerate(ys):
                prim = UsdGeom.Xform.Define(s, f'{r}/Wheel{name}_{k}').GetPrim()
                pose(prim, (x, y, WHEEL_R), (90, 0, 0))
                lathe(s, f'{r}/Wheel{name}_{k}/Tyre', tyre_profile(WHEEL_R, WHEEL_W, RIM_R), m['tyre'], 112, radius_fn=tread, uv_scale=.3)
                lathe(s, f'{r}/Wheel{name}_{k}/Rim', rim_profile(RIM_R + .005, WHEEL_W - .02), m['rim'], 56)
                lathe(s, f'{r}/Wheel{name}_{k}/Hub', [(0., -.09), (.13, -.09), (.14, -.06), (.14, .06), (.13, .09), (0., .09)], m['chrome'], 32)
                for b in range(10):
                    a = b * 36.
                    shape(s, f'{r}/Wheel{name}_{k}/Bolt{b}', 'cylinder',
                          (math.cos(math.radians(a)) * .2, math.sin(math.radians(a)) * .2, .0), (.015, .015, WHEEL_W - .02), m['plastic'])
                self.wheels.append(prim)
                self.wheel_home.append((x, y, WHEEL_R))
        for side in (-1, 1):   # front mudguards under the cab for both steer axles
            for name in ('F1', 'F2'):
                rbox(s, f'{r}/MudGuard{name}{side+1}', (AXLES[name] - .62, min(side * .75, side * 1.25), WHEEL_R + .42),
                     (AXLES[name] + .62, max(side * .75, side * 1.25), WHEEL_R + .50), m['paint_dark'], .03)

    def lights(self):
        s, m, r = self.stage, self.m, self.root
        x1 = CAB_X[1]
        w = CAB_W / 2
        # Grille: dark recess with chrome chevron slats, chrome frame; the emblem goes on top (badges()).
        box(s, f'{r}/GrilleBack', (x1 - .01, -.92, 1.18), (x1 + .012, .92, 1.98), m['plastic'])
        for k in range(4):
            z = 1.27 + k * .18
            hw = .86 - k * .07
            rbox(s, f'{r}/GrilleBar{k}', (x1 + .012, -hw, z), (x1 + .05, hw, z + .06), m['chrome'], .012)
        rbox(s, f'{r}/GrilleFrameL', (x1 + .005, -.96, 1.15), (x1 + .04, -.90, 2.0), m['chrome'], .012)
        rbox(s, f'{r}/GrilleFrameR', (x1 + .005, .90, 1.15), (x1 + .04, .96, 2.0), m['chrome'], .012)
        for side in (-1, 1):
            lo_y, hi_y = min(side * .98, side * 1.2), max(side * .98, side * 1.2)
            box(s, f'{r}/HeadlightHousing{side+1}', (x1 - .03, lo_y - .02, 1.10), (x1 + .01, hi_y + .02, 1.70), m['plastic'])
            box(s, f'{r}/Headlight{side+1}', (x1 + .005, lo_y, 1.22), (x1 + .02, hi_y, 1.62), m['headlight'])
            box(s, f'{r}/DRL{side+1}', (x1 + .012, lo_y, 1.62), (x1 + .03, hi_y + .02, 1.68), m['drl'])
            box(s, f'{r}/Indicator{side+1}', (x1 + .005, lo_y, 1.12), (x1 + .02, hi_y, 1.2), m['amber'])
            box(s, f'{r}/FogLight{side+1}', (x1 + .28, min(side * .78, side * 1.0), .80), (x1 + .3, max(side * .78, side * 1.0), .92), m['headlight'])
            box(s, f'{r}/Taillight{side+1}', (BED_X[0] - .012, min(side * .80, side * 1.12), .74), (BED_X[0] + .005, max(side * .80, side * 1.12), .92), m['taillight'])
            box(s, f'{r}/SideMarker{side+1}', (BED_X[0] + 1.0, min(side * w, side * (w + .01)), 1.2), (BED_X[0] + 1.3, max(side * w, side * (w + .01)), 1.26), m['amber'])
        # Bumper with the lower air dam
        rbox(s, f'{r}/Bumper', (x1 - .1, -w, .62), (x1 + .32, w, 1.0), m['paint_dark'], .06)
        rbox(s, f'{r}/BumperUpper', (x1 - .05, -w, 1.0), (x1 + .05, w, 1.18), m['paint'], .03)
        for k, y in enumerate((-.9, -.45, 0., .45, .9)):   # roof marker lamps
            box(s, f'{r}/RoofLamp{k}', (x1 - .18, y - .06, ROOF), (x1 - .06, y + .06, ROOF + .05), m['amber'])
        self.beacon = shape(s, f'{r}/RiskBeacon', 'sphere', (CAB_X[0] + .6, 0, ROOF + .12), (.1, .1, .1), m['beacon'])
        shape(s, f'{r}/RiskBeaconBase', 'cylinder', (CAB_X[0] + .6, 0, ROOF + .03), (.09, .09, .06), m['plastic'])

    def badges(self):
        """Hyundai emblem (from the supplied logo) on the grille and the bunk wall, HYUNDAI wordmark on the
        tailgate and the cab sides, XCIENT on the sun visor, model code on the doors."""
        s, r = self.stage, self.root
        if not (ASSETS / 'hyundai_emblem_chrome.png').exists():
            return
        emblem = decal_material(s, 'DecalEmblem', ASSETS / 'hyundai_emblem_chrome.png')
        wordmark = decal_material(s, 'DecalWordmark', ASSETS / 'hyundai_wordmark_white.png')
        xcient = decal_material(s, 'DecalXcient', ASSETS / 'xcient_text.png')
        model = decal_material(s, 'DecalModel', ASSETS / 'model_text.png')
        plate = decal_material(s, 'DecalPlate', ASSETS / 'plate_text.png')
        x1 = CAB_X[1]
        w = CAB_W / 2
        # Grille emblem faces +x: a viewer in front of the truck (looking toward -x) has +y on their right.
        rbox(s, f'{r}/EmblemMount', (x1 + .012, -.36, 1.82), (x1 + .05, .36, 2.0), self.m['plastic'], .01)
        decal(s, f'{r}/EmblemFront', (x1 + .053, 0, 1.905), (0, .32, 0), (0, 0, .143), emblem)
        decal(s, f'{r}/VisorText', (x1 + .342, 0, 3.19), (0, .60, 0), (0, 0, .15), xcient)
        # Tailgate: wordmark + emblem face -x (viewer behind has -y on the right).
        decal(s, f'{r}/WordmarkRear', (BED_X[0] - .002, .15, BED_TOP + .25), (0, -.55, 0), (0, 0, .128), wordmark)
        decal(s, f'{r}/EmblemRear', (BED_X[0] - .002, -.85, BED_TOP + .25), (0, -.17, 0), (0, 0, .076), emblem)
        decal(s, f'{r}/PlateText', (-6.462, 0, .76), (0, -.25, 0), (0, 0, .055), plate)
        # Cab sides (the shell is filleted, so decals sit a little outside the nominal face).
        for side in (-1, 1):
            y = side * (w + .004)
            decal(s, f'{r}/WordmarkSide{side+1}', (CAB_X[0] + .42, y, 3.25), (-side * .30, 0, 0), (0, 0, .07), wordmark)
            decal(s, f'{r}/EmblemSide{side+1}', (CAB_X[0] + .42, y, 2.82), (-side * .17, 0, 0), (0, 0, .076), emblem)
            decal(s, f'{r}/ModelText{side+1}', (CAB_X[0] + 1.75, y, 1.55), (-side * .28, 0, 0), (0, 0, .07), model)

    # ------------------------------------------------------------------ runtime
    def set_beacon(self, level):
        level = 0 if level is None else int(max(0, min(3, level)))
        if level != self.beacon_level:
            self.beacon_level = level
            self.beacon_shader.GetInput('emissive_color').Set(Gf.Vec3f(*RISK_COLOURS[level]))
            self.beacon_shader.GetInput('emissive_intensity').Set(900. if level else 300.)

    def steer(self, angle_deg):
        pose(self.wheel_xf, rotation=(0, 0, -angle_deg))
        self.road_steer = max(-30., min(30., angle_deg / 18.))   # heavy-truck steering ratio ~18:1

    def suspend(self, pitch_deg, roll_deg):
        """Body pitch / roll from braking, accelerating and cornering is a rotation of the whole vehicle about
        its ground point; move every wheel and axle along the body's vertical (suspension travel) so the tyres
        stay on the road instead of the rear end sinking into it."""
        if abs(pitch_deg - self.suspended[0]) < .01 and abs(roll_deg - self.suspended[1]) < .01:
            return
        self.suspended = (pitch_deg, roll_deg)
        p, q = math.radians(pitch_deg), math.radians(roll_deg)
        # Ry(pitch) @ Rx(roll), the body rotation of rotateXYZ(roll, pitch, yaw) without the yaw
        row_z = (-math.sin(p), math.cos(p) * math.sin(q), math.cos(p) * math.cos(q))
        up_z = row_z[2]
        for prim, (x, y, z) in list(zip(self.wheels, self.wheel_home)) + [(a, c) for a, c in self.axles]:
            height = row_z[0] * x + row_z[1] * y + row_z[2] * z
            dz = (WHEEL_R - height) / up_z
            UsdGeom.Xformable(prim).GetOrderedXformOps()[0].Set(Gf.Vec3d(x, y, z + dz))

    def roll_wheels(self, distance_m):
        """Roll every wheel for a travelled distance (positive = forward); both steer axles also steer.

        USD rotateXYZ applies X, then Y, then Z about the parent axes: X(90) tilts the lathe axis onto the
        axle (y), Y(spin) rolls it about the axle, Z(yaw) steers it about the vertical."""
        spin = math.degrees(distance_m / WHEEL_R) % 360
        yaw = getattr(self, 'road_steer', 0.)
        for prim in self.wheels:
            ops = UsdGeom.Xformable(prim).GetOrderedXformOps()
            ops[1].Set(Gf.Vec3f(90, spin, yaw if prim.GetName().startswith('WheelF') else 0.))
