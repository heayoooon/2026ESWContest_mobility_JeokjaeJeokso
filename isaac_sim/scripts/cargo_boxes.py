"""Cargo as closed boxes, made exactly like the display Pi's digital twin does it.

The display (display/twin/server, team repo) does not draw the raw depth: twin_server.py runs
twin_view.StableBoxMesh (stable_twin.StableTwin / StableHeight: a fixed reference built from the first second,
only changes that last > 0.5 s are taken over) -> cargo_model.CargoModel (2.5-D boxes fitted to the horizontal
top planes, tracked over time) -> stack_layers.StackLayers (stacked boxes split into layers, the cargo floor).
twin_view itself needs pygame / OpenGL, so `DisplayBoxes` repeats StableBoxMesh.build (mode 0) around the
team's own pure-numpy modules, copied unchanged into IsaacTwin/team_display/ by sync_team_analysis.py.

`BoxDrawer` shows the result as USD cubes / cylinders on the rig (same placement as measured_surface).
Display box frame: x = camera-world x (rig +y), y = grid y (rig +x after GRID_Y_TO_RIG_X), z = height above the
room floor (ref_h) - the plate top is PLATE_ABOVE_FLOOR above it.
"""
import io
import json
import sys
from pathlib import Path

import numpy as np

TEAM_DISPLAY = Path(__file__).resolve().parents[1] / 'team_display'
if not (TEAM_DISPLAY / 'cargo_model.py').exists():
    # inside the team repo (isaac_sim/) the display code sits next door - use it when nothing was synced
    TEAM_DISPLAY = Path(__file__).resolve().parents[2] / 'display' / 'twin' / 'server'
if str(TEAM_DISPLAY) not in sys.path:
    sys.path.insert(0, str(TEAM_DISPLAY))
from cargo_model import CargoModel          # noqa: E402  (team display code)
from stable_twin import StableHeight, StableTwin   # noqa: E402
from stack_layers import StackLayers        # noqa: E402

REFIT = 3            # twin_view.StableBoxMesh: frames refitted after the input changed
REFIT_FRAC = .08     # share of depth pixels that must change by > 25 mm before refitting
MAX_BOXES = 12


def team_display_version():
    try:
        v = json.loads((TEAM_DISPLAY / 'TEAM_VERSION.json').read_text(encoding='utf-8'))
        return f"{v.get('branch')}@{str(v.get('commit'))[:7]}"
    except (OSError, ValueError):
        return 'unknown'


def frame_from_parts(header, depth, jpeg, height_mm, colour):
    """Display-style frame dict (twin_view.Receiver.decode) from decoded parts."""
    from PIL import Image
    g = header['grid']
    h = np.asarray(height_mm)
    hm = np.where(h == g.get('nodata', -32768), np.nan, h.astype(np.float32) / 1000.)
    rgb = np.asarray(Image.open(io.BytesIO(jpeg)).convert('RGB'))
    return {'hdr': header, 'h': hm, 'col': np.asarray(colour, np.uint8), 'depth': np.asarray(depth, np.uint16), 'rgb': rgb}


class DisplayBoxes:
    """twin_view.StableBoxMesh.build (mode 0) + twin_server's StackLayers step, without the GL parts."""

    def __init__(self):
        # One StackLayers for good, as on the display: its own reset (new run / time going back) keeps the cargo
        # floor and the last layer shape ('carry'), so stacked cargo is not shown as one block after a switch.
        self.layers = StackLayers()
        self.reset()

    def reset(self):
        self.model = CargoModel();self.twin = StableTwin();self.sh = StableHeight()
        self.stamp = None;self.ready = False;self.refit = 0;self.fit_depth = None
        self.mesh_boxes = [];self.base = .34;self.base_locked = None
        self.boxes = [];self.floor = None

    def _fit(self, frame):
        self.mesh_boxes = self.model.update(frame)
        if self.mesh_boxes:
            self.base = min(b['bottom'] for b in self.mesh_boxes) - .001

    def update(self, frame):
        """frame: display frame dict (hdr, h, col, depth, rgb). Returns (boxes, floor height above the room floor)."""
        hdr = frame['hdr']
        stamp = (hdr.get('session'), hdr['run'])
        if self.stamp is not None and (stamp != self.stamp[0] or hdr['t_run'] < self.stamp[1]):
            self.reset()
        self.stamp = (stamp, hdr['t_run'])
        acc = hdr.get('cam_accel')
        o1 = self.twin.update(frame['depth'], frame['rgb'], acc)
        o2 = self.sh.update(frame['h'], frame['col'].astype(np.float32), acc)
        if o1 is None or o2 is None:                 # the first second builds the reference: fit the raw frame
            self._fit(frame)
        else:
            d, c, _ = o1;h, col, _ = o2
            moved = 0.
            if self.fit_depth is not None:
                any_ = (d > 0) | (self.fit_depth > 0)
                moved = float((np.abs(d - self.fit_depth)[any_] > 25).mean()) if any_.any() else 0.
            if self.fit_depth is None or moved > REFIT_FRAC:
                self.refit = REFIT;self.fit_depth = d.copy()
            if self.refit:
                sf = dict(frame)
                sf['depth'] = np.clip(d, 0, 65535).astype(np.uint16);sf['rgb'] = np.clip(c, 0, 255).astype(np.uint8)
                sf['h'] = h;sf['col'] = np.clip(col, 0, 255).astype(np.uint8)
                self._fit(sf)
                if self.ready:
                    self.base = self.base_locked
                self.refit -= 1
                if not self.ready:
                    self.base_locked = self.base
                self.ready = True
        try:                                         # twin_server: StackLayers reads the frame as received
            self.boxes, floor = self.layers.update(frame, self.mesh_boxes)
        except Exception:                            # twin_server does the same: fall back to the plain boxes
            self.boxes, floor = list(self.mesh_boxes or []), None
        self.floor = floor if floor is not None else self.base
        return self.boxes, self.floor


class BoxDrawer:
    """USD boxes / cylinders under the rig; unused ones are hidden."""

    def __init__(self, stage, path, mat, plate_above_floor, deck_top, grid_y_to_rig_x):
        from pxr import Gf, UsdGeom
        self.Gf, self.UsdGeom = Gf, UsdGeom
        self.stage, self.path = stage, path
        self.off = (plate_above_floor, deck_top, grid_y_to_rig_x)
        self.root = UsdGeom.Xform.Define(stage, path)
        self.prims = []
        from forest_scene import bind
        for i in range(MAX_BOXES):
            cube = UsdGeom.Cube.Define(stage, f'{path}/Box{i}');cube.CreateSizeAttr(1.)
            cyl = UsdGeom.Cylinder.Define(stage, f'{path}/Can{i}');cyl.CreateAxisAttr('X');cyl.CreateHeightAttr(1.);cyl.CreateRadiusAttr(.5)
            for g in (cube, cyl):
                bind(g.GetPrim(), mat)
                x = UsdGeom.Xformable(g)
                x.ClearXformOpOrder()
                x.AddTranslateOp();x.AddScaleOp()
                g.CreateDisplayColorAttr([(.6, .6, .6)])
                UsdGeom.Imageable(g).MakeInvisible()
            self.prims.append((cube, cyl, [None, None]))
        self.shown = 0
        self.visible = True

    def set_visible(self, on):
        self.visible = on
        img = self.UsdGeom.Imageable(self.root.GetPrim())
        img.MakeVisible() if on else img.MakeInvisible()

    def rig_box(self, b, floor):
        """Display box -> rig centre and size. Boxes standing on the cargo floor start on the deck."""
        above, deck, gx = self.off
        z = lambda v: v - above + deck
        bottom = b['bottom']
        if floor is not None and abs(bottom - floor) < .02:
            bottom = min(bottom, floor)
        lo, hi = max(deck + .001, z(bottom)), max(deck + .01, z(b['top']))
        cx, cy = (b['y0'] + b['y1']) / 2 + gx, (b['x0'] + b['x1']) / 2
        return (cx, cy, (lo + hi) / 2), (max(b['y1'] - b['y0'], .005), max(b['x1'] - b['x0'], .005), max(hi - lo, .005))

    def draw(self, boxes, floor):
        Gf = self.Gf
        n = 0
        # The lowest boxes stand on the plate. Without a measured cargo floor the display puts them on a fixed
        # .365 m 'foundation' (its camera's reference) - here they go down to the deck instead of floating.
        lowest = min((b['bottom'] for b in boxes), default=None)
        for b in boxes[:MAX_BOXES]:
            cube, cyl, _ = self.prims[n]
            use, other = (cyl, cube) if b.get('shape') == 'cylinder' else (cube, cyl)
            if b.get('shape') != 'cylinder' and lowest is not None and b['bottom'] - lowest < .02:
                b = dict(b, bottom=self.off[0])
            centre, size = self.rig_box(b, floor)
            ops = self.UsdGeom.Xformable(use).GetOrderedXformOps()
            ops[0].Set(Gf.Vec3d(*map(float, centre)));ops[1].Set(Gf.Vec3f(*map(float, size)))
            colour = np.clip(np.asarray(b.get('color', (.6, .6, .6)), float), 0, 1)
            colour = np.clip(np.power(colour, .7) * 1.1, 0, 1) ** 2.2        # display's tone, then sRGB -> linear
            if b.get('held'):
                colour = colour * .65 + .05
            use.GetDisplayColorAttr().Set([tuple(map(float, colour))])
            self.UsdGeom.Imageable(use).MakeVisible();self.UsdGeom.Imageable(other).MakeInvisible()
            n += 1
        for i in range(n, self.shown):
            for g in self.prims[i][:2]:
                self.UsdGeom.Imageable(g).MakeInvisible()
        self.shown = n
