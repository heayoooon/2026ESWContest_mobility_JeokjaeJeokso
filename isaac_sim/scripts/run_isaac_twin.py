import json
import math
import time
from pathlib import Path

import numpy as np
from isaacsim import SimulationApp

LIGHT_KIT = (
    Path.home()
    / "anaconda3"
    / "envs"
    / "env_isaacsim42"
    / "Lib"
    / "site-packages"
    / "isaacsim"
    / "apps"
    / "isaac_twin_light.kit"
)

simulation_app = SimulationApp(
    {"headless": False, "renderer": "RayTracedLighting"},
    experience=str(LIGHT_KIT),
)

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid, VisualCuboid, VisualSphere
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from pxr import Gf, UsdGeom


ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = json.loads((ROOT / "scenarios.json").read_text(encoding="utf-8"))["cases"]
CART_USD = ROOT / "assets" / "cart.usd"
CARGO_USD = ROOT / "assets" / "cargo.usd"
SURFACE_MANIFEST = ROOT / "assets" / "surfaces" / "manifest.json"
SURFACE_BY_CASE = {
    "constant_drive": "cruise",
    "hard_start_stop": "brake",
    "right_turn": "turn",
    "cargo_drop": "drop",
}

CASE_COLORS = {
    "constant_drive": np.array([0.2, 0.7, 1.0]),
    "hard_start_stop": np.array([1.0, 0.4, 0.1]),
    "right_turn": np.array([0.5, 1.0, 0.3]),
    "cargo_drop": np.array([1.0, 0.2, 0.2]),
}


def add_reference(stage, path: Path, prim_path: str, scale: float = 1.0) -> None:
    create_prim(prim_path=prim_path, usd_path=str(path))
    xform = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path))
    set_xform_op(xform, UsdGeom.XformOp.TypeScale, Gf.Vec3f(scale, scale, scale))


def set_xform_op(xform: UsdGeom.Xformable, op_type, value) -> None:
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == op_type:
            op.Set(value)
            return
    if op_type == UsdGeom.XformOp.TypeTranslate:
        xform.AddTranslateOp().Set(value)
    elif op_type == UsdGeom.XformOp.TypeScale:
        xform.AddScaleOp().Set(value)
    else:
        raise ValueError(f"Unsupported xform op type: {op_type}")


def set_visible(stage, prim_path: str, visible: bool) -> None:
    imageable = UsdGeom.Imageable(stage.GetPrimAtPath(prim_path))
    if visible:
        imageable.MakeVisible()
    else:
        imageable.MakeInvisible()


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def frame_at(frames: list[dict], t: float) -> dict:
    if t <= frames[0]["t"]:
        return frames[0]
    if t >= frames[-1]["t"]:
        return frames[-1]
    lo = 0
    hi = len(frames) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if frames[mid]["t"] <= t:
            lo = mid
        else:
            hi = mid
    a = frames[lo]
    b = frames[hi]
    span = max(b["t"] - a["t"], 1e-6)
    u = (t - a["t"]) / span
    return {key: lerp(float(a[key]), float(b[key]), u) for key in a}


def set_pose(obj, position, roll_deg, pitch_deg, yaw_deg=0.0):
    quat = euler_angles_to_quat(
        [math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)]
    )
    obj.set_world_pose(position=position, orientation=quat)


def main() -> None:
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # Keep the scanned assets in the stage as visual references. The simple
    # proxy bodies below carry the animated log replay so it stays responsive.
    add_reference(world.stage, CART_USD, "/World/ScanCart", 0.002)
    add_reference(world.stage, CARGO_USD, "/World/ScanCargo", 0.002)

    surface_paths = {}
    if SURFACE_MANIFEST.exists():
        for item in json.loads(SURFACE_MANIFEST.read_text(encoding="utf-8")):
            prim_path = f"/World/D435iSurface/{item['id']}"
            add_reference(world.stage, Path(item["usd"]), prim_path, 1.0)
            set_xform_op(
                UsdGeom.Xformable(world.stage.GetPrimAtPath(prim_path)),
                UsdGeom.XformOp.TypeTranslate,
                Gf.Vec3f(0.08, -0.18, 0.62),
            )
            surface_paths[item["id"]] = prim_path
            set_visible(world.stage, prim_path, False)

    cart = world.scene.add(
        FixedCuboid(
            prim_path="/World/CartProxy",
            name="cart_proxy",
            position=[0, 0, 0.18],
            scale=[1.6, 1.05, 0.08],
            color=np.array([0.06, 0.08, 0.09]),
        )
    )
    cargo = world.scene.add(
        DynamicCuboid(
            prim_path="/World/CargoProxy",
            name="cargo_proxy",
            position=[0.18, 0.0, 0.55],
            scale=[0.85, 0.58, 0.55],
            color=np.array([0.72, 0.50, 0.28]),
            mass=13.0,
        )
    )
    cog = world.scene.add(
        VisualSphere(
            prim_path="/World/CenterOfGravity",
            name="center_of_gravity",
            position=[0, 0, 1.05],
            radius=0.055,
            color=np.array([1.0, 0.9, 0.05]),
        )
    )
    volume_bar = world.scene.add(
        VisualCuboid(
            prim_path="/World/DepthVolumeBar",
            name="depth_volume_bar",
            position=[-1.15, -0.8, 0.4],
            scale=[0.08, 0.08, 0.5],
            color=np.array([0.1, 0.55, 1.0]),
        )
    )

    world.reset()
    world.scene.get_object("cart_proxy").set_visibility(True)

    case_index = 0
    started = time.perf_counter()
    pause_between_cases = 1.5
    print("Isaac Twin replay started")
    print("Cases:", ", ".join(case["title"] for case in SCENARIOS))

    while simulation_app.is_running():
        case = SCENARIOS[case_index]
        frames = case["frames"]
        duration = case["summary"]["duration_s"]
        elapsed = time.perf_counter() - started

        if elapsed > duration + pause_between_cases:
            case_index = (case_index + 1) % len(SCENARIOS)
            started = time.perf_counter()
            continue

        t = min(elapsed, duration)
        frame = frame_at(frames, t)
        active_surface = SURFACE_BY_CASE.get(case["id"])
        for key, prim_path in surface_paths.items():
            set_visible(world.stage, prim_path, key == active_surface)

        # CSV COG is percent across the load platform. Map it onto the proxy cart.
        x = (frame["cog_x_pct"] - 50.0) / 50.0 * 0.55
        y = (frame["cog_y_pct"] - 50.0) / 50.0 * 0.36
        z = 0.88

        roll = frame["roll_deg"]
        pitch = frame["pitch_deg"]
        color = CASE_COLORS[case["id"]]

        set_pose(cart, [0, 0, 0.18], roll * 0.45, pitch * 0.45)

        cargo_drop_offset = 0.0
        if case["id"] == "cargo_drop" and frame["cog_y_pct"] > 80:
            cargo_drop_offset = min((frame["cog_y_pct"] - 80.0) / 12.0, 1.0)

        set_pose(
            cargo,
            [x * 0.55, y * 0.45 - cargo_drop_offset * 0.9, 0.55 - cargo_drop_offset * 0.18],
            roll * 0.65,
            pitch * 0.65,
            0.0,
        )
        cog.set_world_pose(position=[x, y, z])
        volume_height = max(0.15, min(frame["cam_volume_l"] / 120.0, 1.4))
        volume_bar.set_world_pose(position=[-1.15, -0.8, volume_height / 2.0])
        volume_bar.set_local_scale([0.08, 0.08, volume_height])

        if int(t * 10) % 10 == 0:
            print(
                f"{case['title']} t={t:4.1f}s total={frame['total_g']:6.0f}g "
                f"cog=({frame['cog_x_pct']:4.1f},{frame['cog_y_pct']:4.1f}) "
                f"roll={roll:4.1f} pitch={pitch:4.1f}",
                end="\r",
            )

        world.step(render=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
