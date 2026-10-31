import sys
from pathlib import Path

import bpy


def convert(source: Path, target: Path) -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()
    bpy.ops.import_scene.gltf(filepath=str(source))
    target.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.usd_export(
        filepath=str(target),
        export_textures=True,
        export_materials=True,
        selected_objects_only=False,
    )


def main() -> None:
    args = sys.argv[sys.argv.index("--") + 1 :]
    if len(args) % 2:
        raise SystemExit("usage: blender -b --python convert_glb_to_usd.py -- in.glb out.usd [...]")
    for i in range(0, len(args), 2):
        convert(Path(args[i]), Path(args[i + 1]))


if __name__ == "__main__":
    main()
