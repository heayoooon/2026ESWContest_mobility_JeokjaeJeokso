import json
import struct
import zlib
from pathlib import Path

import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
CAPTURES = ROOT / "captures"
OUT = ROOT / "assets" / "surfaces"


def read_twn(path: Path):
    data = path.read_bytes()
    header_len = struct.unpack_from("!I", data, 0)[0]
    header = json.loads(data[4 : 4 + header_len].decode("utf-8"))
    offset = 4 + header_len
    h_len, c_len = header["len"]
    s_depth_len, s_jpeg_len = header["surface"]["lens"]
    height = zlib.decompress(data[offset : offset + h_len])
    offset += h_len
    colors = zlib.decompress(data[offset : offset + c_len])
    offset += c_len
    depth_delta = zlib.decompress(data[offset : offset + s_depth_len])
    offset += s_depth_len
    jpeg = data[offset : offset + s_jpeg_len]
    grid = header["grid"]
    nx, ny = int(grid["nx"]), int(grid["ny"])
    height_mm = np.frombuffer(height, dtype="<i2").reshape(ny, nx).astype(np.float32)
    color = np.frombuffer(colors, dtype=np.uint8).reshape(ny, nx, 3)
    return header, height_mm, color, depth_delta, jpeg


def write_surface_usda(path: Path, header: dict, height_mm: np.ndarray, color: np.ndarray) -> None:
    grid = header["grid"]
    nx, ny = int(grid["nx"]), int(grid["ny"])
    cell = float(grid["cell"])
    x0 = float(grid["x0"])
    y0 = float(grid["y0"])
    points = []
    for y in range(ny):
        for x in range(nx):
            h = height_mm[y, x]
            z = 0.0 if h <= -30000 else float(h) / 1000.0
            points.append((x0 + (x + 0.5) * cell, y0 + (y + 0.5) * cell, z))

    counts = []
    indices = []
    face_colors = []
    for y in range(ny - 1):
        for x in range(nx - 1):
            ids = [y * nx + x, y * nx + x + 1, (y + 1) * nx + x + 1, (y + 1) * nx + x]
            if any(height_mm.flat[i] <= -30000 for i in ids):
                continue
            counts.append(4)
            indices.extend(ids)
            rgb = color[y, x].astype(np.float32) / 255.0
            face_colors.append(tuple(float(v) for v in rgb))

    def vec3_list(values):
        return ",\n        ".join(f"({a:.5f}, {b:.5f}, {c:.5f})" for a, b, c in values)

    path.parent.mkdir(parents=True, exist_ok=True)
    text = f'''#usda 1.0
(
    defaultPrim = "Surface"
    metersPerUnit = 1
)

def Xform "Surface"
{{
    def Mesh "HeightColorMesh"
    {{
        int[] faceVertexCounts = [{", ".join(map(str, counts))}]
        int[] faceVertexIndices = [{", ".join(map(str, indices))}]
        point3f[] points = [
        {vec3_list(points)}
        ]
        color3f[] primvars:displayColor = [
        {vec3_list(face_colors)}
        ] (
            interpolation = "uniform"
        )
        uniform token subdivisionScheme = "none"
    }}
}}
'''
    path.write_text(text, encoding="utf-8")


def main() -> None:
    manifest = []
    for twn in sorted(CAPTURES.glob("*.twn")):
        header, height_mm, color, _, jpeg = read_twn(twn)
        out = OUT / f"{twn.stem}_surface.usda"
        write_surface_usda(out, header, height_mm, color)
        (OUT / f"{twn.stem}_color.jpg").write_bytes(jpeg)
        manifest.append(
            {
                "id": twn.stem,
                "title": header.get("scenario", twn.stem),
                "usd": str(out),
                "jpeg": str(OUT / f"{twn.stem}_color.jpg"),
                "t_run": header.get("t_run"),
                "frame_ts": header.get("frame_ts"),
                "stats": header.get("stats"),
            }
        )
        npy = CAPTURES / f"{twn.stem}_rgb.npy"
        if npy.exists():
            rgb = np.load(npy)
            Image.fromarray(rgb.astype(np.uint8)).save(OUT / f"{twn.stem}_raw_rgb.jpg", quality=90)
    (OUT / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"wrote {len(manifest)} surface USD files to {OUT}")


if __name__ == "__main__":
    main()
