"""저장된 D435i 프레임으로 2.5D vs 3D 처리 비용 측정 (PC / 라즈베리파이 동일 스크립트).

usage: python3 twin/bench.py [frame_dir]      (pi/ 에서, 기본 twin/out)
"""
import json, os, time, platform, pathlib, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # pi/

from twin.twin_core import world_rotation, deproject, HeightMapper, voxel_downsample, TSDF  # noqa: E402

D = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else pathlib.Path(__file__).parent / "out"
m = json.loads((D / "meta.json").read_text(encoding="utf-8"))
filt = np.load(D / "depth_filtered_u16.npy")          # 424x240 (decimation 후)
raw = np.load(D / "depth_raw_u16.npy")                # 848x480
fi, ri, s = m["depth_intrinsics_filtered"], m["depth_intrinsics_raw"], m["depth_scale_m_per_unit"]
R = world_rotation(m["imu"]["accel_mean"])

pw = deproject(filt, fi, s)[0] @ R.T
bounds = [tuple(np.percentile(pw[:, 0], [1, 99])), tuple(np.percentile(pw[:, 1], [1, 99]))]


def bench(fn, n):
    fn()
    t = []
    for _ in range(n):
        t0 = time.perf_counter(); fn(); t.append((time.perf_counter() - t0) * 1000)
    return float(np.median(t))


res = {}
hm_lo = HeightMapper(fi, s, R, bounds, 0.01, m["ref_plane_height_m"])
hm_hi = HeightMapper(ri, s, R, bounds, 0.01, m["ref_plane_height_m"])
res["2.5D heightmap 424x240 -> 1cm grid"] = bench(lambda: hm_lo(filt), 30)
res["2.5D heightmap 848x480 -> 1cm grid"] = bench(lambda: hm_hi(raw), 20)
res["3D deproject 848x480 (points only)"] = bench(lambda: deproject(raw, ri, s), 20)
p_hi = deproject(raw, ri, s)[0]
res["3D voxel downsample 1cm"] = bench(lambda: voxel_downsample(p_hi, 0.01), 5)
tsdf2 = TSDF(fi, s, origin=(-1.2, -0.8, 0.2), size=(2.4, 1.6, 2.4), voxel=0.02)
res[f"3D TSDF integrate 2cm ({tsdf2.tsdf.size/1e6:.1f}M voxels)"] = bench(lambda: tsdf2.integrate(filt), 10)
tsdf1 = TSDF(fi, s, origin=(-1.2, -0.8, 0.2), size=(2.4, 1.6, 2.4), voxel=0.01)
res[f"3D TSDF integrate 1cm ({tsdf1.tsdf.size/1e6:.1f}M voxels)"] = bench(lambda: tsdf1.integrate(filt), 3)

print(f"== {platform.node()} {platform.machine()} numpy {np.__version__}")
for k, v in res.items():
    print(f"  {k:45s} {v:8.1f} ms  (~{1000/v:5.1f} fps)")
(D / f"bench_{platform.machine()}.json").write_text(json.dumps(res, indent=2), encoding="utf-8")
