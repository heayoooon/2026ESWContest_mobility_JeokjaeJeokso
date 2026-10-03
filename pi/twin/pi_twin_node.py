"""라즈베리파이에서 돌릴 실시간 트윈 노드 (PC에서도 그대로 동작).

매 프레임: depth 수신 -> 후처리 필터 -> 2.5D 높이맵(mm, int16) -> 다른 센서값과 묶어 UDP 전송
  패킷 = [4B 헤더길이][JSON 헤더][zlib(int16 높이맵)]
  JSON 헤더: seq, t, grid 정보, IMU(카메라 흔들림 감지), 외부 센서값(로드셀·MPU-6050 등)

usage: python3 twin/pi_twin_node.py [dest_ip] [seconds]      (pi/ 에서)
"""
import json, os, socket, struct, sys, time, zlib
import numpy as np
import pyrealsense2 as rs

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # pi/

from twin.twin_core import world_rotation, HeightMapper  # noqa: E402

DEST = (sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1", 5005)
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 0      # 0 = 무한
CELL, W, H, FPS = 0.02, 848, 480, 30


def read_other_sensors():
    """여기에 로드셀(HX711)·MPU-6050 등 값을 넣는다. 지금은 자리표시자."""
    return {"loadcell_g": None, "mpu6050_tilt_deg": None}


def main():
    dev = rs.context().query_devices()[0]
    motion = next(s for s in dev.query_sensors() if s.is_motion_sensor())
    last_acc = np.zeros(3); acc_hist = []

    def on_motion(f):
        nonlocal last_acc
        if f.get_profile().stream_type() == rs.stream.accel:
            d = f.as_motion_frame().get_motion_data()
            last_acc = np.array([d.x, d.y, d.z]); acc_hist.append(last_acc)

    motion.open([p for p in motion.get_stream_profiles() if p.fps() == 200])
    motion.start(on_motion)
    time.sleep(1.0)
    R = world_rotation(np.mean(acc_hist, 0))            # 설치 시 1회: 중력 기준 수평 보정

    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    prof = pipe.start(cfg)
    scale = prof.get_device().first_depth_sensor().get_depth_scale()
    filters = [rs.decimation_filter(2), rs.disparity_transform(True), rs.spatial_filter(),
               rs.temporal_filter(), rs.disparity_transform(False), rs.hole_filling_filter(1)]

    def next_depth():
        d = pipe.wait_for_frames().get_depth_frame()
        for f in filters:
            d = f.process(d)
        return d.as_depth_frame()

    for _ in range(15):
        d = next_depth()
    i = d.profile.as_video_stream_profile().intrinsics
    intr = dict(width=i.width, height=i.height, fx=i.fx, fy=i.fy, ppx=i.ppx, ppy=i.ppy)

    # 설치 시 1회: 격자 범위와 기준면(가장 흔한 높이)을 첫 프레임에서 정함
    probe = HeightMapper(intr, scale, R, ((-5, 5), (-5, 5)), CELL)
    z = np.asanyarray(d.get_data()).reshape(-1).astype(np.float32) * scale
    ok = (z > 0) & (z < 3.0); p = probe.rays[ok] * z[ok, None]
    hist, edges = np.histogram(p[:, 2], 400); ref_h = float(edges[hist.argmax()])
    bounds = tuple(tuple(np.percentile(p[:, k], [1, 99]).round(2)) for k in (0, 1))
    hm = HeightMapper(intr, scale, R, bounds, CELL, ref_h)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq, t_start, tp = 0, time.time(), []
    print(f"-> udp {DEST}, grid {hm.nx}x{hm.ny} @ {CELL*100:.0f}cm, bounds {bounds}")
    while not SECONDS or time.time() - t_start < SECONDS:
        t0 = time.perf_counter()
        d = next_depth()
        h = hm(np.asanyarray(d.get_data()))
        body = zlib.compress(np.nan_to_num(h * 1000, nan=-32768).astype(np.int16).tobytes(), 1)
        a = last_acc
        header = json.dumps(dict(
            seq=seq, t=time.time(), frame_ts=d.get_timestamp(),
            grid=dict(nx=hm.nx, ny=hm.ny, cell=CELL, x0=bounds[0][0], y0=bounds[1][0], unit="mm", nodata=-32768),
            imu=dict(accel=a.round(3).tolist(),
                     tilt_change_deg=float(np.degrees(np.arccos(np.clip(a @ R[2] / (np.linalg.norm(a) + 1e-9), -1, 1))))),
            sensors=read_other_sensors(),
        )).encode()
        sock.sendto(struct.pack("!I", len(header)) + header + body, DEST)
        tp.append((time.perf_counter() - t0) * 1000); seq += 1
        if seq % 30 == 0:
            print(f"seq {seq:5d}  {1000/np.mean(tp[-30:]):5.1f} fps  proc {np.median(tp[-30:]):5.1f} ms  "
                  f"pkt {len(header)+len(body)} B  max h {np.nanmax(h):.2f} m")
    pipe.stop(); motion.stop(); motion.close()


if __name__ == "__main__":
    main()
