"""라즈베리파이에서 돌릴 실시간 트윈 노드 (PC에서도 그대로 동작).

매 프레임: depth 수신 -> 후처리 필터 -> 2.5D 높이맵(mm, int16) -> 다른 센서값과 묶어 UDP 전송
  패킷 = [4B 헤더길이][JSON 헤더][zlib(int16 높이맵)]
  JSON 헤더: seq, t, grid 정보, IMU(카메라 흔들림 감지), stats(높이맵 요약), 외부 센서값(로드셀·MPU-6050 등),
             calib(설치 보정값: 기준면 높이 · 월드 회전 R · intrinsics 등. 매 패킷 같은 값이라 아무 패킷으로도 트윈을 세운다)

usage: python3 twin/pi_twin_node.py [dest[,dest...]] [seconds]      (pi/ 에서)
  dest 는 주소 또는 주소:포트 (포트 기본 5006 = pipeline.py 의 CAM_PORT. 5005 는 ESP32 IMU). 쉼표로 여러 개 주면 같은 패킷을 다 보낸다
  예) 시각화 PC + 센서 파이의 pipeline.py:  192.168.0.5,127.0.0.1

녹화: 컬러 + depth 원본(필터 전)을 RealSense 녹화 파일 한 개로 남긴다. 높이맵 송신과 같이 돈다.
  librealsense 2.58 은 ROS1 .bag 대신 ROS2 rosbag2 형식(.db3, SQLite)으로만 녹화한다.
  BAG_DIR=<폴더>  녹화 파일을 둘 곳. 기본 pi/acquisition/logs (pipeline.py 엑셀과 같은 곳)
  BAG=0           녹화 안 함 (높이맵 송신만)
  파일 = <시각>_d435i.db3. 848x480@30 컬러(rgb8) + depth(z16), 무손실 압축으로 1분에 약 1.5 GB (장면에 따라 다르다).
  realsense-viewer 로 열거나, pyrealsense2 에서 cfg.enable_device_from_file(<.db3>) 로 다시 돌린다.
  pipeline.py 카메라 엑셀의 '카메라 프레임 시각(ms)' 열이 녹화 파일 depth 프레임 타임스탬프와 같다 -> 센서 엑셀과 시간 맞추기.
  카메라 내장 IMU (accel · gyro 200Hz) 도 같이 들어간다.
  Ctrl+C / kill 로 끝내도 녹화 파일을 닫고 끝난다. kill -9 / 전원 차단이면 녹화 파일이 깨질 수 있다.
"""
import json, os, shutil, signal, socket, struct, sys, time, zlib
import numpy as np
import pyrealsense2 as rs

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # pi/

from twin.twin_core import world_rotation, HeightMapper, summarize  # noqa: E402

DESTS = [(h, int(p or 5006)) for h, _, p in
         (d.partition(":") for d in (sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1").split(","))]
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 0      # 0 = 무한
CELL, W, H, FPS = 0.02, 848, 480, 30
RECORD = os.environ.get("BAG", "1") != "0"
BAG_DIR = os.environ.get("BAG_DIR") or os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "acquisition", "logs")
BAG_RATE = 25e6                             # 압축 녹화 바이트/초 (파이 실측 약 25 MB/s, 비압축이면 61 MB/s)


def read_other_sensors():
    """여기에 로드셀(HX711)·MPU-6050 등 값을 넣는다. 지금은 자리표시자."""
    return {"loadcell_g": None, "mpu6050_tilt_deg": None}


def main():
    for name in ("SIGTERM", "SIGHUP"):          # kill / SSH 끊김도 Ctrl+C 처럼 -> 녹화 파일을 닫고 끝낸다
        signal.signal(getattr(signal, name), signal.default_int_handler)
    cleanup, state = [], {}
    try:
        run(cleanup, state)
    except KeyboardInterrupt:
        print("\n중단")
    finally:
        for f in reversed(cleanup):
            try:
                f()
            except Exception as e:
                print("! 정리 중 오류:", e)
        cleanup.clear()
        state.pop("rec", None)                  # 센서를 다 닫고 녹화기를 놓아야 녹화 파일이 닫힌다
        bag = state.get("bag")
        if bag and os.path.exists(bag):
            print(f"녹화: {bag} ({os.path.getsize(bag) / 1e9:.2f} GB)")


def run(cleanup, state):
    dev = rs.context().query_devices()[0]
    motion = next(s for s in dev.query_sensors() if s.is_motion_sensor())
    last_acc = np.zeros(3); acc_hist = []

    def on_motion(f):
        nonlocal last_acc
        if f.get_profile().stream_type() == rs.stream.accel:
            d = f.as_motion_frame().get_motion_data()
            last_acc = np.array([d.x, d.y, d.z]); acc_hist.append(last_acc)

    motion.open([p for p in motion.get_stream_profiles() if p.fps() == 200])
    cleanup.append(motion.close)
    motion.start(on_motion)
    cleanup.append(motion.stop)
    time.sleep(1.0)
    acc0 = np.mean(acc_hist, 0)
    R = world_rotation(acc0)                            # 설치 시 1회: 중력 기준 수평 보정

    # pipeline 의 enable_record_to_file 은 압축을 못 켠다. 비압축(약 61 MB/s)은 파이 SD 카드(약 36 MB/s)가
    # 못 따라가 프레임이 빠진다 -> 압축 녹화기로 센서를 직접 연다 (실측 30fps 빠짐 없음, 약 25 MB/s)
    src, bag = dev, None
    if RECORD:
        os.makedirs(BAG_DIR, exist_ok=True)
        bag = os.path.join(BAG_DIR, time.strftime("%Y%m%d_%H%M%S") + "_d435i.db3")
        src = state["rec"] = rs.recorder(bag, dev, True)   # True = 압축
        state["bag"] = bag
        free = shutil.disk_usage(BAG_DIR).free
        print(f"녹화 -> {bag}  (컬러 + depth {W}x{H}@{FPS}, 압축. 1분에 약 {BAG_RATE * 60 / 1e9:.1f} GB, "
              f"디스크 여유 {free / 1e9:.0f} GB = 약 {free / BAG_RATE / 60:.0f}분)")
    sensors = src.query_sensors()
    depth_s = next(s for s in sensors if s.is_depth_sensor())
    streams = [(depth_s, rs.stream.depth, rs.format.z16)]
    if RECORD:
        streams.append((next(s for s in sensors if s.is_color_sensor()), rs.stream.color, rs.format.rgb8))
    queue = rs.frame_queue(2, keep_frames=True)         # 높이맵이 밀리면 오래된 depth 는 버린다 (녹화는 전부 된다)
    for s, st, fmt in streams:
        s.open(next(p for p in s.get_stream_profiles()
                    if p.stream_type() == st and p.format() == fmt and p.fps() == FPS
                    and p.as_video_stream_profile().width() == W and p.as_video_stream_profile().height() == H))
        cleanup.append(s.close)
        s.start(queue if st == rs.stream.depth else (lambda f: None))  # 컬러는 녹화만
        cleanup.append(s.stop)
    scale = depth_s.as_depth_sensor().get_depth_scale()
    filters = [rs.decimation_filter(2), rs.disparity_transform(True), rs.spatial_filter(),
               rs.temporal_filter(), rs.disparity_transform(False), rs.hole_filling_filter(1)]

    def next_depth():
        d = queue.wait_for_frame(5000)
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
    calib = dict(                                       # 높이맵을 다시 3D 로 세울 때 필요한 설치 기준
        ref_h=round(ref_h, 4), R=np.round(R, 6).tolist(), accel0=np.round(acc0, 4).tolist(),
        intr=intr, depth_scale=scale, max_range=hm.max_range,
        stream=f"depth {W}x{H} z16 @{FPS}fps",
        filters="decimation 2, disparity, spatial, temporal, disparity back, hole_filling 1",
        camera=" ".join(dev.get_info(k) for k in (rs.camera_info.name, rs.camera_info.serial_number))
               + " fw " + dev.get_info(rs.camera_info.firmware_version),
        bag=os.path.basename(bag) if bag else None)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq, t_start, tp = 0, time.time(), []
    print(f"-> udp {DESTS}, grid {hm.nx}x{hm.ny} @ {CELL*100:.0f}cm, bounds {bounds}")
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
            stats=summarize(h, CELL, bounds[0][0], bounds[1][0]),
            sensors=read_other_sensors(),
            calib=calib,
        )).encode()
        pkt = struct.pack("!I", len(header)) + header + body
        for dest in DESTS:
            sock.sendto(pkt, dest)
        tp.append((time.perf_counter() - t0) * 1000); seq += 1
        if seq % 30 == 0:
            print(f"seq {seq:5d}  {1000/np.mean(tp[-30:]):5.1f} fps  proc {np.median(tp[-30:]):5.1f} ms  "
                  f"pkt {len(header)+len(body)} B  max h {np.nanmax(h):.2f} m")


if __name__ == "__main__":
    main()
