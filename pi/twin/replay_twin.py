"""녹화 재생 트윈 송신기 (센서 파이).

녹화된 D435i(.db3) + 센서 CSV 를 실제 시간 그대로 다시 돌리면서, 실시간 트윈 노드와 같은 일을 한다:
  depth -> 실측 컬러 표면 + 1cm 높이맵 -> 동기화 센서 -> TCP로 디스플레이 파이에 보낸다.

  v2 프레임 = 헤더 JSON + zlib(높이) + zlib(셀 RGB) + zlib(깊이 mm) + zlib(정렬 RGB)
  기본 TCP는 길이 프리픽스와 영구 연결을 사용한다. --transport udp는 TWN1/TWP1 조각과 XOR 복구를 쓴다.

usage (pi/ 에서):
  ~/d435i/.venv/bin/python twin/replay_twin.py [dest[:port]] [--hz 10] [--runs cruise,brake,turn,drop] [--once]
  dest 기본 100.90.251.42 (display, Tailscale), 포트 기본 5010
"""
import argparse, csv, datetime as dt, io, json, os, socket, struct, sys, threading, time, zlib
import numpy as np
import pyrealsense2 as rs
from transport_core import fragments, ReliableSender
try:
    from PIL import Image
except ImportError:
    # Debian's existing Pillow is ABI-compatible with this Python 3.13 venv.
    # No package installation or system configuration change is needed.
    sys.path.append('/usr/lib/python3/dist-packages')
    from PIL import Image

PI_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PI_DIR)
from twin.twin_core import world_rotation, summarize  # noqa: E402

LOG = os.path.join(PI_DIR, "acquisition", "logs")
RUNS = {   # 키: (시나리오, 센서 CSV, d435i.db3)
    "cruise": ("정속주행", "20261003_165743", "20261003_165741"),
    "brake": ("급출발 · 급정거", "20261003_170015", "20261003_170013"),
    "turn": ("우회전", "20261003_170232", "20261003_170231"),
    "drop": ("화물 떨어짐", "20261003_170459", "20261003_170458"),
}
CAM_WALL_OFS = 0.023    # 카메라 엑셀: time(벽시계) - frame_ts 중앙값 (4런 모두 0.022~0.023 s)
CHUNK = 1200
MAX_RANGE = 2.5
SESSION = os.urandom(8).hex()
CTRL = dict(paused=False, restart=False)   # 디스플레이 앱 버튼 (UDP 5011: pause / resume / restart)


class Restart(Exception):
    pass


def control_listener(port=5011):
    c = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    c.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    c.bind(("0.0.0.0", port))
    while True:
        m = c.recvfrom(64)[0].decode(errors="ignore").strip()
        if m == "pause":
            CTRL["paused"] = True
        elif m == "resume":
            CTRL["paused"] = False
        elif m == "restart":
            CTRL.update(restart=True, paused=False)
        print("control", m, flush=True)


def find_latest(min_age=3.0):
    """logs 의 가장 새 <시각>_d435i.db3 와 시각이 가장 가까운 센서 CSV(15 s 이내) 한 쌍. 아직 쓰는 중인 파일은 건너뛴다."""
    now = time.time()
    dbs = sorted(f for f in os.listdir(LOG) if f.endswith("_d435i.db3") and now - os.path.getmtime(os.path.join(LOG, f)) > min_age)
    for f in reversed(dbs):
        db = f[:-len("_d435i.db3")]
        try:
            t_db = dt.datetime.strptime(db, "%Y%m%d_%H%M%S")
        except ValueError:
            continue
        best = None
        for c in os.listdir(LOG):
            if not c.endswith(".csv") or c.endswith("_camera.csv"):
                continue
            try:
                d = abs((dt.datetime.strptime(c[:-4], "%Y%m%d_%H%M%S") - t_db).total_seconds())
            except ValueError:
                continue
            if d <= 15 and (best is None or d < best[0]):
                best = (d, c[:-4])
        if best:
            return ("최신 녹화 " + db[-6:], best[1], db)
    return None


CAL = {}               # 설치 보정 (R, ref_h, bounds). 카메라 마운트는 4런 모두 같다 -> 정지 상태 런에서 한 번만 잡는다
SENSOR_COLS = ("SW_g", "SE_g", "NW_g", "NE_g", "total_g", "cog_x_pct", "cog_y_pct", "c1_N",
               "imu1_ax_G", "imu1_ay_G", "imu1_az_G", "imu1_gx_dps", "imu1_gy_dps", "imu1_gz_dps",
               "imu1_roll_deg", "imu1_pitch_deg",
               # 2026-10-04 부터: 뒤 벨트 장력 · mm 무게중심 · 위험 판정 (analysis/risk.py 등)
               "c2_N", "load_kg", "cog_x_mm", "cog_y_mm", "cog_pred_x_mm", "cog_pred_y_mm", "h_x_mm", "h_y_mm",
               "c1_min_N", "c2_min_N", "risk", "lv_load", "lv_cog", "lv_move", "lv_c1", "lv_c2")


def load_sensor_csv(path):
    rows = []
    with open(path, encoding="utf-8-sig") as f:
        for r in csv.DictReader(f):
            t = dt.datetime.strptime(r["time"], "%Y-%m-%d %H:%M:%S.%f").timestamp()
            v = {}
            for k in SENSOR_COLS:
                try:
                    v[k] = float(r.get(k))
                except (TypeError, ValueError):
                    v[k] = None
            v["flags"] = r.get("flags", "")
            v["risk_text"] = r.get("risk_text", "") or ""
            # 10-04 부터 무게중심이 mm 로만 적힌다 -> % 도 채운다 (BED_L 250 / BED_W 392, acquisition/env.sh)
            if v.get("cog_x_pct") is None and v.get("cog_x_mm") is not None:
                v["cog_x_pct"] = round(v["cog_x_mm"] / 250.0 * 100, 1)
            if v.get("cog_y_pct") is None and v.get("cog_y_mm") is not None:
                v["cog_y_pct"] = round(v["cog_y_mm"] / 392.0 * 100, 1)
            v["t_s"] = float(r["t_s"])
            rows.append((t, v))
    return np.array([t for t, _ in rows]), [v for _, v in rows]


class ColorHeightMapper:
    """twin_core.HeightMapper 와 같은 2.5D 높이맵 + 칸별 평균 컬러 (depth 점을 컬러 카메라로 투영)."""

    def __init__(self, intr, cintr, ex_R, ex_t, scale, R, bounds, cell, ref_h, max_range=MAX_RANGE):
        h, w = intr.height, intr.width
        u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
        rays = np.stack([(u - intr.ppx) / intr.fx, (v - intr.ppy) / intr.fy, np.ones_like(u)], -1).reshape(-1, 3)
        self.rays_w = (rays @ np.asarray(R, np.float32).T).astype(np.float32)
        self.rays_c = (rays @ np.asarray(ex_R, np.float32).T).astype(np.float32)
        self.ex_t = np.asarray(ex_t, np.float32)
        self.cintr = cintr
        self.scale, self.cell, self.ref_h, self.max_range = scale, cell, ref_h, max_range
        (self.x0, self.x1), (self.y0, self.y1) = bounds
        self.nx = int(np.ceil((self.x1 - self.x0) / cell))
        self.ny = int(np.ceil((self.y1 - self.y0) / cell))

    def __call__(self, depth_u16, color):
        z = depth_u16.reshape(-1).astype(np.float32) * self.scale
        ok = (z > 0) & (z < self.max_range)
        zz = z[ok, None]
        p = self.rays_w[ok] * zz
        ix = np.floor((p[:, 0] - self.x0) / self.cell).astype(np.int32)
        iy = np.floor((p[:, 1] - self.y0) / self.cell).astype(np.int32)
        inb = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        flat = iy[inb] * self.nx + ix[inb]
        n = self.nx * self.ny
        hm = np.full(n, -np.inf, np.float32)
        np.maximum.at(hm, flat, p[inb, 2] - self.ref_h)
        hm[np.isinf(hm)] = np.nan
        # 컬러: 같은 점을 컬러 카메라 좌표로 옮겨 투영 (왜곡 무시)
        pc = self.rays_c[ok][inb] * zz[inb] + self.ex_t
        ci = self.cintr
        cu = (pc[:, 0] / pc[:, 2] * ci.fx + ci.ppx).astype(np.int32)
        cv = (pc[:, 1] / pc[:, 2] * ci.fy + ci.ppy).astype(np.int32)
        inc = (cu >= 0) & (cu < ci.width) & (cv >= 0) & (cv < ci.height)
        inc &= p[inb, 2] - self.ref_h >= hm[flat] - 0.015
        rgb = color[cv[inc], cu[inc]].astype(np.float32)
        cnt = np.bincount(flat[inc], minlength=n).astype(np.float32)
        col = np.zeros((n, 3), np.float32)
        for k in range(3):
            col[:, k] = np.bincount(flat[inc], rgb[:, k], minlength=n)
        col = np.where(cnt[:, None] > 0, col / np.maximum(cnt, 1)[:, None], 90)
        return hm.reshape(self.ny, self.nx), col.astype(np.uint8).reshape(self.ny, self.nx, 3)


def send_packet(sock, dests, seq, header, *bodies):
    hb = json.dumps(header, ensure_ascii=False).encode()
    data = struct.pack("!I", len(hb)) + hb + b"".join(bodies)
    if isinstance(sock,ReliableSender):
        return sock.send_frame(dests,data)
    packets = fragments(data,seq)
    for dest in dests:
        for i,packet in enumerate(packets):
            sock.sendto(packet,dest)
            if i % 4 == 3:
                time.sleep(.0008)  # Pace Tailscale bursts; ~12ms per 56KB frame.
    return sum(map(len,packets)),len(packets)


def play_run(key, args, sock, dests, seq0, calib_only=False):
    scenario, s_name, b_name = RUNS[key]
    st, sv = load_sensor_csv(os.path.join(LOG, s_name + ".csv"))
    t_lo, t_hi = st[0] - 0.3, st[-1] + 0.3            # 센서가 있는 구간만 보낸다
    cfg = rs.config()
    db3 = os.path.join(LOG, b_name + "_d435i.db3")
    if not os.path.exists(db3):                                   # 2026-10-04 녹화는 홈 폴더에 있다
        db3 = os.path.join(os.path.expanduser("~"), b_name + "_d435i.db3")
    cfg.enable_device_from_file(db3, repeat_playback=False)
    pipe = rs.pipeline()
    prof = pipe.start(cfg)
    prof.get_device().as_playback().set_real_time(False)   # 우리가 직접 시간을 맞춘다 (프레임 안 버림)
    scale = prof.get_device().first_depth_sensor().get_depth_scale()
    cprof = prof.get_stream(rs.stream.color).as_video_stream_profile()
    ex = prof.get_stream(rs.stream.depth).get_extrinsics_to(cprof)
    ex_R = np.array(ex.rotation).reshape(3, 3).T                    # librealsense 는 열 우선
    filters = [rs.decimation_filter(2), rs.disparity_transform(True), rs.spatial_filter(),
               rs.temporal_filter(), rs.disparity_transform(False), rs.hole_filling_filter(1)]

    def filt(d):
        for f in filters:
            d = f.process(d)
        return d.as_depth_frame()

    acc, hm, seq = [], None, seq0
    wall0 = ts0 = None
    next_ts = t_lo
    proc, sizes, sent, behind = [], [], 0, 0
    print(f"[{scenario}] 센서 {s_name} ({len(st)}행 {st[-1]-st[0]:.1f}s), 영상 {b_name}")
    try:
        while True:
            ok, fs = pipe.try_wait_for_frames(3000)
            if not ok:
                break
            for f in fs:
                if f.profile.stream_type() == rs.stream.accel:
                    m = f.as_motion_frame().get_motion_data()
                    acc.append((m.x, m.y, m.z))
            d, c = fs.get_depth_frame(), fs.get_color_frame()
            if not d or not c:
                continue
            ts = d.get_timestamp() / 1000 + CAM_WALL_OFS              # 벽시계 기준 (센서 CSV 와 같은 축)
            if hm is None:                                            # 설치 보정: 처음 20프레임 accel + depth
                df = filt(d)
                if len(acc) < 20:
                    continue
                intr = df.profile.as_video_stream_profile().intrinsics
                if CAL:                                               # 이미 잡은 설치 보정을 쓴다
                    hm = ColorHeightMapper(intr, cprof.intrinsics, ex_R, ex.translation, scale, CAL["R"],
                                           CAL["bounds"], args.cell, CAL["ref_h"])
                    continue
                if np.std(acc[:20], 0).max() > 0.5:
                    print("  ! 보정 구간에 움직임이 있다 (accel sd %.2f)" % np.std(acc[:20], 0).max())
                R = world_rotation(np.median(acc, 0))
                probe = ColorHeightMapper(intr, cprof.intrinsics, ex_R, ex.translation, scale, R,
                                          ((-5, 5), (-5, 5)), args.cell, 0.0)
                z = np.asanyarray(df.get_data()).reshape(-1).astype(np.float32) * scale
                okz = (z > 0) & (z < MAX_RANGE)
                p = probe.rays_w[okz] * z[okz, None]
                hist, edges = np.histogram(p[:, 2], 400)
                ref_h = float(edges[hist.argmax()])
                # Fixed camera-frame cargo envelope, inspected against original RGB/depth.
                # This is NOT a measured tray/support polygon; metadata makes that explicit.
                bounds = ((args.roi[0], args.roi[1]), (args.roi[2], args.roi[3]))
                hm = ColorHeightMapper(intr, cprof.intrinsics, ex_R, ex.translation, scale, R, bounds, args.cell, ref_h)
                print(f"  설치 보정 ({key}): grid {hm.nx}x{hm.ny} @ {args.cell*100:.0f}cm  bounds {bounds}  ref_h {ref_h:.3f}")
                CAL.update(R=R, ref_h=ref_h, bounds=bounds)
                if calib_only:
                    break
                continue
            if ts < next_ts:
                if ts > t_lo - 0.5:
                    filt(d)                                           # temporal 필터 상태만 이어간다
                continue
            if ts > t_hi:
                break
            if wall0 is None:
                if isinstance(sock,ReliableSender): sock.prepare(dests)
                wall0, ts0 = time.time(), ts
            if CTRL["restart"]:                                       # 디스플레이 '처음부터'
                CTRL["restart"] = False
                raise Restart()
            while CTRL["paused"]:                                     # 디스플레이 '일시정지': 재생 시계도 멈춘다
                t_p = time.time(); time.sleep(.05); wall0 += time.time() - t_p
            wait = wall0 + (ts - ts0) - time.time()                  # 녹화 시간 그대로 재생
            if wait > 0:
                time.sleep(wait)
            elif wait < -0.15:
                behind += 1
            t0 = time.perf_counter()
            df = filt(d)
            depth = np.asanyarray(df.get_data())
            rgb = np.asanyarray(c.get_data())
            h, col = hm(depth, rgb)
            # Preserve camera-organized geometry, including the observed vertical faces.
            dep = depth.reshape(-1).astype(np.float32) * scale
            pts = hm.rays_w * dep[:, None]
            visible = ((dep > 0) & (dep < MAX_RANGE) &
                       (pts[:,0] >= hm.x0) & (pts[:,0] < hm.x1) &
                       (pts[:,1] >= hm.y0) & (pts[:,1] < hm.y1) &
                       (pts[:,2] > hm.ref_h + .055))
            pc = hm.rays_c * dep[:,None] + hm.ex_t
            ci = hm.cintr
            den = np.maximum(pc[:,2], .001)
            cu = np.rint(pc[:,0]/den*ci.fx+ci.ppx).astype(np.int32)
            cv = np.rint(pc[:,1]/den*ci.fy+ci.ppy).astype(np.int32)
            color_ok = (cu>=0)&(cu<ci.width)&(cv>=0)&(cv<ci.height)
            aligned = np.full((len(dep),3), 120, np.uint8)
            use = visible & color_ok
            aligned[use] = rgb[cv[use],cu[use]]
            aligned[~visible] = 0
            # Five-bit color reduces transport noise/bandwidth without averaging box faces.
            aligned = (aligned >> 3) << 3
            sd = np.where(visible, np.rint(dep*1000), 0).astype('<u2').reshape(depth.shape)
            delta=np.diff(sd.astype(np.int32),axis=1,prepend=0).astype('<i2')
            db = zlib.compress(delta.tobytes(), 3)
            image_bytes=io.BytesIO()
            Image.fromarray(aligned.reshape(*depth.shape,3)).save(image_bytes,format='JPEG',quality=88,subsampling=0)
            rb = image_bytes.getvalue()
            k = int(np.searchsorted(st, ts, "right")) - 1
            sens = sv[k] if 0 <= k < len(sv) else None
            header = dict(
                v=2, session=SESSION, seq=seq, send_t=time.time(), run=key, scenario=scenario, src=[s_name, b_name],
                t_run=round(ts - st[0], 3), dur_run=round(st[-1] - st[0], 3), frame_ts=ts - CAM_WALL_OFS,
                grid=dict(nx=hm.nx, ny=hm.ny, cell=args.cell, x0=hm.x0, y0=hm.y0, unit="mm", nodata=-32768),
                stats=summarize(h, args.cell, hm.x0, hm.y0),
                calib=dict(ref_h=round(hm.ref_h, 4)),             # 카메라 = 월드 원점, 바닥 = ref_h
                cam_accel=np.round(acc[-1], 3).tolist() if acc else None,
                sensors=sens, sensor_age=round(ts - st[k], 3) if sens else None,
                hz=args.hz,
                capture=bool(args.capture_dir and sent == 20),
                surface=dict(intr={name: getattr(intr, name) for name in ('width','height','fx','fy','ppx','ppy')},
                             R=CAL['R'].tolist(), lens=[len(db),len(rb)], depth_codec='delta-zlib',color_codec='jpeg',
                             roi_kind='inspected_cargo_envelope'),
            )
            hb = zlib.compress(np.nan_to_num(h * 1000, nan=-32768).astype(np.int16).tobytes(), 1)
            cb = zlib.compress(col.tobytes(), 1)
            header["len"] = [len(hb), len(cb)]
            size, nchunk = send_packet(sock, dests, seq, header, hb, cb, db, rb)
            if args.capture_dir and sent == 20:
                os.makedirs(args.capture_dir, exist_ok=True)
                np.save(os.path.join(args.capture_dir, key + '_rgb.npy'), rgb)
                meta = json.dumps(header, ensure_ascii=False).encode()
                with open(os.path.join(args.capture_dir, key + '.twn'), 'wb') as out:
                    out.write(struct.pack('!I',len(meta))+meta+hb+cb+db+rb)
            proc.append((time.perf_counter() - t0) * 1000); sizes.append(size)
            seq += 1; sent += 1
            next_ts += 1.0 / args.hz
            while next_ts <= ts:                                      # 밀렸으면 다음 칸으로
                next_ts += 1.0 / args.hz
            if sent % (args.hz * 2) == 0:
                print(f"  t {ts - st[0]:5.1f}s  보낸 {sent:4d}  처리 {np.median(proc[-20:]):5.1f} ms "
                      f"(최대 {max(proc[-20:]):5.1f})  패킷 {np.mean(sizes[-20:])/1024:5.1f} KB  밀림 {behind}")
    finally:
        pipe.stop()
    if proc:
        print(f"  끝: {sent}개 송신, 처리 중앙 {np.median(proc):.1f} ms / p95 {np.percentile(proc, 95):.1f} ms, "
              f"패킷 평균 {np.mean(sizes)/1024:.1f} KB = {np.mean(sizes)*args.hz*8/1e6:.2f} Mbps, 밀림 {behind}")
    return seq


def main():
    global SESSION
    ap = argparse.ArgumentParser()
    ap.add_argument("dest", nargs="?", default="100.90.251.42")
    ap.add_argument("--hz", type=int, default=10)
    ap.add_argument("--cell", type=float, default=0.01)
    ap.add_argument("--roi", type=float, nargs=4, default=[-.24,.16,.035,.45], metavar=('X0','X1','Y0','Y1'))
    ap.add_argument("--capture-dir", default='')
    ap.add_argument('--transport',choices=('tcp','udp'),default='tcp')
    ap.add_argument("--runs", default="cruise,brake,turn,drop")
    ap.add_argument("--calib-run", default="cruise", help="설치 보정을 잡을 정지 상태 런")
    ap.add_argument("--once", action="store_true", help="한 바퀴만 (기본은 반복)")
    ap.add_argument("--pair", action="append", default=[], metavar="KEY:DB3:CSV[:이름]",
                    help="재생할 녹화 쌍 (logs 안 파일 이름, 확장자 없이). 여러 번 줄 수 있다. 주면 --runs 대신 이것만")
    ap.add_argument("--latest", action="store_true",
                    help="logs 의 가장 새 db3 + 가까운 센서 CSV 를 자동으로 재생. 반복할 때마다 다시 찾아 새 녹화를 바로 반영")
    args = ap.parse_args()
    dests = [(h, int(p or 5010)) for h, _, p in (d.partition(":") for d in args.dest.split(","))]
    sock = ReliableSender() if args.transport == 'tcp' else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if args.transport == 'udp': sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
    print(f"-> {args.transport} {dests}  {args.hz} Hz")
    for spec in args.pair:                                            # --pair KEY:DB3:CSV[:이름]
        parts = spec.split(":")
        key, db3, csvn = parts[0], parts[1].replace("_d435i.db3", "").replace("_d435i", ""), parts[2].replace(".csv", "")
        RUNS[key] = (parts[3] if len(parts) > 3 else key, csvn, db3)
    if args.pair:
        args.runs = ",".join(sp.split(":")[0] for sp in args.pair)
        if args.calib_run not in RUNS or args.calib_run not in args.runs.split(","):
            args.calib_run = args.runs.split(",")[0]
    threading.Thread(target=control_listener, daemon=True).start()
    seq, current = 0, None
    try:
        while True:
            if args.latest:                                           # 새 녹화가 생겼으면 그걸로 바꾼다
                found = find_latest()
                if found is None:
                    print("logs 에 재생할 db3 + CSV 쌍이 없다. 5 s 뒤 다시 찾는다", flush=True)
                    time.sleep(5); continue
                if found != current:
                    current = found
                    RUNS["latest"] = found
                    args.runs = args.calib_run = "latest"
                    CAL.clear()
                    print("최신 녹화로 전환:", found, flush=True)
            if not CAL:
                play_run(args.calib_run, args, sock, dests, 0, calib_only=True)
            for key in args.runs.split(","):
                try:
                    seq = play_run(key, args, sock, dests, seq)
                except Restart:
                    print("처음부터 다시", flush=True)
                    SESSION = os.urandom(8).hex()
                    break
                except OSError as exc:
                    print('연결 오류, 다음 시나리오에서 재접속:',exc,flush=True)
                    SESSION=os.urandom(8).hex()
                    time.sleep(1)
            if args.once:
                break
    except KeyboardInterrupt:
        print("\n중단")


if __name__ == "__main__":
    main()
