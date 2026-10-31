#!/usr/bin/env python3
"""SCSS 실시간 트윈 송신기 (센서 파이, ~/d435i/.venv/bin/python).

D435i 실시간 깊이 · 컬러 -> 2.5D 높이맵 + 실측 표면(깊이 · 정렬 컬러)을 만들고,
sensor_feed.py 가 UDP 127.0.0.1:5020 으로 넘기는 최신 센서 · 위험 판정을 붙여
디스플레이 파이(display/twin, TCP 5010)로 약 10 Hz 로 보낸다. 프레임 형식은 display/twin/README.md 참고.

설치 보정(카메라 기울기 R, 바닥 높이 ref_h)은 시작할 때 한 번: 카메라 IMU 1 초 평균 + 첫 깊이 프레임의 가장 흔한 높이.
디스플레이의 '화면 멈춤' 단추는 UDP 5011 'pause' / 'resume' 으로 온다 (멈춘 동안 보내지 않는다).

usage: ~/d435i/.venv/bin/python live_twin.py [dest[:port],...] [--hz 10] [--mirror host[:port],...]
  dest 기본 192.168.10.2 (디스플레이 파이 LAN). Tailscale 이면 100.90.251.42
  --mirror 같은 프레임을 따로 받을 곳 (예: Isaac Sim 트윈 PC, isaac_sim/scripts/live_link.py 포트 5010).
           자기 송신 스레드로 보내서, 그쪽이 꺼져 있어도 디스플레이 송신은 느려지지 않는다.
"""
import argparse, io, json, os, socket, struct, sys, threading, time, zlib
import numpy as np
import pyrealsense2 as rs

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.environ.get('SCSS_PI_DIR', os.path.dirname(HERE))          # 레포의 pi/
sys.path[:0] = [HERE, PI]
from twin_frame import ColorHeightMapper, ReliableSender, send_packet, MAX_RANGE, Image   # noqa: E402
from twin.twin_core import world_rotation, summarize                                     # noqa: E402

W, H, FPS = 848, 480, 30
SESSION = os.urandom(8).hex()
STATE = dict(sensors=None, sensor_t=0.0, paused=False, acc=None)


def sensor_listener(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1', port))
    while True:
        data = s.recvfrom(65536)[0]
        try:
            STATE['sensors'] = json.loads(data)
            STATE['sensor_t'] = time.time()
        except ValueError:
            pass


def control_listener(port=5011):
    c = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    c.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    c.bind(('0.0.0.0', port))
    while True:
        m = c.recvfrom(4096)[0].decode(errors='ignore').strip()
        if m in ('pause', 'resume'):
            STATE['paused'] = m == 'pause'
            print('control', m, flush=True)
        elif m.startswith('rec_start '):                              # rec.sh: 그 폴더에 보낸 프레임을 그대로 저장
            d = m[len('rec_start '):]
            os.makedirs(d, exist_ok=True)
            old, STATE['rec'] = STATE.get('rec'), open(os.path.join(d, 'twin.tws'), 'wb')
            if old:
                old.close()
            print('트윈 기록 시작 ->', d, flush=True)
        elif m == 'rec_stop' and STATE.get('rec'):
            STATE['rec'].close()
            STATE['rec'] = None
            print('트윈 기록 끝', flush=True)


def profile(sensor, stream, fmt):
    return next(p for p in sensor.get_stream_profiles()
                if p.stream_type() == stream and p.format() == fmt and p.fps() == FPS
                and p.as_video_stream_profile().width() == W and p.as_video_stream_profile().height() == H)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dest', nargs='?', default='192.168.10.2')
    ap.add_argument('--hz', type=float, default=10)
    ap.add_argument('--mirror', default='', help='같은 프레임을 따로 받을 곳 (예: Isaac Sim 트윈 PC host:5010)')
    ap.add_argument('--cell', type=float, default=0.01)
    ap.add_argument('--roi', type=float, nargs=4, default=[-.27, .27, .05, .50], metavar=('X0', 'X1', 'Y0', 'Y1'))
    ap.add_argument('--sensor-port', type=int, default=5020)
    ap.add_argument('--plate-z', type=float, default=-0.579, help='카메라 기준 판 윗면 높이 예상값 (m)')
    ap.add_argument('--plate-above-floor', type=float, default=0.208, help='판 윗면이 기준(방바닥)보다 높은 만큼 (m)')
    args = ap.parse_args()
    dests = [(h, int(p or 5010)) for h, _, p in (d.partition(':') for d in args.dest.split(','))]
    mirrors = [(h, int(p or 5010)) for h, _, p in (d.partition(':') for d in args.mirror.split(',') if d)]
    threading.Thread(target=sensor_listener, args=(args.sensor_port,), daemon=True).start()
    threading.Thread(target=control_listener, daemon=True).start()

    dev = rs.context().query_devices()[0]
    sensors = dev.query_sensors()
    depth_s = next(s for s in sensors if s.is_depth_sensor())
    color_s = next(s for s in sensors if s.is_color_sensor())
    motion = next(s for s in sensors if s.is_motion_sensor())
    acc_hist = []

    def on_motion(f):
        if f.get_profile().stream_type() == rs.stream.accel:
            d = f.as_motion_frame().get_motion_data()
            STATE['acc'] = (d.x, d.y, d.z)
            if len(acc_hist) < 400:
                acc_hist.append((d.x, d.y, d.z))

    motion.open([p for p in motion.get_stream_profiles() if p.fps() == 200])
    motion.start(on_motion)
    dp, cp = profile(depth_s, rs.stream.depth, rs.format.z16), profile(color_s, rs.stream.color, rs.format.rgb8)
    sync = rs.syncer(2)
    depth_s.open(dp)
    color_s.open(cp)
    depth_s.start(sync)
    color_s.start(sync)
    scale = depth_s.as_depth_sensor().get_depth_scale()
    ex = dp.get_extrinsics_to(cp)
    ex_R = np.array(ex.rotation).reshape(3, 3).T                       # librealsense 는 열 우선
    cintr = cp.as_video_stream_profile().intrinsics
    filters = [rs.decimation_filter(2), rs.disparity_transform(True), rs.spatial_filter(),
               rs.temporal_filter(), rs.disparity_transform(False), rs.hole_filling_filter(1)]

    def filt(d):
        for f in filters:
            d = f.process(d)
        return d.as_depth_frame()

    def next_pair():
        while True:
            fs = sync.wait_for_frames(3000)
            d, c = fs.get_depth_frame(), fs.get_color_frame()
            if d and c:
                return d, c

    try:
        # ── 설치 보정: 카메라 IMU 1 초 + 첫 깊이 (판 위가 조용할 때 켠다) ──
        time.sleep(1.2)
        for _ in range(15):
            d, c = next_pair()
            df = filt(d)
        acc = np.median(np.array(acc_hist), 0)
        if np.std(acc_hist, 0).max() > 0.5:
            print('! 보정하는 동안 카메라가 흔들렸다 (accel sd %.2f)' % np.std(acc_hist, 0).max(), flush=True)
        R = world_rotation(acc)
        intr = df.profile.as_video_stream_profile().intrinsics
        probe = ColorHeightMapper(intr, cintr, ex_R, ex.translation, scale, R, ((-5, 5), (-5, 5)), args.cell, 0.0)
        z = np.asanyarray(df.get_data()).reshape(-1).astype(np.float32) * scale
        okz = (z > 0) & (z < MAX_RANGE)
        p = probe.rays_w[okz] * z[okz, None]
        # 기준 높이: 녹화 때와 같게 '판 아래 방바닥' 을 0 으로 (상자 인식 규칙이 그 기준으로 맞춰져 있다).
        # 빈 판으로 켜면 가장 흔한 높이 = 판 윗면이므로, 판 윗면을 찾아 PLATE_ABOVE_FLOOR 만큼 내린다.
        # 거치대는 그대로라 판 높이는 늘 비슷하다 (2026-10-04 녹화 -0.579 m, 10-14 실시간 -0.579 m)
        hist, edges = np.histogram(p[:, 2], np.arange(-1.2, 0.0, 0.005))
        near = (edges[:-1] > args.plate_z - 0.05) & (edges[:-1] < args.plate_z + 0.05)
        if hist[near].max(initial=0) > 300:
            plate = float(edges[:-1][near][hist[near].argmax()] + 0.0025)
            sel = np.abs(p[:, 2] - plate) < 0.01
            plate = float(np.median(p[sel, 2]))
            ref_h = plate - args.plate_above_floor
            print('판 윗면 %.3f m -> 기준(방바닥) %.3f m' % (plate, ref_h), flush=True)
        else:
            ref_h = args.plate_z - args.plate_above_floor
            print('! 판 윗면을 못 찾아 기본 기준 %.3f m 를 쓴다 (판 위에 화물이 많거나 카메라가 바뀌었다)' % ref_h, flush=True)
        bounds = ((args.roi[0], args.roi[1]), (args.roi[2], args.roi[3]))
        hm = ColorHeightMapper(intr, cintr, ex_R, ex.translation, scale, R, bounds, args.cell, ref_h)
        print('설치 보정: ref_h %.3f m, 격자 %dx%d @ %.0f cm, ROI %s' % (ref_h, hm.nx, hm.ny, args.cell * 100, bounds), flush=True)
        intr_d = {k: getattr(intr, k) for k in ('width', 'height', 'fx', 'fy', 'ppx', 'ppy')}

        sock = ReliableSender()
        # 미러: 디스플레이와 같은 연결로 보내면 미러가 꺼져 있을 때 연결 시도(3 s x 2)마다 디스플레이 프레임까지 막힌다.
        # 따로 돌면서 가장 최근 프레임만 보내고, 실패하면 2 s 쉬었다 다시 한다.
        mirror_box, mirror_cv = [None], threading.Condition()

        def mirror_sender():
            msock, merr = ReliableSender(), 0
            while True:
                with mirror_cv:
                    while mirror_box[0] is None:
                        mirror_cv.wait()
                    pkt, mirror_box[0] = mirror_box[0], None
                try:
                    send_packet(msock, mirrors, *pkt)
                    if merr:
                        print('미러 다시 연결됨 %s' % mirrors, flush=True)
                    merr = 0
                except OSError as e:
                    merr += 1
                    if merr == 1:
                        print('미러 %s 에 못 보냄 (%s) - 2 s 마다 다시 시도 (디스플레이 송신은 그대로)' % (mirrors, e), flush=True)
                    time.sleep(2.0)

        if mirrors:
            threading.Thread(target=mirror_sender, daemon=True).start()
        seq, t0 = 0, time.time()
        period = 1.0 / args.hz
        next_t = time.time()
        proc, sizes, errors, last_print = [], [], 0, 0.0
        print('-> %s  %.0f Hz%s' % (dests, args.hz, ('  미러 %s' % mirrors) if mirrors else ''), flush=True)
        while True:
            d, c = next_pair()
            now = time.time()
            if now < next_t:
                continue                                                 # 30 fps 중 10 Hz 만 처리 · 송신
            next_t = max(next_t + period, now - period)
            if STATE['paused']:
                continue
            t1 = time.perf_counter()
            df = filt(d)
            depth = np.asanyarray(df.get_data())
            rgb = np.asanyarray(c.get_data())
            h, col = hm(depth, rgb)
            # 실측 표면: 화물 영역(ROI) 안, 바닥보다 5.5 cm 위인 깊이만 + 그 점의 컬러
            dep = depth.reshape(-1).astype(np.float32) * scale
            pts = hm.rays_w * dep[:, None]
            visible = ((dep > 0) & (dep < MAX_RANGE) & (pts[:, 0] >= hm.x0) & (pts[:, 0] < hm.x1) &
                       (pts[:, 1] >= hm.y0) & (pts[:, 1] < hm.y1) & (pts[:, 2] > hm.ref_h + .055))
            pc = hm.rays_c * dep[:, None] + hm.ex_t
            den = np.maximum(pc[:, 2], .001)
            cu = np.rint(pc[:, 0] / den * cintr.fx + cintr.ppx).astype(np.int32)
            cv = np.rint(pc[:, 1] / den * cintr.fy + cintr.ppy).astype(np.int32)
            color_ok = (cu >= 0) & (cu < cintr.width) & (cv >= 0) & (cv < cintr.height)
            aligned = np.full((len(dep), 3), 120, np.uint8)
            use = visible & color_ok
            aligned[use] = rgb[cv[use], cu[use]]
            aligned[~visible] = 0
            aligned = (aligned >> 3) << 3
            sd = np.where(visible, np.rint(dep * 1000), 0).astype('<u2').reshape(depth.shape)
            delta = np.diff(sd.astype(np.int32), axis=1, prepend=0).astype('<i2')
            db = zlib.compress(delta.tobytes(), 3)
            buf = io.BytesIO()
            Image.fromarray(aligned.reshape(*depth.shape, 3)).save(buf, format='JPEG', quality=88, subsampling=0)
            rb = buf.getvalue()
            sens = STATE['sensors']
            age = round(now - STATE['sensor_t'], 3) if sens else None
            if sens and age > 2.0:
                sens = None                                              # sensor_feed 가 멈췄으면 센서 없이
            header = dict(
                v=2, session=SESSION, seq=seq, send_t=time.time(), run='live', scenario=None, src=['live', 'd435i'],
                t_run=round(now - t0, 3), frame_ts=d.get_timestamp() / 1000,
                grid=dict(nx=hm.nx, ny=hm.ny, cell=args.cell, x0=hm.x0, y0=hm.y0, unit='mm', nodata=-32768),
                stats=summarize(h, args.cell, hm.x0, hm.y0),
                calib=dict(ref_h=round(hm.ref_h, 4)),
                cam_accel=np.round(STATE['acc'], 3).tolist() if STATE['acc'] else None,
                sensors=sens, sensor_age=age if sens else None, hz=args.hz,
                surface=dict(intr=intr_d, R=np.asarray(R).tolist(), lens=[len(db), len(rb)], depth_codec='delta-zlib',
                             color_codec='jpeg', roi_kind='inspected_cargo_envelope'))
            hb = zlib.compress(np.nan_to_num(h * 1000, nan=-32768).astype(np.int16).tobytes(), 1)
            cb = zlib.compress(col.tobytes(), 1)
            header['len'] = [len(hb), len(cb)]
            rec = STATE.get('rec')
            if rec:                                                      # 디스플레이에서 다시 재생할 수 있는 형식 (.tws)
                hj = json.dumps(header, ensure_ascii=False).encode()
                blob = struct.pack('!I', len(hj)) + hj + hb + cb + db + rb
                try:
                    rec.write(struct.pack('!dI', time.time(), len(blob)) + blob)
                except (OSError, ValueError):
                    pass
            if mirrors:
                with mirror_cv:
                    mirror_box[0] = (seq, header, hb, cb, db, rb)
                    mirror_cv.notify()
            try:
                size, _ = send_packet(sock, dests, seq, header, hb, cb, db, rb)
                sizes.append(size)
            except OSError as e:
                errors += 1
                if errors == 1 or errors % 50 == 0:
                    print('디스플레이에 못 보냄 (%s) — 다시 시도한다' % e, flush=True)
                time.sleep(0.5)
                continue
            proc.append((time.perf_counter() - t1) * 1000)
            seq += 1
            if now - last_print > 5:
                last_print = now
                print('%6.1fs 보낸 %5d  처리 %4.1f ms  %4.1f KB  센서 %s' % (
                    now - t0, seq, np.median(proc[-50:]), np.mean(sizes[-50:]) / 1024,
                    '없음' if not sens else '%.2fs 전' % age), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        for s in (depth_s, color_s, motion):
            try:
                s.stop()
                s.close()
            except Exception:
                pass


if __name__ == '__main__':
    main()
