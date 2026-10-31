"""Convert the 2026-10-14/15 sensor-Pi recordings (data/live_runs/*) for the twin.

Each run folder holds sensors.csv (10 Hz sensor + judgement rows), twin.tws (D435i 2.5-D height map,
cell colours, depth, JPEG colour frame with the latest sensor row embedded) and summary.json.

Outputs
  live_data.json                 cases (all CSV channels kept as recorded, judgement, derived motion)
  data/live_cache/<id>.npz       per-frame height map (int16 mm), cell colours, stats, JPEG bytes

Derived motion (documented assumptions, see notes in the JSON):
  a_fwd    forward acceleration m/s^2 = (imu1_ax_G - bias) * g, bias = median of the first 0.5 s,
           then linearly detrended so the integrated speed returns to its start value (or to 0 when the
           run ends at rest). Hand-cart IMU drift otherwise reaches several m/s over one run.
  yaw_rate deg/s = imu1_gz_dps - bias (bias = median of the first 0.5 s); gz+ is a left turn.
  Recorded roll/pitch are accelerometer tilt estimates and follow ax/ay almost exactly (corr -0.8..-0.9),
  so the twin tilts the body from the accelerations (suspension model) and shows the raw angles as data.
Camera grid -> bed: grid x is parallel to the bed X axis (NW->NE) with opposite sign, grid y is parallel to
bed Y (N->S); the camera sits at the N (front) end looking toward S. Verified on 2026-10-18 from the load-cell
COG / camera centroid correlation and the box layout in the height maps.
"""
import csv
import json
import math
import struct
import zlib
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RUNS = ROOT / 'data' / 'live_runs'
CACHE = ROOT / 'data' / 'live_cache'
G = 9.80665
MOVING_WORDS = ('주행', '커브', '진동', '충돌', '급출발', '급정거', '이동', '쓰러짐')
# English titles for the UI (Kit's default font has no Hangul glyphs); keyword order matters.
ENGLISH = (('정속주행', 'Constant drive'), ('급출발급정거', 'Hard start / stop'), ('무게중심쏠림', 'COG offset'),
           ('장력고정', 'belts fixed'), ('장력정상', 'belts normal'), ('벨트풀고', 'belt loosened'),
           ('뒷벨트없음화물이동', 'No rear belt, cargo shift'), ('뒷벨트없음화물쓰러짐', 'No rear belt, cargo tip-over'),
           ('좌회전급커브', 'Sharp left turn'), ('우회전급커브', 'Sharp right turn'), ('진동', 'Vibration'),
           ('충돌', 'Collision'))


def english_title(title):
    """Translate the keyword chunks of a run title in order of appearance; a trailing digit is kept."""
    found = []
    rest = title
    for ko, en in ENGLISH:
        pos = rest.find(ko)
        if pos >= 0:
            found.append((pos, en))
            rest = rest.replace(ko, ' ' * len(ko), 1)
    digits = ''.join(ch for ch in rest if ch.isdigit())
    out = ', '.join(en for _, en in sorted(found)) or title
    return f'{out} #{digits}' if digits else out
START_REST_WORDS = ('급출발',)
END_REST_WORDS = ('급정거',)


def number(value):
    try:
        x = float(value)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def read_tws(path):
    data = path.read_bytes()
    off = 0
    frames = []
    while off + 12 <= len(data):
        wall, length = struct.unpack_from('!dI', data, off)
        off += 12
        blob = data[off:off + length]
        off += length
        hl = struct.unpack_from('!I', blob, 0)[0]
        header = json.loads(blob[4:4+ hl])
        o = 4 + hl
        hb, cb = header['len']
        db, rb = header['surface']['lens']
        grid = header['grid']
        height = np.frombuffer(zlib.decompress(blob[o:o + hb]), '<i2').reshape(grid['ny'], grid['nx'])
        o += hb
        colour = np.frombuffer(zlib.decompress(blob[o:o + cb]), np.uint8).reshape(grid['ny'], grid['nx'], 3)
        o += cb
        intr = header['surface']['intr']
        # Depth is stored as int16 row deltas (prepend 0) of the visible-only depth in mm.
        delta = np.frombuffer(zlib.decompress(blob[o:o + db]), '<i2').reshape(intr['height'], intr['width'])
        depth = np.cumsum(delta.astype(np.int32), axis=1).astype(np.uint16)
        o += db
        jpeg = blob[o:o + rb]
        frames.append((wall, header, height, colour, jpeg, depth))
    return frames


def detrend_velocity(t, a_fwd):
    """Linear drift removal: the integrated speed change over the run is forced to zero.

    The IMU only measures speed *changes*; the twin adds them to a base speed (0 for runs that start
    at rest, otherwise the cruise-speed slider) and clamps the result at zero."""
    v = np.concatenate([[0.0], np.cumsum(0.5 * (a_fwd[1:] + a_fwd[:-1]) * np.diff(t))])
    drift = v[-1] / max(t[-1] - t[0], 1e-6)
    a_fwd = a_fwd - drift
    v = np.concatenate([[0.0], np.cumsum(0.5 * (a_fwd[1:] + a_fwd[:-1]) * np.diff(t))])
    return a_fwd, v


def prepare_run(folder):
    title = folder.name.split('_', 2)[2] if folder.name.count('_') >= 2 else folder.name
    run_id = '_'.join(folder.name.split('_', 2)[:2])
    rows = list(csv.DictReader((folder / 'sensors.csv').open(encoding='utf-8-sig', newline='')))
    rows = [r for r in rows if number(r.get('t_s')) is not None]
    rows.sort(key=lambda r: float(r['t_s']))
    t0 = float(rows[0]['t_s'])
    wall0 = number(rows[0].get('wall'))
    frames = []
    for r in rows:
        f = {k: number(v) for k, v in r.items() if k not in ('flags', 'risk_text', 'algo', 'risk_overall')}
        f['t'] = float(r['t_s']) - t0
        f['flags'] = r.get('flags', '')
        f['risk_text'] = r.get('risk_text', '')
        f['risk_overall'] = r.get('risk_overall', '')
        frames.append(f)
    t = np.array([f['t'] for f in frames])
    ax = np.array([f.get('imu1_ax_G') if f.get('imu1_ax_G') is not None else np.nan for f in frames])
    gz = np.array([f.get('imu1_gz_dps') if f.get('imu1_gz_dps') is not None else np.nan for f in frames])
    ax = np.where(np.isnan(ax), np.nanmedian(ax) if np.isfinite(np.nanmedian(ax)) else 0.0, ax)
    gz = np.where(np.isnan(gz), np.nanmedian(gz) if np.isfinite(np.nanmedian(gz)) else 0.0, gz)
    head = t <= t[0] + 0.5
    ax_bias = float(np.median(ax[head]))
    gz_bias = float(np.median(gz[head]))
    moving = any(w in title for w in MOVING_WORDS)
    starts_at_rest = any(w in title for w in START_REST_WORDS) or not moving
    ends_at_rest = any(w in title for w in END_REST_WORDS) or not moving
    if moving:
        a_fwd, v_imu = detrend_velocity(t, (ax - ax_bias) * G)
        yaw_rate = gz - gz_bias
    else:  # parked (cargo pushed by hand): the vehicle does not move
        a_fwd = np.zeros_like(ax)
        v_imu = np.zeros_like(ax)
        yaw_rate = np.zeros_like(gz)
    yaw = np.concatenate([[0.0], np.cumsum(0.5 * (yaw_rate[1:] + yaw_rate[:-1]) * np.diff(t))])
    for f, a, v, w, y in zip(frames, a_fwd, v_imu, yaw_rate, yaw):
        f['a_fwd'] = round(float(a), 4)
        f['v_imu'] = round(float(v), 4)
        f['yaw_rate'] = round(float(w), 3)
        f['yaw_imu'] = round(float(y), 2)

    # Camera frames: align by wall clock (the .tws records wall time per frame, the CSV has 'wall').
    tws = read_tws(folder / 'twin.tws')
    cam_t = np.array([wall - wall0 for wall, *_ in tws]) if wall0 else np.arange(len(tws)) * 0.1
    heights = np.stack([fr[2] for fr in tws]).astype(np.int16)
    colours = np.stack([fr[3] for fr in tws]).astype(np.uint8)
    depths = np.stack([fr[5] for fr in tws]).astype(np.uint16)
    jpegs = [fr[4] for fr in tws]
    offsets = np.cumsum([0] + [len(j) for j in jpegs])
    stats = [fr[1]['stats'] for fr in tws]
    grid = tws[0][1]['grid']
    calib = tws[0][1].get('calib', {})
    surface = tws[0][1]['surface']
    camera = {'intr': surface['intr'], 'R': surface['R'], 'ref_h': calib.get('ref_h'),
              'cam_accel': tws[0][1].get('cam_accel')}
    CACHE.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(CACHE / f'{run_id}.npz', t=cam_t, height=heights, colour=colours, depth=depths,
                        jpeg=np.frombuffer(b''.join(jpegs), np.uint8), jpeg_offsets=offsets,
                        stats=json.dumps(stats), grid=json.dumps(grid), camera=json.dumps(camera))
    summary = json.loads((folder / 'summary.json').read_text(encoding='utf-8'))
    levels = [f.get('risk') for f in frames if f.get('risk') is not None]
    ay = np.array([f.get('imu1_ay_G') if f.get('imu1_ay_G') is not None else np.nan for f in frames])
    ay_bias = np.nanmedian(ay[head]) if np.isfinite(np.nanmedian(ay[head])) else 0.0   # first 0.5 s, like ax
    return {
        'id': run_id, 'title': title, 'title_en': english_title(title), 'dir': str(folder),
        'duration': float(t[-1]), 'rows': len(frames),
        'ay_bias_G': round(float(ay_bias), 4),
        'recorded': summary.get('started'), 'frames': frames,
        'motion': {'moving': moving, 'starts_at_rest': starts_at_rest, 'ends_at_rest': ends_at_rest,
                   'ax_bias_G': round(ax_bias, 4), 'gz_bias_dps': round(gz_bias, 3),
                   'v_imu_peak': round(float(v_imu.max()), 3), 'yaw_total_deg': round(float(yaw[-1]), 1)},
        'camera': {'frames': len(tws), 'duration': float(cam_t[-1] - cam_t[0]) if len(tws) > 1 else 0.0,
                   'grid': grid, 'ref_h': calib.get('ref_h'), 'cache': str(CACHE / f'{run_id}.npz')},
        'risk_max': max(levels) if levels else None,
        'load_kg_mean': round(float(np.mean([f['load_kg'] for f in frames if f.get('load_kg') is not None])), 2)
        if any(f.get('load_kg') is not None for f in frames) else None,
    }


def main():
    cases = [prepare_run(folder) for folder in sorted(RUNS.iterdir()) if (folder / 'sensors.csv').exists()]
    notes = [
        'Sensor rows and judgement columns are the Pi sensor_feed.py output (team/analysis 2026-10-08), unchanged.',
        'a_fwd / yaw_rate / v_imu are derived from imu1 (bias from the first 0.5 s, linear drift removed);'
        ' whether a run starts/ends at rest is taken from its Korean title keywords.',
        'Recorded imu1 roll/pitch are accelerometer tilt estimates (they track ax/ay); the twin derives body'
        ' tilt from acceleration instead and shows the recorded angles as data.',
        'Camera grid (x -0.27..0.27, y 0.05..0.50 m, 1 cm) maps to bed X (opposite sign) and bed Y;'
        ' the camera is at the N (front) end looking toward S. Heights are mm above the room floor; the plate'
        ' top is 208 mm above the floor (live_twin.py --plate-above-floor).',
        'Bed: load cells NW/NE/SW/SE 250 x 392 mm, N = front (belt c1), S = rear (belt c2). RATED_KG 15.',
    ]
    out = {'cases': cases, 'notes': notes, 'support_mm': [250, 392], 'rated_kg': 15.0,
           'plate_above_floor_m': 0.208}
    (ROOT / 'live_data.json').write_text(json.dumps(out, ensure_ascii=False, allow_nan=False), encoding='utf-8')
    for c in cases:
        m = c['motion']
        print(f"{c['id']} {c['title']:<18s} rows {c['rows']:3d} {c['duration']:5.2f}s cam {c['camera']['frames']:3d}"
              f" moving={m['moving']!s:5s} rest {m['starts_at_rest']!s:5s}/{m['ends_at_rest']!s:5s}"
              f" vpk {m['v_imu_peak']:5.2f} yaw {m['yaw_total_deg']:7.1f} risk {c['risk_max']} load {c['load_kg_mean']}")
    print('cases', len(cases), '->', ROOT / 'live_data.json')


if __name__ == '__main__':
    main()
