"""Stream the twin to the display Pi's digital twin (display/twin) as if the sensor Pi were sending it.

The display kiosk (display/twin/server/twin_server.py) accepts one TCP sender on port 5010 and draws each
frame: [4-byte length][4-byte header length][header JSON][zlib heightmap][zlib colours][zlib depth][JPEG]
(format: display/twin/README.md). It has no replay of its own and drops a sender that is quiet for 5 s, so
this module keeps sending at 10 Hz until it is told to stop, and reconnects if the display restarts:

  * live (Send to display)   - the twin's current readings, every 0.1 s: replay or sandbox, slider changes
                               (strap preload / stiffness, slope, mass...) show on the display at once;
  * recording (download page / Claude) - a saved recording, looped until stopped.

Camera bodies come from the sensor Pi's own frames (twin.tws) of the recorded run at the replay time. In the
physics sandbox the run's first frame is used and its cargo cells are moved by the cargo displacement the
twin measures on the plate (and removed when the cargo has left the plate), so the display's 3D boxes
follow the simulated cargo. The header is rewritten (own session, run 'isaac_live' / 'isaac_<recording>',
t_run counting up). If the sensor Pi is streaming live the display reads that connection first and this one
waits until the live sender stops.
"""
import json
import os
import socket
import struct
import threading
import time
import zlib
from pathlib import Path

import numpy as np

DEFAULT_HOST = os.environ.get('TWIN_DISPLAY', '100.90.251.42')   # display Pi on the team tailnet
PORT = 5010
HZ = 10.
ISAAC_ONLY = {'rec_time_s', 'clock', 'mode', 'run', 'replay_time_s', 'vehicle_kmh', 'lap_s_m', 'lane_d_m',
              'heading_deg', 'road_slope_deg', 'road_bank_deg', 'cargo_mass_kg', 'straps_connected', 'belt_c1_connected', 'belt_c2_connected', 'preload_c1_N', 'preload_c2_N',
              'shift_reason', 'shift_reason_ko', 'estimated', 't', 'cargo_offset_mm', 'cargo_on_plate',
              'cargo_secured', 'friction', 'grip_demand_N', 'grip_capacity_N', 'lv_move_twin', 'recorded_judgement'}


def read_tws(path):
    """(wall, blob) per frame of a sensor-Pi twin.tws (each blob is exactly one display frame body)."""
    data = Path(path).read_bytes()
    frames, off = [], 0
    while off + 12 <= len(data):
        wall, length = struct.unpack_from('!dI', data, off)
        off += 12
        frames.append((wall, data[off:off + length]))
        off += length
    return frames


class RunFrames:
    """Camera frames of one recorded run, indexed by replay time (aligned by wall clock like the twin)."""

    def __init__(self, folder):
        self.frames = read_tws(folder / 'twin.tws')
        wall0 = None
        with (folder / 'sensors.csv').open(encoding='utf-8-sig') as stream:
            header = stream.readline().strip().split(',')
            first = stream.readline().strip().split(',')
            if 'wall' in header:
                try:
                    wall0 = float(first[header.index('wall')])
                except (ValueError, IndexError):
                    pass
        self.times = [(wall - wall0) if wall0 else i * .1 for i, (wall, _) in enumerate(self.frames)]

    def at(self, t):
        best = min(range(len(self.times)), key=lambda i: abs(self.times[i] - t))
        return self.frames[best][1]


def sensors_from_row(row):
    """Recorder / live row -> the sensor dictionary the display expects (sensor Pi names)."""
    s = {}
    for key, value in row.items():
        if key in ISAAC_ONLY or value == '':
            continue
        if isinstance(value, str) and value[:1] in '[{':
            try:
                value = json.loads(value)
            except ValueError:
                pass
        s[key] = value
    for axis, length in (('x', 250.), ('y', 392.)):
        if s.get(f'cog_{axis}_pct') is None and isinstance(s.get(f'cog_{axis}_mm'), (int, float)):
            s[f'cog_{axis}_pct'] = round(100 * s[f'cog_{axis}_mm'] / length, 1)
    s['algo'] = s.get('algo') or 'isaac-twin'   # 4-level judgement (display: is_4level)
    s.pop('recorded_judgement', None)
    return s


def move_cargo(blob, offset_mm=None, on_plate=True):
    """Shift the cargo cells of a frame's heightmap by the plate displacement (mm, plate X / Y = cog_x / cog_y
    directions), or clear them when the cargo has left the plate. Display registration: plate X = camera x,
    plate Z (rear +) = -camera y, so +cog_y (towards the rear belt) is -grid y."""
    hl = struct.unpack_from('!I', blob, 0)[0]
    header = json.loads(blob[4:4 + hl])
    g = header['grid']
    lh, lc = header['len']
    body = blob[4 + hl:]
    h = np.frombuffer(zlib.decompress(body[:lh]), np.int16).reshape(g['ny'], g['nx']).copy()
    col = np.frombuffer(zlib.decompress(body[lh:lh + lc]), np.uint8).reshape(g['ny'], g['nx'], 3).copy()
    valid = h != g['nodata']
    if not valid.any():
        return blob
    floor = int(np.percentile(h[valid], 10))
    cargo = valid & (h > floor + 30)                      # cells more than 3 cm above the plate
    plate_h = np.where(valid, floor, g['nodata']).astype(np.int16)
    plate_c = col.copy()
    if cargo.any():
        plate_c[cargo] = np.median(col[valid & ~cargo], axis=0) if (valid & ~cargo).any() else 90
    if on_plate and offset_mm is not None:
        cell_mm = g['cell'] * 1000.
        dx, dy = int(round(offset_mm[0] / cell_mm)), int(round(-offset_mm[1] / cell_mm))
        ys, xs = np.nonzero(cargo)
        ys2, xs2 = ys + dy, xs + dx
        keep = (ys2 >= 0) & (ys2 < g['ny']) & (xs2 >= 0) & (xs2 < g['nx'])
        plate_h[ys2[keep], xs2[keep]] = h[ys[keep], xs[keep]]
        plate_c[ys2[keep], xs2[keep]] = col[ys[keep], xs[keep]]
    elif on_plate:
        return blob
    hb, cb = zlib.compress(plate_h.astype('<i2').tobytes(), 6), zlib.compress(plate_c.tobytes(), 6)
    header['len'] = [len(hb), len(cb)]
    head = json.dumps(header, ensure_ascii=False).encode()
    return struct.pack('!I', len(head)) + head + hb + cb + body[lh + lc:]


def rewrite(blob, header_update):
    hl = struct.unpack_from('!I', blob, 0)[0]
    header = json.loads(blob[4:4 + hl])
    header.update(header_update)
    hb = json.dumps(header, ensure_ascii=False).encode()
    return struct.pack('!I', len(hb)) + hb + blob[4 + hl:]


class DisplaySender:
    def __init__(self, live_runs):
        self.live_runs = Path(live_runs)
        self.runs = {}
        self.thread = None
        self.stop_event = threading.Event()
        self.status = 'Display: idle'
        self.busy = False
        self.live = False
        self.latest = None            # newest live row, offered by the twin's main loop
        self.frames_sent = 0

    def frames_for(self, run_id):
        if run_id not in self.runs:
            folder = next((p for p in self.live_runs.glob(run_id + '*') if (p / 'twin.tws').exists()), None)
            if folder is None:
                raise FileNotFoundError(f'no twin.tws for run {run_id}')
            self.runs[run_id] = RunFrames(folder)
        return self.runs[run_id]

    # ---------------------------------------------------------------- control
    def start_live(self, host=None, port=PORT):
        if self.busy:
            return False, 'already sending - stop it first'
        self.live, self.latest = True, None
        return self._start(self._live_rows, host, port, 'live twin')

    def send(self, rec_dir, host=None, port=PORT):
        """Loop a saved recording on the display until stop()."""
        if self.busy:
            return False, 'already sending - stop it first'
        rec_dir = Path(rec_dir)
        if not (rec_dir / 'twin_log.jsonl').exists():
            return False, f'{rec_dir.name}: no twin_log.jsonl (stop the recording first)'
        rows = [json.loads(line) for line in (rec_dir / 'twin_log.jsonl').read_text(encoding='utf-8').splitlines() if line]
        if not rows:
            return False, f'{rec_dir.name} is empty'
        self.live = False
        return self._start(lambda: self._recording_rows(rows), host, port, rec_dir.name, run='isaac_' + rec_dir.name)

    def offer(self, row):
        self.latest = row

    def stop(self):
        self.stop_event.set()

    def _start(self, source, host, port, label, run='isaac_live'):
        self.stop_event.clear()
        self.busy = True
        self.frames_sent = 0
        self.thread = threading.Thread(target=self._run, args=(source, host or DEFAULT_HOST, port, label, run), daemon=True)
        self.thread.start()
        return True, f'sending {label} to {host or DEFAULT_HOST}:{port} until stopped'

    # ---------------------------------------------------------------- row sources (yield (row, t_run))
    def _live_rows(self):
        start = time.time()
        while not self.stop_event.is_set():
            if self.latest is not None:
                yield self.latest, time.time() - start
            self.stop_event.wait(1 / HZ)

    def _recording_rows(self, rows):
        duration = rows[-1]['rec_time_s'] + 1 / HZ
        start, lap = time.time(), 0
        while not self.stop_event.is_set():
            for row in rows:
                due = start + lap * duration + row['rec_time_s']
                wait = due - time.time()
                if wait > 0 and self.stop_event.wait(wait):
                    return
                yield row, lap * duration + row['rec_time_s']
            lap += 1

    # ---------------------------------------------------------------- sender thread
    def frame(self, row, t_run, session, seq, run):
        frames = self.frames_for(str(row['run']))
        physics = row.get('mode') == 'physics'
        template = frames.frames[0][1] if physics else frames.at(float(row.get('replay_time_s') or 0))
        if physics:
            template = move_cargo(template, row.get('cargo_offset_mm'), row.get('cargo_on_plate', True))
        now = time.time()
        return rewrite(template, {
            'session': session, 'seq': seq, 'send_t': now, 'frame_ts': now, 'run': run,
            'scenario': 'Isaac_Sim_' + ('physics' if physics else str(row['run'])), 'src': ['isaac', row.get('mode', '')],
            't_run': round(t_run, 3), 'dur_run': None, 'sensors': sensors_from_row(row), 'sensor_age': .05, 'hz': HZ})

    def _run(self, source, host, port, label, run):
        session, seq, sock, waited = os.urandom(8).hex(), 0, None, 0.
        try:
            for row, t_run in source():
                if sock is None:
                    try:
                        sock = socket.create_connection((host, port), timeout=3)
                        sock.settimeout(6)
                        waited = 0.
                    except OSError as error:
                        waited += 2
                        self.status = (f'Display: waiting for {host}:{port} ({error.__class__.__name__}, {waited:.0f} s) '
                                       '- is the display app on?')
                        if self.stop_event.wait(2):
                            break
                        continue
                try:
                    blob = self.frame(row, t_run, session, seq, run)
                    sock.sendall(struct.pack('!I', len(blob)) + blob)
                    seq += 1
                    self.frames_sent = seq
                    self.status = f'Display: {"live" if self.live else label} - {seq} frames sent (until Stop display)'
                except socket.timeout:
                    self.status = 'Display: not reading - is the sensor Pi streaming live? (stop_live.sh)'
                    sock.close()
                    sock = None
                except OSError:
                    self.status = 'Display: connection lost - reconnecting...'
                    sock.close()
                    sock = None
            self.status = f'Display: stopped after {seq} frames'
        except Exception as error:
            self.status = f'Display: failed ({error})'
        finally:
            if sock is not None:
                sock.close()
            self.busy = False
            self.live = False
