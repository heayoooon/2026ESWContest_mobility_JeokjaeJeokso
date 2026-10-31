"""Receive the sensor Pi's live frames, so the twin shows the real cargo bed as it is now.

The sensor Pi (~/scss_live/live_twin.py) streams to the display Pi's digital twin over TCP: each frame is
[4-byte length][4-byte header length][header JSON][zlib heightmap][zlib colours][zlib depth row deltas][JPEG],
about 10 Hz (the D435i runs at 30 fps, the Pi processes every third frame). The header carries the latest
sensor row (load cells, belts, IMU and the Pi's own judgement) and the camera calibration. The display Pi only
draws what it receives and forwards nothing, so the sensor Pi mirrors the same frames to this PC
(live_twin.py --mirror <this PC>:5010, its own sender thread: a twin that is off never slows the display).

This module listens on PORT, keeps only the newest frame and decodes it for the twin's measured surface
(measured_surface.DepthSurface) and the team's judgement. One sender at a time; a new connection replaces it.
"""
import json
import socket
import struct
import threading
import time
import zlib
from collections import deque

import numpy as np

PORT = 5010
STALE_S = 2.          # no frame for this long: shown as 'connection lost'


def decode(blob):
    """Display frame body -> dict(header, sensors, depth (H, W) uint16 mm, jpeg bytes, grid, camera)."""
    hl = struct.unpack_from('!I', blob, 0)[0]
    header = json.loads(blob[4:4 + hl])
    o = 4 + hl
    hb, cb = header['len']
    g = header['grid']
    # 1 cm height map (int16 mm) and cell colours: the display-style boxes (cargo_boxes) use them
    height = np.frombuffer(zlib.decompress(blob[o:o + hb]), '<i2').reshape(g['ny'], g['nx'])
    colour = np.frombuffer(zlib.decompress(blob[o + hb:o + hb + cb]), np.uint8).reshape(g['ny'], g['nx'], 3)
    o += hb + cb
    surface = header['surface']
    db, rb = surface['lens']
    intr = surface['intr']
    # visible-only depth in mm, stored as int16 row deltas (prepend 0) - same as prepare_live_runs.read_tws
    delta = np.frombuffer(zlib.decompress(blob[o:o + db]), '<i2').reshape(intr['height'], intr['width'])
    depth = np.cumsum(delta.astype(np.int32), axis=1).astype(np.uint16)
    o += db
    camera = {'intr': intr, 'R': surface['R'], 'ref_h': (header.get('calib') or {}).get('ref_h'),
              'cam_accel': header.get('cam_accel')}
    return {'header': header, 'sensors': header.get('sensors'), 'stats': header.get('stats') or {},
            'grid': header['grid'], 'camera': camera, 'depth': depth, 'jpeg': bytes(blob[o:o + rb]),
            'height': height, 'colour': colour,
            'seq': header.get('seq'), 'send_t': header.get('send_t')}


def read_exact(conn, count):
    data = bytearray()
    while len(data) < count:
        part = conn.recv(count - len(data))
        if not part:
            return None
        data.extend(part)
    return bytes(data)


class LiveReceiver:
    def __init__(self, port=PORT):
        self.port = port
        self.lock = threading.Lock()
        self.frame = None              # newest decoded frame
        self.count = 0                 # frames received since start (the twin's 'new frame' test)
        self.arrivals = deque(maxlen=30)
        self.delay = None              # receive time - the Pi's send time (s), clocks permitting
        self.peer = None
        self.error = ''
        self.server = None
        self.running = False

    # ------------------------------------------------------------------ control
    def start(self):
        if self.running:
            return True, f'listening on port {self.port}'
        try:
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(('0.0.0.0', self.port))
            server.listen(2)
            server.settimeout(1.)
        except OSError as e:
            self.error = str(e)
            return False, f'port {self.port}: {e}'
        self.server, self.running, self.error = server, True, ''
        self.frame, self.count = None, 0
        self.arrivals.clear()
        threading.Thread(target=self._accept, daemon=True).start()
        return True, f'listening on port {self.port}'

    def stop(self):
        self.running = False
        if self.server is not None:
            try:
                self.server.close()
            except OSError:
                pass
        self.server = None

    # ------------------------------------------------------------------ state for the twin
    def latest(self):
        with self.lock:
            return self.frame, self.count

    def hz(self):
        a = list(self.arrivals)
        return (len(a) - 1) / (a[-1] - a[0]) if len(a) > 2 and a[-1] > a[0] else None

    def receiving(self):
        return bool(self.arrivals) and time.time() - self.arrivals[-1] < STALE_S

    def status(self):
        if not self.running:
            return 'Live: off' + (f' ({self.error})' if self.error else '')
        if self.receiving():
            hz = self.hz()
            delay = f', delay {self.delay:.2f} s' if self.delay is not None and -1 < self.delay < 10 else ''
            return f'Live: receiving from {self.peer} ({hz:.1f} Hz{delay})' if hz else f'Live: receiving from {self.peer}'
        if self.arrivals:
            return 'Live: connection lost - waiting for the sensor Pi'
        return f'Live: waiting for the sensor Pi (port {self.port})'

    # ------------------------------------------------------------------ threads
    def _accept(self):
        while self.running:
            try:
                conn, addr = self.server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            # a reconnecting Pi opens a new connection: serve it (the old one ends on its own)
            threading.Thread(target=self._serve, args=(conn, addr[0]), daemon=True).start()

    def _serve(self, conn, peer):
        conn.settimeout(10.)
        try:
            while self.running:
                head = read_exact(conn, 4)
                if head is None:
                    break
                size = struct.unpack('!I', head)[0]
                if not 0 < size < 20_000_000:
                    break
                blob = read_exact(conn, size)
                if blob is None:
                    break
                try:
                    frame = decode(blob)
                except (ValueError, KeyError, zlib.error, struct.error) as e:
                    self.error = f'bad frame: {e}'
                    continue
                now = time.time()
                frame['recv_t'] = now
                with self.lock:
                    self.frame = frame
                    self.count += 1
                self.peer = peer
                self.arrivals.append(now)
                if frame.get('send_t'):
                    self.delay = now - float(frame['send_t'])
        except OSError:
            pass
        finally:
            conn.close()
