"""live_twin.py 가 쓰는 높이맵 · 프레임 보내기 (센서 파이 pi/twin/replay_twin.py · transport_core.py 에서 가져옴).

  ColorHeightMapper  깊이 -> 1cm 2.5D 높이맵 + 칸별 평균 컬러 (twin_core.HeightMapper 와 같은 규칙)
  ReliableSender     디스플레이 파이(display/twin, TCP 5010)로 [4바이트 길이][본문] 을 보내는 영구 연결
  send_packet        본문 = [4바이트 헤더 길이][헤더 JSON][바디들...]
"""
import json, socket, struct, sys
import numpy as np
try:
    from PIL import Image  # noqa: F401  (live_twin.py 가 JPEG 에 쓴다)
except ImportError:
    # ~/d435i/.venv 에 Pillow 가 없으면 시스템(Debian) Pillow 를 쓴다 (같은 ABI)
    sys.path.append('/usr/lib/python3/dist-packages')
    from PIL import Image  # noqa: F401

MAX_RANGE = 2.5   # m, 이보다 먼 깊이는 버린다


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


class ReliableSender:
    """디스플레이마다 TCP 연결 하나를 열어 두고 프레임을 [4바이트 길이][본문] 으로 보낸다. 끊기면 한 번 다시 연결."""

    def __init__(self):
        self.connections = {}

    def prepare(self, dests):
        for dest in dests:
            if dest not in self.connections:
                conn = socket.create_connection(dest, timeout=3)
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                conn.settimeout(.2)
                self.connections[dest] = conn

    def send_frame(self, dests, blob):
        packet = struct.pack('!I', len(blob)) + blob
        for dest in dests:
            for attempt in range(2):
                try:
                    self.prepare([dest])
                    self.connections[dest].sendall(packet)
                    break
                except OSError:
                    old = self.connections.pop(dest, None)
                    if old is not None:
                        old.close()
                    if attempt:
                        raise
        return len(packet), 1


def send_packet(sender, dests, seq, header, *bodies):
    """헤더 JSON + 바디들을 한 프레임으로 보낸다. (보낸 바이트, 조각 수)"""
    hb = json.dumps(header, ensure_ascii=False).encode()
    return sender.send_frame(dests, struct.pack('!I', len(hb)) + hb + b''.join(bodies))
