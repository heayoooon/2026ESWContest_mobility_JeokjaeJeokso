"""디스플레이 파이: 실시간 디지털 트윈 뷰어 (pygame + OpenGL, 800x480 DSI 화면).

센서 파이가 보내는 TCP v2 표면 프레임을 받아 (--udp로 이전 UDP 노드도 지원)
  왼쪽: 실제 깊이 픽셀의 컬러 3D 표면 / 포인트 / 높이색 + 관측 ROI 기준 격자
  오른쪽: 총하중, 4모서리 하중 + 무게중심(평면도), 벨트 장력, 기울기, G-볼, 경고
화면 드래그 = 회전, 탭 = 표현 전환, 시점 전환 버튼 = 4개 고정 시점, ESC/q = 종료

usage: python3 twin_view.py [port=5010] [--window]
  ssh 로 띄울 때: XDG_RUNTIME_DIR=/run/user/1000 WAYLAND_DISPLAY=wayland-0 SDL_VIDEODRIVER=wayland
"""
import collections, io, json, math, os, socket, struct, sys, threading, time, zlib
from PIL import Image
import numpy as np
import pygame
from surface_core import rays, surface_indices, valid_triangles, DepthSmoother
from transport_core import Assembler, read_exact
from pygame.locals import DOUBLEBUF, OPENGL, FULLSCREEN, QUIT, KEYDOWN, K_ESCAPE, K_q, \
    MOUSEBUTTONDOWN, MOUSEBUTTONUP, MOUSEMOTION
from OpenGL.GL import *  # noqa: F401,F403

W, H = 800, 480
VIEW_W = 500                                   # 3D 영역 폭, 나머지는 패널
HERE = os.path.dirname(os.path.abspath(__file__))
FONT = os.path.join(HERE, "fonts", "NanumGothic.ttf")            # 나눔고딕 (OFL). 시스템에 한글 폰트가 없다
FONT_BOLD = os.path.join(HERE, "fonts", "NanumGothicBold.ttf")
FONT_LATIN = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
LIGHT = np.array([0.35, -0.55, 0.76], np.float32); LIGHT /= np.linalg.norm(LIGHT)
COLOR_LUT = np.clip((np.arange(256,dtype=np.float32)/255)**.70*1.12,0,1)


# ---------------------------------------------------------------- 수신
class Receiver(threading.Thread):
    def __init__(self, port, transport='tcp'):
        super().__init__(daemon=True)
        self.transport=transport
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM if transport=='tcp' else socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
        self.sock.bind(("0.0.0.0", port))
        if transport=='tcp': self.sock.listen(1)
        self.lock = threading.Lock()
        self.latest, self.new = None, False
        self.partial, self.last_seq = {}, None
        self.session = None
        self.ok = self.lost = 0
        self.arrivals = collections.deque(maxlen=50)
        self.decode_ms = 0.0
        self.run_metrics = None
        self.assembler = Assembler()

    def messages(self):
        if self.transport=='tcp':
            while True:
                conn,_=self.sock.accept()
                conn.settimeout(5)
                try:
                    while True:
                        prefix=read_exact(conn,4)
                        if prefix is None: break
                        length=struct.unpack('!I',prefix)[0]
                        if not 4<=length<=2_000_000: break
                        blob=read_exact(conn,length)
                        if blob is None: break
                        yield blob,time.time()
                except OSError:
                    pass
                finally:
                    conn.close()
        else:
            while True:
                data,_=self.sock.recvfrom(4096)
                now=time.time()
                assembled=self.assembler.feed(data,now)
                if assembled is not None: yield assembled[1],now

    def run(self):
        for blob,now in self.messages():
            t0 = time.perf_counter()
            try:
                frame = self.decode(blob)
                seq=frame['hdr']['seq']
            except Exception as e:                      # 깨진 패킷은 건너뛴다
                print("decode error", e)
                continue
            self.decode_ms = (time.perf_counter() - t0) * 1000
            frame["recv_t"] = now
            session = frame['hdr'].get('session')
            if session != self.session:
                self.last_seq = None
                self.session = session
                self.arrivals.clear()
            if self.last_seq is not None:
                gap = seq - self.last_seq
                if gap > 1 and gap < 1000:
                    self.lost += gap - 1
                elif gap <= 0 and gap > -1000:          # 순서 뒤바뀜 -> 버림
                    continue
            self.last_seq = seq
            run_key=(session,frame['hdr']['run'])
            t_run=frame['hdr']['t_run']
            if self.run_metrics and (self.run_metrics['key'] != run_key or t_run < self.run_metrics['t_run']):
                m=self.run_metrics
                print('RUN %s received %d lost %d active %.2f Hz' %
                      (m['key'][1],m['count'],self.lost-m['lost'],
                       (m['count']-1)/max(m['last']-m['first'],.001)),flush=True)
                self.run_metrics=None
            if self.run_metrics is None:
                self.run_metrics=dict(key=run_key,count=0,lost=self.lost,first=now,last=now,t_run=t_run)
            self.run_metrics.update(count=self.run_metrics['count']+1,last=now,t_run=t_run)
            self.ok += 1
            self.arrivals.append(now)
            with self.lock:
                self.latest, self.new = frame, True

    @staticmethod
    def decode(blob):
        hl = struct.unpack("!I", blob[:4])[0]
        hdr = json.loads(blob[4:4 + hl])
        g = hdr["grid"]
        lh, lc = hdr["len"]
        body = blob[4 + hl:]
        h = np.frombuffer(zlib.decompress(body[:lh]), np.int16).reshape(g["ny"], g["nx"])
        col = np.frombuffer(zlib.decompress(body[lh:lh + lc]), np.uint8).reshape(g["ny"], g["nx"], 3)
        hm = np.where(h == g["nodata"], np.nan, h.astype(np.float32) / 1000.0)
        frame = dict(hdr=hdr, h=hm, col=col)
        if 'surface' in hdr:
            sf = hdr['surface']; intr = sf['intr']; ld, lr = sf['lens']
            start = lh+lc
            delta_codec=sf.get('depth_codec')=='delta-zlib'
            dep=np.frombuffer(zlib.decompress(body[start:start+ld]), '<i2' if delta_codec else '<u2').reshape(intr['height'],intr['width'])
            frame['depth'] = np.cumsum(dep,axis=1,dtype=np.int32).astype(np.uint16) if delta_codec else dep
            color_bytes=body[start+ld:start+ld+lr]
            if sf.get('color_codec')=='jpeg':
                frame['rgb']=np.asarray(Image.open(io.BytesIO(color_bytes)).convert('RGB'))
            else:
                frame['rgb'] = np.frombuffer(zlib.decompress(color_bytes), np.uint8).reshape(intr['height'],intr['width'],3)
        return frame

    def take(self):
        with self.lock:
            f, n = self.latest, self.new
            self.new = False
        return f, n

    def rate(self):
        now = time.time()
        return sum(1 for t in self.arrivals if now - t <= 2.0) / 2.0      # 최근 2초


# ---------------------------------------------------------------- 행렬
def perspective(fovy, aspect, n, f):
    t = 1 / math.tan(math.radians(fovy) / 2)
    return np.array([[t / aspect, 0, 0, 0], [0, t, 0, 0],
                     [0, 0, (f + n) / (n - f), 2 * f * n / (n - f)], [0, 0, -1, 0]], np.float32)


def look_at(eye, tgt, up=(0, 0, 1)):
    eye, tgt, up = (np.asarray(v, np.float32) for v in (eye, tgt, up))
    f = tgt - eye; f /= np.linalg.norm(f)
    s = np.cross(f, up); s /= np.linalg.norm(s)
    u = np.cross(s, f)
    m = np.eye(4, dtype=np.float32)
    m[0, :3], m[1, :3], m[2, :3] = s, u, -f
    m[:3, 3] = -m[:3, :3] @ eye
    return m


def height_cmap(t):
    """0..1 -> 파랑-청록-초록-노랑-빨강"""
    t = np.clip(t, 0, 1)[..., None]
    stops = np.array([[0.15, 0.25, 0.6], [0.1, 0.6, 0.7], [0.2, 0.75, 0.3], [0.95, 0.85, 0.2], [0.9, 0.3, 0.15]],
                     np.float32)
    x = t * (len(stops) - 1)
    i = np.clip(x.astype(int), 0, len(stops) - 2)[..., 0]
    fr = x[..., 0] - i
    return stops[i] * (1 - fr[..., None]) + stops[i + 1] * fr[..., None]


# ---------------------------------------------------------------- 메시
class Mesh:
    def __init__(self):
        self.key = None
        self.verts = self.cols = self.idx = None

    def build(self, frame, mode):
        hdr, h, col = frame["hdr"], frame["h"], frame["col"]
        g = hdr["grid"]
        ny, nx, c = g["ny"], g["nx"], g["cell"]
        if self.key != (nx, ny):
            self.key = (nx, ny)
            j, i = np.mgrid[0:ny - 1, 0:nx - 1]
            a = (j * nx + i).ravel(); b = a + 1; cc = a + nx; d = cc + 1
            self.tri = np.stack([np.stack([a, b, d], 1), np.stack([a, d, cc], 1)], 1).reshape(-1, 3).astype(np.uint32)
        valid = ~np.isnan(h)
        z = np.where(valid, h, 0.0).astype(np.float32)
        ys, xs = np.mgrid[0:ny, 0:nx].astype(np.float32)
        X = g["x0"] + (xs + 0.5) * c
        Y = g["y0"] + (ys + 0.5) * c
        self.verts = np.ascontiguousarray(np.stack([X, Y, z], -1).reshape(-1, 3), np.float32)
        gy, gx = np.gradient(z, c)
        n = np.stack([-gx, -gy, np.ones_like(z)], -1)
        n /= np.linalg.norm(n, axis=-1, keepdims=True)
        shade = np.clip(0.30 + 0.75 * (n @ LIGHT), 0.25, 1.15)[..., None]
        if mode == 0:
            base = np.clip(col.astype(np.float32) / 255 * 1.5 + 0.04, 0, 1)   # 어두운 실내라 밝게
        else:
            base = height_cmap(z / 0.8)
        floor = (z < 0.04)[..., None]                    # 바닥(기준면)은 어둡게 -> 화물이 도드라지게
        base = np.where(floor, base * 0.4 + 0.06, base)
        self.cols = np.ascontiguousarray(np.clip(base * shade, 0, 1).reshape(-1, 3), np.float32)
        v = valid.ravel()
        keep = v[self.tri[:, 0]] & v[self.tri[:, 1]] & v[self.tri[:, 2]]
        keep &= np.ptp(z.ravel()[self.tri], axis=1) <= .03
        self.idx = np.ascontiguousarray(self.tri[keep].ravel())

    def draw(self):
        if self.idx is None or not len(self.idx):
            return
        glEnableClientState(GL_VERTEX_ARRAY); glEnableClientState(GL_COLOR_ARRAY)
        glVertexPointer(3, GL_FLOAT, 0, self.verts)
        glColorPointer(3, GL_FLOAT, 0, self.cols)
        glDrawElements(GL_TRIANGLES, len(self.idx), GL_UNSIGNED_INT, self.idx)
        glDisableClientState(GL_VERTEX_ARRAY); glDisableClientState(GL_COLOR_ARRAY)


class SurfaceMesh:
    """Observed camera surface; 3D discontinuities never become stretched curtains."""
    def __init__(self):
        self.key = None
        self.verts = self.cols = self.idx = None
        self.fallback = Mesh()
        self.smoother = DepthSmoother()
        self.last_t = None
        self.mode = 0

    def build(self, frame, mode):
        self.mode = mode
        self.is_surface = 'depth' in frame
        if not self.is_surface:
            self.fallback.build(frame, int(mode == 2)); return
        sf = frame['hdr']['surface']; intr = sf['intr']
        key = (intr['width'],intr['height'],tuple(np.asarray(sf['R']).ravel()))
        if key != self.key:
            self.key = key
            self.rays = rays(intr) @ np.asarray(sf['R'],np.float32).T
            self.triangles = surface_indices(frame['depth'].shape)
            self.smoother = DepthSmoother()
        stamp = (frame['hdr']['run'],frame['hdr']['t_run'])
        if self.last_t and (stamp[0] != self.last_t[0] or stamp[1] < self.last_t[1]):
            self.smoother = DepthSmoother()
        self.last_t = stamp
        depth = self.smoother.update(frame['depth']) * .001
        pts = self.rays*depth[...,None]
        pts[...,2] -= frame['hdr']['calib']['ref_h']
        valid = depth > 0
        self.verts = np.ascontiguousarray(pts.reshape(-1,3),np.float32)
        self.idx = valid_triangles(pts,valid,self.triangles)
        self.point_idx = np.ascontiguousarray(np.flatnonzero(valid).astype(np.uint32))
        # Fixed gamma avoids exposure pumping as white boxes move across the frame.
        color = COLOR_LUT[frame['rgb']]
        if mode == 2:
            color = height_cmap(pts[...,2]/.7)
        self.cols = np.ascontiguousarray(color.reshape(-1,3),np.float32)

    def draw(self):
        if not getattr(self,'is_surface',False):
            self.fallback.draw(); return
        glEnableClientState(GL_VERTEX_ARRAY); glEnableClientState(GL_COLOR_ARRAY)
        glVertexPointer(3,GL_FLOAT,0,self.verts); glColorPointer(3,GL_FLOAT,0,self.cols)
        if self.mode == 1:
            glPointSize(2.5)
            glDrawElements(GL_POINTS,len(self.point_idx),GL_UNSIGNED_INT,self.point_idx)
        else:
            glDrawElements(GL_TRIANGLES,len(self.idx),GL_UNSIGNED_INT,self.idx)
        glDisableClientState(GL_VERTEX_ARRAY); glDisableClientState(GL_COLOR_ARRAY)


# ---------------------------------------------------------------- 경고 (휴리스틱)
class Alerts:
    """센서 값만 보고 거는 단순 규칙 (임계값은 이번 4개 녹화에서 고름). 종류별로 한 줄씩."""

    def __init__(self):
        self.hist = collections.deque()                  # (t_run, total, cogx, cogy, vol)
        self.gz = 0.0
        self.active = {}                                 # 종류 -> (문구, 유지 끝 시각, 심각)
        self.run = None

    def hit(self, kind, text, hold, severe=False):
        self.active[kind] = (text, time.time() + hold, severe)

    def update(self, hdr):
        s = hdr.get("sensors") or {}
        t = hdr["t_run"]
        if hdr["run"] != self.run or (self.hist and t < self.hist[-1][0]):
            self.hist.clear(); self.active.clear(); self.run = hdr["run"]; self.gz = 0.0
        ax, ay, gz = s.get("imu1_ax_G"), s.get("imu1_ay_G"), s.get("imu1_gz_dps")
        if ax is not None and abs(ax) > 0.25:
            self.hit("accel", "급가속 · 급제동  ax %+.2f G" % ax, 1.5)
        if gz is not None:
            self.gz = 0.6 * self.gz + 0.4 * gz
            if abs(self.gz) > 20:
                self.hit("turn", "선회 중  요레이트 %+.0f°/s" % self.gz, 0.6)
        calm = (ax is None or abs(ax) < 0.15) and abs(self.gz) < 15
        tot, cx, cy = s.get("total_g"), s.get("cog_x_pct"), s.get("cog_y_pct")
        vol = (hdr.get("stats") or {}).get("volume_l")
        old = [r for r in self.hist if t - 3.0 <= r[0] <= t - 1.0]
        if old and tot is not None:
            bt = np.median([r[1] for r in old if r[1] is not None] or [tot])
            if abs(tot - bt) > 1500:
                self.hit("load", "하중 급변  %+.1f kg" % ((tot - bt) / 1000), 2.0, True)
            ox = [r[2] for r in old if r[2] is not None]; oy = [r[3] for r in old if r[3] is not None]
            if cx is not None and cy is not None and ox and oy:
                dcg = math.hypot(cx - np.median(ox), cy - np.median(oy))
                if dcg > 12:
                    self.hit("cog", "화물 쏠림  무게중심 %.0f%%p 이동" % dcg, 2.0, True)
            ov = [r[4] for r in old if r[4]]
            if calm and vol and ov and abs(vol / np.median(ov) - 1) > 0.5:   # 차가 흔들리면 카메라도 흔들려서 제외
                self.hit("shape", "화물 형상 변화  카메라 부피 %+.0f%%" % ((vol / np.median(ov) - 1) * 100), 2.0, True)
        self.hist.append((t, tot, cx, cy, vol))
        while self.hist and self.hist[0][0] < t - 3.5:
            self.hist.popleft()
        now = time.time()
        for k in [k for k, v in self.active.items() if v[1] < now]:
            del self.active[k]
        return [(v[0], v[2]) for v in self.active.values()]


# ---------------------------------------------------------------- HUD (pygame 2D -> 텍스처)
C_BG = (14, 18, 26, 235); C_FG = (225, 230, 238); C_DIM = (130, 140, 155)
C_ACC = (90, 180, 255); C_WARN = (255, 170, 60); C_BAD = (240, 80, 70); C_OK = (90, 210, 130)


class Hud:
    def __init__(self):
        self.surf = pygame.Surface((W, H), pygame.SRCALPHA)
        # 한글은 나눔고딕, 라틴/숫자/기호는 DejaVu (나눔의 ° ± · 가 작아서) -> 글자 묶음별로 골라 그린다
        self.f = {s: (pygame.font.Font(FONT_LATIN, s - 1), pygame.font.Font(FONT_BOLD if s >= 20 else FONT, s))
                  for s in (12, 14, 16, 20, 28)}
        self.cog_trail = collections.deque(maxlen=25)
        self.g_trail = collections.deque(maxlen=25)
        self.tex = glGenTextures(1)

    def render_mixed(self, s, size, color):
        lat, han = self.f[size]
        if not s:
            return pygame.Surface((1, 1), pygame.SRCALPHA)
        runs = []
        for ch in s:
            k = 1 if ("가" <= ch <= "힣" or "㄰" <= ch <= "㆏" or "ᄀ" <= ch <= "ᇿ") else 0
            if runs and runs[-1][0] == k:
                runs[-1][1] += ch
            else:
                runs.append([k, ch])
        imgs = [((han if k else lat).render(t, True, color), (han if k else lat)) for k, t in runs]
        asc = max(f.get_ascent() for _, f in imgs)
        hgt = max(asc - f.get_ascent() + im.get_height() for im, f in imgs)
        out = pygame.Surface((sum(im.get_width() for im, _ in imgs), hgt), pygame.SRCALPHA)
        x = 0
        for im, f in imgs:
            out.blit(im, (x, asc - f.get_ascent())); x += im.get_width()
        return out

    def text(self, s, size, pos, color=C_FG, anchor="topleft"):
        img = self.render_mixed(s, size, color)
        r = img.get_rect(**{anchor: pos})
        self.surf.blit(img, r)
        return r

    def bar(self, x, y, w, h, frac, color):
        pygame.draw.rect(self.surf, (40, 46, 58), (x, y, w, h), border_radius=3)
        pygame.draw.rect(self.surf, color, (x, y, int(w * max(0, min(1, frac))), h), border_radius=3)

    def render(self, frame, rx, alerts, link, mode):
        S = self.surf
        S.fill((0, 0, 0, 0))
        if frame is None:
            S.fill((10, 12, 18, 255))
            self.text("디지털 트윈 · 센서 파이 신호 대기 중...", 20, (W // 2, H // 2), C_DIM, "center")
            self.upload(); return
        hdr = frame["hdr"]; s = hdr.get("sensors") or {}; st = hdr.get("stats") or {}
        # ---- 3D 영역 위: 제목 + 진행
        pygame.draw.rect(S, (10, 14, 20, 230), (0, 0, VIEW_W, 46))
        self.text("CARGO / LIVE TWIN", 20, (14, 4), (130,220,244))
        self.text(hdr["scenario"] + " · 녹화", 14, (VIEW_W-12, 7), C_FG, "topright")
        frac = hdr["t_run"] / max(hdr["dur_run"], 0.1)
        self.bar(10, 34, VIEW_W - 20, 5, frac, C_ACC)
        self.text("%.1f / %.1f s" % (max(0, hdr["t_run"]), hdr["dur_run"]), 12, (VIEW_W - 10, 20), C_DIM, "topright")
        y = 52
        for a, severe in alerts[:3]:
            col = C_BAD if severe else C_WARN
            r = pygame.Rect(10, y, VIEW_W - 20, 26)
            pygame.draw.rect(S, col + (215,), r, border_radius=5)
            self.text(a, 16, (r.x + 10, r.y + 3), (20, 16, 10))
            y += 30
        # ---- 3D 영역 아래: 카메라 요약 + 링크
        pygame.draw.rect(S, (10, 14, 20, 190), (0, H - 40, VIEW_W, 40))
        g = hdr["grid"]
        self.text("실측 표면 · 미관측면 비움  |  바닥 기준 최고 %s mm" % st.get('max_mm','-'), 12, (10,H-38), C_FG)
        for x,w,label in ((12,168,('표면 · 컬러','포인트 클라우드','표면 · 높이')[mode]),(190,118,'시점 전환')):
            pygame.draw.rect(S,(28,48,65,245),(x,399,w,32),border_radius=8)
            pygame.draw.rect(S,(64,102,123),(x,399,w,32),1,border_radius=8)
            self.text(label,14,(x+w//2,406),C_ACC,'midtop')
        self.text('드래그로 회전',12,(VIEW_W-12,409),C_DIM,'topright')
        self.text('관측 ROI · 적재함 치수 미보정',12,(12,379),C_DIM)
        hz, lost, lat, fps, dec = link
        self.text("수신 %.1f Hz (목표 %d)  ·  손실 %d  ·  지연 %s  ·  화면 %.0f fps  ·  복원 %.0f ms" % (
            hz, hdr.get("hz", 10), lost, ("%.0f ms" % lat) if lat is not None else "-", fps, dec),
            12, (10, H - 20), C_OK if hz >= 0.9 * hdr.get("hz", 10) else C_WARN)
        # ---- 패널
        px = VIEW_W
        pygame.draw.rect(S, C_BG, (px, 0, W - px, H))
        x0 = px + 14
        tot = s.get("total_g")
        self.text("총 하중", 14, (x0, 8), C_DIM)
        self.text("%.2f kg" % (tot / 1000) if tot is not None else "-", 28, (x0, 24))
        age = hdr.get("sensor_age")
        self.text("센서 %s" % ("%.0f ms 전" % (age * 1000) if age is not None else "없음"), 12,
                  (W - 12, 12), C_DIM, "topright")
        # 평면도: NW=앞왼 NE=앞오 SW=뒤왼 SE=뒤오 (COG_CORNERS)
        bx, by, bw, bh = x0 + 26, 82, 220, 130
        pygame.draw.rect(S, (32, 38, 50), (bx, by, bw, bh), border_radius=6)
        pygame.draw.rect(S, (80, 90, 110), (bx, by, bw, bh), 1, border_radius=6)
        self.text("앞", 12, (bx + bw // 2, by - 14), C_DIM, "midtop")
        pygame.draw.line(S, (60, 68, 84), (bx + bw // 2, by), (bx + bw // 2, by + bh))
        pygame.draw.line(S, (60, 68, 84), (bx, by + bh // 2), (bx + bw, by + bh // 2))
        loads = [s.get(k) for k in ("NW_g", "NE_g", "SW_g", "SE_g")]
        known = [v for v in loads if v is not None]
        mx = max(known + [1])
        for (lx, ly), v, name in zip(((bx, by), (bx + bw, by), (bx, by + bh), (bx + bw, by + bh)), loads,
                                     ("NW", "NE", "SW", "SE")):
            if v is None:
                pygame.draw.circle(S, C_DIM, (lx, ly), 6, 1); continue
            share = v / tot if tot else 0.25
            col = C_OK if share < 0.35 else (C_WARN if share < 0.45 else C_BAD)
            pygame.draw.circle(S, col, (lx, ly), int(6 + 12 * max(0, v) / mx))
            self.text("%.1f" % (v / 1000), 12, (lx, ly + (20 if ly > by else -22)), C_FG, "midtop")
        cx, cy = s.get("cog_x_pct"), s.get("cog_y_pct")
        if cx is not None and cy is not None:
            p = (bx + bw * cx / 100, by + bh * cy / 100)
            self.cog_trail.append(p)
            for i, q in enumerate(self.cog_trail):
                pygame.draw.circle(S, (255, 220, 80, 40 + 8 * i), q, 2)
            pygame.draw.circle(S, (255, 220, 80), p, 7)
            pygame.draw.circle(S, (20, 20, 20), p, 7, 2)
            self.text("무게중심 %.0f%% / %.0f%%" % (cx, cy), 12, (x0, by + bh + 36), (255, 220, 80))
        # 장력
        y = by + bh + 58
        c1 = s.get("c1_N")
        self.text("벨트 장력", 14, (x0, y), C_DIM)
        self.text("%.1f N" % c1 if c1 is not None else "-", 16, (W - 14, y - 2), C_FG, "topright")
        self.bar(x0, y + 22, W - px - 28, 8, (c1 or 0) / 15, C_WARN if "stuck" in s.get("flags", "") else C_ACC)
        if "stuck" in s.get("flags", ""):
            self.text("장력 센서 값 고정 (stuck)", 12, (x0, y + 32), C_WARN)
        # 기울기 + G 볼
        y += 44
        roll, pitch = s.get("imu1_roll_deg"), s.get("imu1_pitch_deg")
        self.text("기울기", 14, (x0, y), C_DIM)
        self.text("롤 %+.1f°" % roll if roll is not None else "롤 -", 16, (x0, y + 20))
        self.text("피치 %+.1f°" % pitch if pitch is not None else "피치 -", 16, (x0, y + 42))
        gz = s.get("imu1_gz_dps")
        self.text("요 %+.0f°/s" % gz if gz is not None else "", 16, (x0, y + 64))
        gx0, gy0, gr = W - 70, y + 50, 46
        pygame.draw.circle(S, (32, 38, 50), (gx0, gy0), gr)
        pygame.draw.circle(S, (80, 90, 110), (gx0, gy0), gr, 1)
        pygame.draw.circle(S, (60, 68, 84), (gx0, gy0), gr // 2, 1)
        pygame.draw.line(S, (60, 68, 84), (gx0 - gr, gy0), (gx0 + gr, gy0))
        pygame.draw.line(S, (60, 68, 84), (gx0, gy0 - gr), (gx0, gy0 + gr))
        ax, ay = s.get("imu1_ax_G"), s.get("imu1_ay_G")
        if ax is not None and ay is not None:
            q = (gx0 + gr * max(-1, min(1, ay / 0.5)), gy0 - gr * max(-1, min(1, ax / 0.5)))
            self.g_trail.append(q)
            for i, t in enumerate(self.g_trail):
                pygame.draw.circle(S, (90, 180, 255, 30 + 8 * i), t, 2)
            pygame.draw.circle(S, C_BAD if abs(ax) > 0.25 else C_ACC, q, 6)
        self.text("±0.5G", 12, (gx0, gy0 + gr + 2), C_DIM, "midtop")
        self.upload()

    def upload(self):
        data = pygame.image.tobytes(self.surf, "RGBA", True)
        glBindTexture(GL_TEXTURE_2D, self.tex)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, W, H, 0, GL_RGBA, GL_UNSIGNED_BYTE, data)

    def draw(self):
        glViewport(0, 0, W, H)
        glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(0, W, 0, H, -1, 1)
        glMatrixMode(GL_MODELVIEW); glLoadIdentity()
        glDisable(GL_DEPTH_TEST); glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_TEXTURE_2D); glBindTexture(GL_TEXTURE_2D, self.tex); glColor4f(1, 1, 1, 1)
        glBegin(GL_QUADS)
        glTexCoord2f(0, 0); glVertex2f(0, 0)
        glTexCoord2f(1, 0); glVertex2f(W, 0)
        glTexCoord2f(1, 1); glVertex2f(W, H)
        glTexCoord2f(0, 1); glVertex2f(0, H)
        glEnd()
        glDisable(GL_TEXTURE_2D); glDisable(GL_BLEND)


# ---------------------------------------------------------------- 3D 장면
def draw_scene(frame, mesh, yaw, el, dist, tgt):
    hdr = frame["hdr"]; g = hdr["grid"]
    gx1 = g["x0"] + g["nx"] * g["cell"]; gy1 = g["y0"] + g["ny"] * g["cell"]
    eye = tgt + dist * np.array([math.cos(el) * math.sin(yaw), -math.cos(el) * math.cos(yaw), math.sin(el)], np.float32)
    glViewport(0, 0, VIEW_W, H)
    glEnable(GL_SCISSOR_TEST); glScissor(0, 0, VIEW_W, H)
    glClearColor(0.055, 0.08, 0.105, 1); glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glDisable(GL_SCISSOR_TEST)
    glMatrixMode(GL_PROJECTION); glLoadMatrixf(perspective(42, VIEW_W / H, 0.05, 20).T)
    glMatrixMode(GL_MODELVIEW); glLoadMatrixf(look_at(eye, tgt).T)
    glEnable(GL_DEPTH_TEST)
    # Reference grid only: no fabricated tray, wheels, hidden boxes or registered load cells.
    glColor3f(0.22, 0.26, 0.33); glBegin(GL_LINES)
    for x in np.arange(math.floor(g["x0"] * 10) / 10, gx1 + 1e-6, 0.1):
        glVertex3f(x, g["y0"], -0.002); glVertex3f(x, gy1, -0.002)
    for y in np.arange(math.floor(g["y0"] * 10) / 10, gy1 + 1e-6, 0.1):
        glVertex3f(g["x0"], y, -0.002); glVertex3f(gx1, y, -0.002)
    glEnd()
    glColor3f(.25,.62,.72); glLineWidth(2)
    glBegin(GL_LINE_LOOP)
    for x,y in ((g['x0'],g['y0']),(gx1,g['y0']),(gx1,gy1),(g['x0'],gy1)):
        glVertex3f(x,y,0)
    glEnd(); glLineWidth(1)
    mesh.draw()


def main():
    port = int(next((a for a in sys.argv[1:] if a.isdigit()), 5010))
    windowed = "--window" in sys.argv
    capture_dir = sys.argv[sys.argv.index('--capture-dir')+1] if '--capture-dir' in sys.argv else None
    captured = set()
    pygame.init()
    pygame.display.set_mode((W, H), DOUBLEBUF | OPENGL | (0 if windowed else FULLSCREEN))
    pygame.display.set_caption("Digital Twin")
    pygame.mouse.set_visible(windowed)
    rx = Receiver(port,'udp' if '--udp' in sys.argv else 'tcp'); rx.start()
    hud, mesh, alerts = Hud(), SurfaceMesh(), Alerts()
    frame, mode = None, 0
    yaw0, el, dist, tgt = math.radians(0), math.radians(55), 1.04, None
    presets = [(0,55),(-35,35),(50,35),(0,82)]
    preset = 0
    drag, user_t, yaw_user = None, 0.0, 0.0
    clock = pygame.time.Clock()
    t_log, frames, mesh_ms, lat = time.time(), 0, [], None
    hud.render(None, rx, [], None, mode)
    print("listening",rx.transport,port)
    while True:
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYDOWN and e.key in (K_ESCAPE, K_q)):
                pygame.quit(); return
            if e.type == MOUSEBUTTONDOWN and e.pos[0] < VIEW_W:
                drag = [e.pos, e.pos, False]
            elif e.type == MOUSEMOTION and drag:
                dx, dy = e.pos[0] - drag[1][0], e.pos[1] - drag[1][1]
                drag[1] = e.pos
                if abs(e.pos[0] - drag[0][0]) + abs(e.pos[1] - drag[0][1]) > 8:
                    drag[2] = True
                yaw_user -= dx * 0.008
                el = min(math.radians(85), max(math.radians(8), el + dy * 0.006))
                user_t = time.time()
            elif e.type == MOUSEBUTTONUP and drag:
                if not drag[2]:
                    if 190 <= e.pos[0] <= 308 and 399 <= e.pos[1] <= 431:
                        preset = (preset+1)%len(presets)
                        yaw0,el = map(math.radians,presets[preset]); yaw_user=0
                    else:
                        mode = (mode+1)%3
                    if frame is not None:
                        mesh.build(frame, mode)
                drag = None
        f, new = rx.take()
        if new:
            frame = f
            t0 = time.perf_counter()
            mesh.build(frame, mode)
            mesh_ms.append((time.perf_counter() - t0) * 1000)
            lat = (frame["recv_t"] - frame["hdr"]["send_t"]) * 1000
            act = alerts.update(frame["hdr"])
            hud.render(frame, rx, act, (rx.rate(), rx.lost, lat, clock.get_fps(), rx.decode_ms), mode)
        elif frame is not None and time.time() - frame["recv_t"] > 5.0:
            frame = None
            hud.render(None, rx, [], None, mode)
        sway = 0.0
        if frame is not None:
            st = frame["hdr"].get("stats") or {}
            g = frame['hdr']['grid']
            tgt = np.array([g['x0']+g['nx']*g['cell']/2, g['y0']+g['ny']*g['cell']/2, .40],np.float32)
            draw_scene(frame, mesh, yaw0 + yaw_user + sway, el, dist, tgt)
        else:
            glViewport(0, 0, W, H); glClearColor(0.04, 0.05, 0.07, 1); glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        hud.draw()
        if capture_dir and frame is not None and frame['hdr'].get('capture'):
            stamp = (frame['hdr']['run'],frame['hdr']['seq'])
            if stamp not in captured:
                os.makedirs(capture_dir,exist_ok=True)
                pixels = glReadPixels(0,0,W,H,GL_RGB,GL_UNSIGNED_BYTE)
                img = pygame.image.frombytes(pixels,(W,H),'RGB',True)
                pygame.image.save(img,os.path.join(capture_dir,stamp[0]+'.png'))
                with open(os.path.join(capture_dir,stamp[0]+'.json'),'w') as out:
                    json.dump(frame['hdr'],out,ensure_ascii=False,indent=2)
                captured.add(stamp)
                if len(captured)>100: captured={stamp}
        pygame.display.flip()
        clock.tick(30)
        frames += 1
        if time.time() - t_log >= 5:
            print("recv %.1f Hz  ok %d lost %d  render %.1f fps  mesh %.1f ms  decode %.1f ms  latency %s  repaired %d" % (
                rx.rate(), rx.ok, rx.lost, frames / (time.time() - t_log),
                np.median(mesh_ms) if mesh_ms else 0, rx.decode_ms, "%.0f ms" % lat if lat is not None else "-",rx.assembler.repaired),
                flush=True)
            t_log, frames, mesh_ms = time.time(), 0, []


if __name__ == "__main__":
    main()
