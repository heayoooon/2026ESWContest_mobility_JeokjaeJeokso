"""경보음 + 음성(Voice)과 새 HUD(카드형 패널, 경고/주의 표지판). twin_view.py 가 import 한다.

Voice: 새로 뜨거나 단계가 오른 알림에 한 번. 위험 = 빠른 '삐삐삐' x4 + '위험 위험', 경고 = '삐빅' x3 + '경고 경고',
       주의 = '띵-동' 차임 + '주의 주의' (밑에 신호음 반복), 그 뒤에 이유 음성 (sounds/reason/*.mp3).
Hud:   왼쪽 3D 위 상단 바 · 경고 표지판(새 경고 2.2 s 크게, 이후 작은 알림) · 테두리 점멸,
       오른쪽 카드 4장: 총하중(+추이), 적재함 평면도(모서리 하중 · 무게중심), 벨트 장력 게이지, 자세(수평계 · G 볼).
"""
import collections, math, os, time
import numpy as np
import pygame
import pygame.gfxdraw as gfx
from OpenGL.GL import *  # noqa: F401,F403

W, H = 800, 480
VIEW_W = 500
HERE = os.path.dirname(os.path.abspath(__file__))
FONT = os.path.join(HERE, "fonts", "NanumGothic.ttf")
FONT_BOLD = os.path.join(HERE, "fonts", "NanumGothicBold.ttf")
FONT_LATIN = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_LATIN_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

# 색
BG = (11, 15, 22); CARD = (21, 27, 38); CARD_HI = (27, 35, 49); EDGE = (44, 55, 74)
FG = (232, 237, 244); DIM = (128, 142, 162); FAINT = (70, 82, 102)
ACC = (72, 196, 255); OK = (88, 214, 141); WARN = (255, 184, 48); BAD = (244, 67, 54); GOLD = (255, 214, 92)


# ---------------------------------------------------------------- 소리
def _tone(freqs, dur, sr, vol=1.0, harm=True, decay=None):
    t = np.arange(int(sr * dur)) / sr
    y = np.zeros_like(t)
    for f in freqs:
        y += np.sin(2 * np.pi * f * t)
        if harm:                                          # 홀수 배음 -> '삐' 같은 날카로운 소리
            y += .30 * np.sin(6 * np.pi * f * t) + .12 * np.sin(10 * np.pi * f * t)
    y /= np.abs(y).max() + 1e-9
    env = np.ones_like(t); a = max(1, int(sr * .006))
    env[:a] = np.linspace(0, 1, a); env[-a:] = np.linspace(1, 0, a)
    if decay:
        env *= np.exp(-t / decay)
    return (y * env * vol).astype(np.float32)


def _place(buf, sig, at, sr):
    i = int(at * sr); j = min(len(buf), i + len(sig))
    if i < len(buf):
        buf[i:j] += sig[:j - i]


class Voice:
    """알림이 새로 뜨거나 단계가 오르면 한 번: 단계 소리('위험 위험' · '경고 경고' · '주의 주의') 뒤에 이유 음성
    (sounds/reason/<이름>.mp3, 예 cog_left = '화물이 왼쪽으로 쏠렸습니다.'). 이유 음성이 없으면 단계 소리만.
    같은 종류는 COOLDOWN 초 안에 같은 단계 이하로 다시 울리지 않는다 (깜빡임). 더 높은 단계가 뜨면 재생 중인 낮은 단계를 끊고,
    한꺼번에 여러 개가 뜨면 높은 단계부터 하나씩 차례로 말한다."""
    COOLDOWN = {3: 4.0, 2: 4.0, 1: 8.0}
    LEAD = {1: 'caution.mp3', 2: 'warning.mp3', 3: 'danger.mp3'}
    NAMES = {1: '주의', 2: '경고', 3: '위험'}
    GAP = .12                                            # 단계 음성과 이유 음성 사이 (초)

    def __init__(self):
        self.ok = False
        self.lead, self.reason, self.cache = {}, {}, {}
        try:
            if not pygame.mixer.get_init():
                pygame.mixer.init()
            self.sr, _, self.nch = pygame.mixer.get_init()
            d = os.path.join(HERE, 'sounds')
            self.lead = {lv: self._load(os.path.join(d, f)) for lv, f in self.LEAD.items() if os.path.exists(os.path.join(d, f))}
            self.lead.setdefault(3, self.lead[2])        # danger.mp3 가 없으면 위험도 '경고 경고'
            rd = os.path.join(d, 'reason')
            if os.path.isdir(rd):
                self.reason = {f[:-4]: self._load(os.path.join(rd, f), trim=True) for f in sorted(os.listdir(rd)) if f.endswith('.mp3')}
            self.ch = pygame.mixer.Channel(0)
            self.ok = True
        except Exception as e:
            print('sound off:', e, flush=True)
        self.prev = {}; self.pending = set(); self.last = {}
        self.playing = None; self.sound_id = None; self.played = 0

    def _load(self, path, trim=False):
        """mp3 -> 모노 float (최대 .72). trim 이면 앞뒤 무음을 잘라 단계 음성 바로 뒤에 붙게 한다."""
        v = pygame.sndarray.array(pygame.mixer.Sound(path)).astype(np.float32)
        v = (v.mean(1) if v.ndim == 2 else v) / 32768.0
        v *= .72 / (np.abs(v).max() + 1e-9)
        if trim:
            on = np.flatnonzero(np.abs(v) > .02)
            if len(on):
                v = v[max(0, on[0] - int(.03 * self.sr)): on[-1] + int(.12 * self.sr)]
        return v

    def sound(self, level, clip=None):
        """단계(1 주의 · 2 경고 · 3 위험) + 이유 음성 이름 -> pygame Sound. 처음 한 번 만들고 둔다."""
        key = (level, clip if clip in self.reason else None)
        if key not in self.cache:
            self.cache[key] = self._compose(*key)
        return self.cache[key]

    def _compose(self, level, clip):
        sr = self.sr
        v = self.lead[level]
        vd = len(v) / sr
        r = self.reason.get(clip)
        rd = 0.0 if r is None else self.GAP + len(r) / sr
        if level == 3:                                   # 위험: 높고 빠른 '삐삐삐' x4 -> 음성 + 밑에 빠른 삐삐 반복
            lead = 1.05
            buf = np.zeros(int((lead + vd + rd + .3) * sr), np.float32)
            hi, lo = _tone([1800], .07, sr, .6), _tone([1350], .07, sr, .6)
            for k in range(4):
                for j, s in enumerate((hi, lo, hi)):
                    _place(buf, s, k * .26 + j * .075, sr)
            t = lead
            while t < lead + vd:
                _place(buf, hi * .30, t, sr); _place(buf, lo * .30, t + .09, sr); t += .3
        elif level == 2:                                 # 경고: 삐빅 x3 -> 음성 + 밑에 작은 삐빅 반복
            lead = .95
            buf = np.zeros(int((lead + vd + rd + .3) * sr), np.float32)
            hi, lo = _tone([1450], .085, sr, .55), _tone([1050], .085, sr, .55)
            for k in range(3):
                _place(buf, hi, k * .30, sr); _place(buf, lo, k * .30 + .11, sr)
            t = lead
            while t < lead + vd:
                _place(buf, hi * .30, t, sr); _place(buf, lo * .30, t + .11, sr); t += .5
        else:                                            # 주의: 띵-동 차임 -> 음성 + 밑에 부드러운 '뚜'
            lead = .80
            buf = np.zeros(int((lead + vd + rd + .3) * sr), np.float32)
            _place(buf, _tone([880], .55, sr, .55, harm=False, decay=.22), 0, sr)
            _place(buf, _tone([660], .60, sr, .55, harm=False, decay=.26), .30, sr)
            t = lead
            while t < lead + vd:
                _place(buf, _tone([600], .14, sr, .16, harm=False), t, sr); t += .7
        _place(buf, v, lead, sr)
        if r is not None:                                # 이유는 신호음 없이 또렷하게
            _place(buf, r, lead + vd + self.GAP, sr)
        arr = (np.clip(buf, -1, 1) * 32000).astype(np.int16)
        if self.nch > 1:
            arr = np.repeat(arr[:, None], self.nch, 1)
        return pygame.sndarray.make_sound(np.ascontiguousarray(arr))

    def update(self, active):
        """active = Alerts.active {종류: (문구, 끝 시각, 심각[, 단계, 이유 음성 이름])}. 단계가 없으면 심각 = 경고, 아니면 주의."""
        now = time.time()
        cur = {k: ((v[3] if len(v) > 3 and v[3] else 2 if v[2] else 1), v[4] if len(v) > 4 else None)
               for k, v in active.items()}
        self.pending = {k for k in self.pending if k in cur} | {k for k, (lv, _) in cur.items() if lv > self.prev.get(k, 0)}
        self.prev = {k: lv for k, (lv, _) in cur.items()}
        if not self.ok:
            self.pending.clear()
            return
        while self.pending:
            kind = max(self.pending, key=lambda k: cur[k][0])
            lv, clip = cur[kind]
            if self.ch.get_busy() and lv <= (self.playing or 0):
                return                                   # 지금 소리가 끝나면 다음 프레임에 이어서
            self.pending.discard(kind)
            t, last_lv = self.last.get(kind, (-1e9, 0))
            if now - t < self.COOLDOWN[lv] and lv <= last_lv:
                continue
            clip = clip if clip in self.reason else None
            self.ch.play(self.sound(lv, clip)); self.playing = lv
            self.sound_id = '%d_%s' % (lv, clip) if clip else str(lv)
            self.last[kind] = (now, lv); self.played += 1
            print('voice', self.NAMES[lv], kind, clip or '-', flush=True)
            return


# ---------------------------------------------------------------- 그리기 도우미
def mix(c, bg, a):
    """투명색 대신 배경과 미리 섞은 불투명색 (gfxdraw 알파가 투명 표면에서 검게 나와서)."""
    return tuple(int(c[i] * a + bg[i] * (1 - a)) for i in range(3))


def rrect(S, color, rect, r=10, width=0):
    pygame.draw.rect(S, color, rect, width, border_radius=r)


def aacircle(S, color, c, r, fill=True):
    x, y, r = int(round(c[0])), int(round(c[1])), int(round(r))
    if r < 1:
        return
    if fill:
        gfx.filled_circle(S, x, y, r, color)
    gfx.aacircle(S, x, y, r, color)


def aapoly(S, color, pts, fill=True):
    pts = [(int(round(x)), int(round(y))) for x, y in pts]
    if fill:
        gfx.filled_polygon(S, pts, color)
    gfx.aapolygon(S, pts, color)


def triangle(cx, cy, size):
    h = size * math.sqrt(3) / 2
    return [(cx, cy - h * 2 / 3), (cx + size / 2, cy + h / 3), (cx - size / 2, cy + h / 3)]


def sign(S, cx, cy, size, severe):
    """경고: 빨간 삼각 + 흰 느낌표.  주의: 노란 삼각 + 검은 테두리 · 느낌표 (도로 주의 표지)."""
    outer, face, mark = ((255, 255, 255), BAD, (255, 255, 255)) if severe else ((20, 20, 20), (255, 202, 40), (20, 20, 20))
    aapoly(S, (0, 0, 0, 90), triangle(cx + 3, cy + 4, size * 1.04))            # 그림자
    aapoly(S, outer, triangle(cx, cy, size))
    aapoly(S, face, triangle(cx, cy + size * .03, size * .80))
    h = size * math.sqrt(3) / 2
    bw = max(3, size * .09)
    top, bot = cy - h * .30, cy + h * .08
    rrect(S, mark, pygame.Rect(cx - bw / 2, top, bw, bot - top), int(bw / 2))
    aacircle(S, mark, (cx, cy + h * .19), bw * .62)


def arc_pts(cx, cy, r, a0, a1, n=40):
    return [(cx + r * math.cos(a), cy - r * math.sin(a)) for a in np.linspace(a0, a1, n)]


# ---------------------------------------------------------------- HUD
class Hud:
    def __init__(self):
        self.surf = pygame.Surface((W, H), pygame.SRCALPHA)
        self.f = {}
        for s in (11, 12, 13, 14, 16, 18, 20, 24, 30, 36):
            self.f[s] = (pygame.font.Font(FONT_LATIN, s - 1), pygame.font.Font(FONT, s))
            self.f[('b', s)] = (pygame.font.Font(FONT_LATIN_BOLD, s - 1), pygame.font.Font(FONT_BOLD, s))
        self.cog_trail = collections.deque(maxlen=30)
        self.g_trail = collections.deque(maxlen=30)
        self.load_hist = collections.deque(maxlen=120)
        self.run = None
        self.seen = {}                                   # 종류 -> 처음 뜬 시각
        self.big_until = 0.0                             # 큰 표지판을 보여줄 끝 시각
        self.big_start = -1e9; self.big_severe = False
        self.tex = glGenTextures(1)

    # -- 글자 (한글 = 나눔, 라틴 = DejaVu)
    def render_mixed(self, s, size, color, bold=False):
        lat, han = self.f[('b', size) if bold else size]
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

    def text(self, s, size, pos, color=FG, anchor="topleft", bold=False):
        img = self.render_mixed(s, size, color, bold)
        r = img.get_rect(**{anchor: pos})
        self.surf.blit(img, r)
        return r

    def chip(self, s, pos, fg, bg, size=11, anchor="topleft", bold=True):
        img = self.render_mixed(s, size, fg, bold)
        r = img.get_rect(); r.inflate_ip(14, 6)
        setattr(r, anchor, pos)
        rrect(self.surf, bg, r, r.height // 2)
        self.surf.blit(img, img.get_rect(center=r.center))
        return r

    def card(self, rect, title=None, right=None):
        rrect(self.surf, CARD, rect, 12)
        rrect(self.surf, EDGE, rect, 12, 1)
        if title:
            self.text(title, 12, (rect[0] + 12, rect[1] + 8), DIM, bold=True)
        if right:
            self.text(right, 11, (rect[0] + rect[2] - 12, rect[1] + 9), DIM, "topright")

    # -- 전체
    def render(self, frame, rx, alerts, link, mode):
        S = self.surf
        S.fill((0, 0, 0, 0))
        now = time.time()
        if frame is None:
            S.fill(BG + (255,))
            aacircle(S, ACC + (60,), (W // 2, H // 2 - 30), 26 + 6 * math.sin(now * 3))
            aacircle(S, ACC, (W // 2, H // 2 - 30), 10)
            self.text("센서 신호를 기다리는 중", 20, (W // 2, H // 2 + 14), FG, "midtop", bold=True)
            self.text("TCP 5010 · 센서 파이 실시간 노드 또는 녹화 재생", 12, (W // 2, H // 2 + 44), DIM, "midtop")
            self.upload(); return
        hdr = frame["hdr"]; s = hdr.get("sensors") or {}
        if hdr["run"] != self.run:
            self.run = hdr["run"]; self.load_hist.clear(); self.cog_trail.clear(); self.g_trail.clear()
            self.big_start, self.big_until = -1e9, 0.0
        active = alerts if isinstance(alerts, dict) else {}
        self.update_sign(active, now)
        self.view(frame, hdr, active, link, mode, now)
        self.panel(hdr, s)
        self.upload()

    def update_sign(self, active, now):
        for k in [k for k in self.seen if k not in active]:
            del self.seen[k]
        for k, v in active.items():
            if k not in self.seen:                       # 새로 뜬 경고/주의 -> 3 s 동안 크게
                self.seen[k] = now
                escalate = v[2] and not self.big_severe and now < self.big_until
                if now - self.big_start > 6.0 or escalate:   # 연달아 뜨면 6 s 에 한 번만 (주의 -> 경고 승격은 바로)
                    self.big_start, self.big_until, self.big_severe = now, now + 3.0, v[2]

    # -- 왼쪽 3D 위
    def view(self, frame, hdr, active, link, mode, now):
        S = self.surf
        sev = [v for v in active.values() if v[2]]; cau = [v for v in active.values() if not v[2]]
        if sev or cau:                                   # 테두리: 경고 = 빨강 점멸, 주의 = 주황
            on = (int(now * 4) % 2 == 0) if sev else True
            if on:
                col = (BAD if sev else WARN) + (230,)
                for i, a in enumerate((230, 120, 50)):
                    rrect(S, col[:3] + (a,), pygame.Rect(i * 3, i * 3, VIEW_W - i * 6, H - i * 6), 14, 3)
        # 상단 바
        rrect(S, (9, 13, 20, 225), pygame.Rect(8, 8, VIEW_W - 16, 40), 12)
        aacircle(S, ACC, (26, 28), 6)
        tr = self.text("CARGO TWIN", 16, (38, 18), FG, bold=True)
        sc = {"cruise": OK, "brake": WARN, "turn": ACC, "drop": BAD}.get(hdr["run"], ACC)
        r = self.chip(hdr["scenario"], (tr.right + 10, 17), (12, 16, 22), sc, 12)
        dot = (int(now * 2) % 2 == 0)
        self.chip(("● " if dot else "○ ") + "녹화 재생", (r.right + 8, 17), FG, (60, 30, 34), 11, bold=False)
        self.text("%.1f / %.1f s" % (max(0, hdr["t_run"]), hdr["dur_run"]), 11, (VIEW_W - 22, 20), DIM, "topright")
        frac = max(0, min(1, hdr["t_run"] / max(hdr["dur_run"], .1)))
        rrect(S, (40, 50, 66), pygame.Rect(20, 42, VIEW_W - 40, 3), 2)
        rrect(S, sc, pygame.Rect(20, 42, int((VIEW_W - 40) * frac), 3), 2)
        # 경고 알림 (작은 줄) — 큰 표지판이 떠 있는 동안에는 그 아래로
        y = 56
        for text, _, severe in sorted(active.values(), key=lambda v: not v[2])[:3]:
            col = BAD if severe else WARN
            rr = pygame.Rect(14, y, VIEW_W - 28, 26)
            rrect(S, (18, 12, 12, 225) if severe else (24, 19, 8, 225), rr, 8)
            rrect(S, col, pygame.Rect(rr.x, rr.y, 5, rr.h), 3)
            sign(S, rr.x + 20, rr.y + 14, 17, severe)
            self.text(("경고  " if severe else "주의  ") + text, 13, (rr.x + 36, rr.y + 5), col if severe else WARN, bold=True)
            y += 30
        # 표지판: 새 경고/주의가 뜨면 3 s 동안 가운데 크게, 그 뒤에는 경고가 이어지는 동안 오른쪽에 작게. 둘 다 2.5 Hz 로 깜빡인다
        blink = int(now * 5) % 2 == 0
        top_sev = next((v for v in active.values() if v[2]), None)
        top_cau = next((v for v in active.values() if not v[2]), None)
        if active and now < self.big_until:
            main = top_sev or top_cau
            severe = main[2]
            col = BAD if severe else WARN
            age = 3.0 - (self.big_until - now)
            k = min(1.0, age / .18)
            cx, cy = VIEW_W // 2, 214
            pnl = pygame.Rect(0, 0, 340, 200); pnl.center = (cx, cy + 6)
            rrect(S, (8, 10, 14, 150), pnl, 18)
            rrect(S, col if blink else mix(col, (8, 10, 14), .35), pnl, 18, 3)
            if blink:
                sign(S, cx, cy - 24, 108 * (.75 + .25 * k), severe)
            self.text("경 고" if severe else "주 의", 30, (cx, cy + 34), col, "midtop", bold=True)
            self.text(main[0], 13, (cx, cy + 72), FG, "midtop")
            if top_sev and top_cau:                      # 둘 다 있으면 다른 하나도 작게
                other = top_cau if severe else top_sev
                if blink:
                    sign(S, pnl.right - 30, pnl.y + 32, 34, other[2])
        elif active:
            x, y = VIEW_W - 56, 196
            for v in (top_sev, top_cau):
                if v is None:
                    continue
                col = BAD if v[2] else WARN
                box = pygame.Rect(x - 44, y - 46, 88, 104)
                rrect(S, (8, 10, 14, 205), box, 14)
                rrect(S, col if blink else mix(col, (8, 10, 14), .35), box, 14, 2)
                if blink:
                    sign(S, x, y - 6, 62, v[2])
                self.text("경고" if v[2] else "주의", 16, (x, y + 30), col, "midtop", bold=True)
                y += 114
        # 아래: 모드 · 시점 버튼 + 상태
        ti = frame.get('twin_info') or {}
        info = frame.get('model_info') or {}
        labels = ('2.5D 안정 트윈', '2.5D 실시간 원본', '2.5D 높이색') if frame.get('mesh_kind') == 'height' else \
                 ('3D 안정 표면', '실시간 원본', '높이색') if frame.get('mesh_kind') == 'surface' else ('입체 모형', '높이 분석', '모형 외곽선')
        for x, w, label, icon in ((12, 168, labels[mode], "◆"), (190, 118, "시점 전환", "↻")):
            rr = pygame.Rect(x, 399, w, 32)
            rrect(S, (20, 34, 48, 235), rr, 16)
            rrect(S, (58, 98, 124), rr, 16, 1)
            self.text(icon + "  " + label, 13, rr.center, ACC, "center", bold=True)
        self.text('드래그로 회전', 11, (VIEW_W - 14, 410), DIM, "topright")
        # 블록 수 + 색 범례 (한 알약 안에)
        pill = pygame.Rect(12, 366, 300, 24)
        rrect(S, (9, 13, 20, 225), pill, 12)
        lx = pill.x + 12
        if frame.get('mesh_kind') == 'box':
            r = self.text("%d 블록 · 형태 고정" % info.get('count', 0), 11, (lx, pill.y + 5), FG, bold=True)
            lx = r.right + 14
        aacircle(S, (255, 70, 60), (lx + 5, pill.centery), 5)
        r = self.text("지속 변화", 11, (lx + 14, pill.y + 5), DIM)
        aacircle(S, (255, 170, 40), (r.right + 15, pill.centery), 5)
        r = self.text("기준 반영", 11, (r.right + 24, pill.y + 5), DIM)
        pill.width = r.right + 12 - pill.x
        # 상태 줄
        rrect(S, (9, 13, 20, 225), pygame.Rect(8, H - 42, VIEW_W - 16, 36), 10)
        if ti.get('shaking'):
            msg, col = "거치대 흔들림 감지 · 판정 멈추고 직전 형상 유지 (화면 %.0f%% 동시 변화)" % ti.get('inst_pct', 0), WARN
        elif ti.get('state') == 'ready':
            c = ti.get('changed_pct', 0)
            msg, col = "기준 형상 고정 · 지속 변화 %.1f%% · 확인 중 %.1f%%" % (c, ti.get('pending_pct', 0)), (WARN if c > 1 else FG)
        else:
            msg, col = "기준 형상 만드는 중 %d/10 프레임" % ti.get('built', 0), WARN
        self.text(msg, 11, (18, H - 39), col)
        hz, lost, lat, fps, dec = link
        good = hz >= .9 * hdr.get("hz", 10)
        aacircle(S, OK if good else WARN, (22, H - 15), 4)
        self.text("수신 %.1f Hz  ·  손실 %d  ·  지연 %s  ·  화면 %.0f fps" % (
            hz, lost, ("%.0f ms" % lat) if lat is not None else "-", fps), 11, (32, H - 22), OK if good else WARN)

    # -- 오른쪽 패널
    def panel(self, hdr, s):
        S = self.surf
        px = VIEW_W
        pygame.draw.rect(S, BG + (255,), (px, 0, W - px, H))
        x0, cw = px + 8, W - px - 16
        tot = s.get("total_g")
        age = hdr.get("sensor_age")
        # 1) 총 하중 + 추이
        c1 = pygame.Rect(x0, 8, cw, 84)
        self.card(c1, "총 하중", "센서 %s" % ("%.0f ms 전" % (age * 1000) if age is not None else "없음"))
        if tot is not None:
            self.load_hist.append(tot)
            r = self.text("%.2f" % (tot / 1000), 30, (c1.x + 12, c1.y + 30), FG, bold=True)
            self.text("kg", 14, (r.right + 5, r.bottom - 20), DIM)
        else:
            self.text("-", 30, (c1.x + 12, c1.y + 30), DIM, bold=True)
        if len(self.load_hist) > 2:                      # 추이 (최근 12초)
            gx, gy, gw, gh = c1.x + 150, c1.y + 30, cw - 162, 44
            v = np.array(self.load_hist, float)
            lo, hi = v.min(), v.max()
            span = max(hi - lo, 500)
            pts = [(gx + gw * i / (len(v) - 1), gy + gh - gh * (x - lo) / span) for i, x in enumerate(v)]
            poly = pts + [(gx + gw, gy + gh), (gx, gy + gh)]
            aapoly(S, mix(ACC, CARD, .18), poly)
            pygame.draw.aalines(S, ACC, False, pts)
            aacircle(S, ACC, pts[-1], 3)
        # 2) 적재함 평면도
        c2 = pygame.Rect(x0, 98, cw, 170)
        self.card(c2, "적재함 하중 분포")
        bx, by, bw, bh = c2.x + 52, c2.y + 40, cw - 104, 92
        cab = pygame.Rect(bx + bw * .25, by - 12, bw * .5, 9)  # 운전석 쪽 표시
        rrect(S, (46, 60, 82), cab, 4)
        self.text("앞", 11, (cab.centerx, cab.y - 1), DIM, "midbottom")
        rrect(S, (30, 38, 54), pygame.Rect(bx, by, bw, bh), 10)
        rrect(S, (66, 82, 108), pygame.Rect(bx, by, bw, bh), 10, 1)
        for i in (1, 2, 3):
            pygame.draw.line(S, (40, 50, 68), (bx + bw * i / 4, by + 4), (bx + bw * i / 4, by + bh - 4))
            pygame.draw.line(S, (40, 50, 68), (bx + 4, by + bh * i / 4), (bx + bw - 4, by + bh * i / 4))
        loads = [s.get(k) for k in ("NW_g", "NE_g", "SW_g", "SE_g")]
        mx = max([v for v in loads if v is not None] + [1])
        corners = ((bx, by, -1, -1), (bx + bw, by, 1, -1), (bx, by + bh, -1, 1), (bx + bw, by + bh, 1, 1))
        for (lx, ly, sx, sy), v in zip(corners, loads):
            if v is None:
                aacircle(S, FAINT, (lx, ly), 7, fill=False); continue
            share = v / tot if tot else .25
            col = OK if share < .35 else (WARN if share < .45 else BAD)
            aacircle(S, mix(col, (30, 38, 54), .28), (lx, ly), 9 + 11 * max(0, v) / mx)
            aacircle(S, col, (lx, ly), 6)
            tx = lx + sx * 26
            self.text("%.1f" % (v / 1000), 13, (tx, ly - 8), col, "midtop", bold=True)
            self.text("%d%%" % round(share * 100), 11, (tx, ly + 7), DIM, "midtop")
        cx, cy = s.get("cog_x_pct"), s.get("cog_y_pct")
        mid = (bx + bw / 2, by + bh / 2)
        pygame.draw.line(S, (70, 86, 110), (mid[0] - 6, mid[1]), (mid[0] + 6, mid[1]))
        pygame.draw.line(S, (70, 86, 110), (mid[0], mid[1] - 6), (mid[0], mid[1] + 6))
        if cx is not None and cy is not None:
            p = (bx + bw * cx / 100, by + bh * cy / 100)
            self.cog_trail.append(p)
            n = len(self.cog_trail)
            for i, q in enumerate(self.cog_trail):
                aacircle(S, mix(GOLD, (30, 38, 54), .2 + .7 * i / n), q, 2)
            aacircle(S, mix(GOLD, (30, 38, 54), .3), p, 11)
            aacircle(S, GOLD, p, 6)
            aacircle(S, (30, 24, 8), p, 2)
            off = math.hypot(cx - 50, cy - 50)
            self.text("무게중심  좌우 %.0f%% · 앞뒤 %.0f%%" % (cx, cy), 11, (c2.centerx, c2.bottom - 17), GOLD, "midtop")
            self.chip("치우침 %.0f%%p" % off, (c2.right - 10, c2.y + 6), (12, 16, 22),
                      OK if off < 12 else (WARN if off < 22 else BAD), 11, "topright")
        # 3) 벨트 장력 게이지
        c3 = pygame.Rect(x0, 274, cw, 82)
        self.card(c3, "벨트 장력")
        t1 = s.get("c1_N")
        stuck = "stuck" in s.get("flags", "")
        gx, gy, gr = c3.x + 64, c3.y + 72, 44
        for a0, a1, col in ((math.pi, math.pi * .62, (64, 80, 104)), (math.pi * .62, math.pi * .28, ACC), (math.pi * .28, 0, WARN)):
            pts = arc_pts(gx, gy, gr, a0, a1) + arc_pts(gx, gy, gr - 7, a1, a0)
            aapoly(S, mix(col, CARD, .65), pts)
        if t1 is not None:
            a = math.pi * (1 - max(0, min(1, t1 / 15)))
            tip = (gx + (gr - 2) * math.cos(a), gy - (gr - 2) * math.sin(a))
            pygame.draw.line(S, FG, (gx, gy), tip, 3)
            aacircle(S, FG, (gx, gy), 4)
        self.text("%.1f" % t1 if t1 is not None else "-", 24, (c3.x + 128, c3.y + 30), WARN if stuck else FG, bold=True)
        self.text("N  (0~15)", 11, (c3.x + 130, c3.y + 58), DIM)
        if stuck:
            self.chip("센서 값 고정", (c3.right - 10, c3.y + 34), (12, 16, 22), WARN, 11, "topright")
        # 4) 자세: 수평계 + G 볼
        c4 = pygame.Rect(x0, 362, cw, 110)
        self.card(c4, "차량 자세")
        roll, pitch, gz = s.get("imu1_roll_deg"), s.get("imu1_pitch_deg"), s.get("imu1_gz_dps")
        hx, hy, hr = c4.x + 50, c4.y + 64, 34
        horizon = pygame.Surface((hr * 2 + 2, hr * 2 + 2), pygame.SRCALPHA)
        aacircle(horizon, (40, 90, 150), (hr, hr), hr)
        if roll is not None and pitch is not None:       # 인공 수평계: 롤만큼 기울고 피치만큼 오르내림
            ra = math.radians(roll); off = max(-hr, min(hr, pitch * 2.2))
            d = (math.sin(ra), math.cos(ra))
            c = (hr, hr + off)
            big = hr * 3
            ground = [(c[0] - big * math.cos(ra), c[1] - big * math.sin(ra)), (c[0] + big * math.cos(ra), c[1] + big * math.sin(ra)),
                      (c[0] + big * math.cos(ra) - big * math.sin(ra), c[1] + big * math.sin(ra) + big * math.cos(ra)),
                      (c[0] - big * math.cos(ra) - big * math.sin(ra), c[1] - big * math.sin(ra) + big * math.cos(ra))]
            aapoly(horizon, (128, 92, 52), ground)
            mask = pygame.Surface(horizon.get_size(), pygame.SRCALPHA)
            aacircle(mask, (255, 255, 255, 255), (hr, hr), hr)
            horizon.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_MIN)
        S.blit(horizon, (hx - hr, hy - hr))
        gfx.aacircle(S, hx, hy, hr, (90, 106, 130))
        pygame.draw.line(S, GOLD, (hx - 16, hy), (hx - 5, hy), 2); pygame.draw.line(S, GOLD, (hx + 5, hy), (hx + 16, hy), 2)
        aacircle(S, GOLD, (hx, hy), 2)
        tx = c4.x + 96
        self.text("롤", 11, (tx, c4.y + 30), DIM); self.text("%+.1f°" % roll if roll is not None else "-", 14, (tx + 34, c4.y + 28), FG, bold=True)
        self.text("피치", 11, (tx, c4.y + 52), DIM); self.text("%+.1f°" % pitch if pitch is not None else "-", 14, (tx + 34, c4.y + 50), FG, bold=True)
        self.text("요", 11, (tx, c4.y + 74), DIM); self.text("%+.0f°/s" % gz if gz is not None else "-", 14, (tx + 34, c4.y + 72), FG, bold=True)
        gx0, gy0, gr = c4.right - 46, c4.y + 62, 34          # G 볼
        aacircle(S, (30, 38, 54), (gx0, gy0), gr)
        gfx.aacircle(S, gx0, gy0, gr, (70, 86, 110)); gfx.aacircle(S, gx0, gy0, gr // 2, (50, 62, 82))
        pygame.draw.line(S, (50, 62, 82), (gx0 - gr, gy0), (gx0 + gr, gy0)); pygame.draw.line(S, (50, 62, 82), (gx0, gy0 - gr), (gx0, gy0 + gr))
        ax, ay = s.get("imu1_ax_G"), s.get("imu1_ay_G")
        if ax is not None and ay is not None:
            q = (gx0 + gr * max(-1, min(1, ay / .5)), gy0 - gr * max(-1, min(1, ax / .5)))
            self.g_trail.append(q)
            n = len(self.g_trail)
            for i, t in enumerate(self.g_trail):
                aacircle(S, mix(ACC, (30, 38, 54), .15 + .6 * i / n), t, 2)
            hot = abs(ax) > .25
            aacircle(S, mix(BAD if hot else ACC, (30, 38, 54), .3), q, 9)
            aacircle(S, BAD if hot else ACC, q, 5)
        self.text("G ±0.5", 11, (gx0, c4.y + 8), DIM, "midtop")

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
