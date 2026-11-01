#!/usr/bin/env python3
"""디스플레이 파이: 화물 적재 디지털 트윈 웹 백엔드 (http://localhost:8090). 센서 파이 실시간 입력 전용.

  입력: TCP 5010 (twin_view.Receiver) -- 센서 파이가 보내는 실시간 프레임 (형식: ../README.md). 녹화 재생은 받지 않는다.
  처리: twin_view.StableBoxMesh (2.5D 상자 모형) + stack_layers.StackLayers (쌓인 상자 층 · 옆 짐)
        + 센서 파이 위험 판정(sensors 의 risk · lv_* · risk_text) + hud_v3.Voice (경보음 · 음성, USB 스피커).
  일시정지: 화면만 멈춘다 (받은 프레임을 버린다). 보내는 쪽 UDP 5011 로 'pause' / 'resume' 도 보낸다 (듣고 있으면).
  적재 · 주행 모드: 보내는 쪽 UDP 5023 으로 'mode load' / 'mode drive' (센서 파이 sensor_feed.py 가 판정기에 반영).
        지금 모드는 sensors 의 mode 로 돌아온다. 화면은 두 모드가 같고, 적재 모드에서는 경고음 · 주의음만 안 낸다.
        sensors 에 mode 가 없는 옛 송신기 · 녹화 재생이면 이 서버가 모드를 들고 소리만 바꾼다 (처음은 주행 = 소리 냄).

HTTP (계약: web/src/lib/twin-types.ts, twin-api.ts)
  GET  /                 ../www (빌드된 화면, SPA, 없는 경로는 index.html). 동영상 등은 Range(206) 지원
  GET  /api/stream       SSE, TwinState JSON 약 10 Hz, 5 s 마다 ': keepalive'
  GET  /api/state        TwinState JSON 한 번
  GET  /api/camera.jpg   최근 카메라 사진 (JPEG q80, 프레임마다 한 번만 만든다). 없으면 204
  POST /api/control      {cmd: pause|resume|toggle|rebase}. rebase = 기준 재설정 (실시간일 때만, UDP 5023 'rebase')
  POST /api/mode         {mode: load|drive} -> {ok, mode, sent, by}. 적용은 다음 프레임의 TwinState.mode 로 확인
  GET|POST /api/volume   {action: up|down|mute|unmute|toggle|set, value 0..1.5} -> {level, muted} (wpctl)
  POST /api/exit         키오스크 크로미움을 닫는다 (먼저 200 응답)
  POST /api/stats        화면 쪽 진단 값을 로그에 한 줄

usage: python3 twin_server.py [--http 8090] [--tcp 5010]
로그: ../logs/server.log
"""
import argparse, collections, email.utils, functools, io, json, logging, logging.handlers, math, os, re, signal, socket, \
    struct, subprocess, sys, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlsplit

os.environ.setdefault('PYGAME_HIDE_SUPPORT_PROMPT', '1')
os.environ.setdefault('XDG_RUNTIME_DIR', '/run/user/1000')          # wpctl · PipeWire (SDL 소리)
HOME = os.path.expanduser('~')
APP_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(APP_DIR)                                      # display/twin
VIEW_DIR = os.environ.get('TWIN_VIEW_DIR', APP_DIR)                  # twin_view 등 처리 모듈은 이 폴더에 같이 있다
DIST = os.environ.get('TWIN_WEB_DIST', os.path.join(ROOT, 'www'))
LOG_DIR = os.path.join(ROOT, 'logs')
RUN_DIR = os.path.join(ROOT, 'run')
KIOSK_PROFILE = os.path.join(HOME, '.cache', 'cargo-twin-kiosk')    # launch.sh 의 --user-data-dir 와 같아야 한다
CTRL_PORT = 5011
MODE_PORT = 5023                                                     # 센서 파이 sensor_feed.py 적재 · 주행 모드
sys.path.insert(0, VIEW_DIR)

import numpy as np                                                    # noqa: E402
import pygame                                                         # noqa: E402
from PIL import Image                                                 # noqa: E402
import twin_view as tv                                                # noqa: E402
import hud_v3                                                         # noqa: E402
import scene_v3                                                       # noqa: E402
import scene_v2                                                       # noqa: E402
from stack_layers import StackLayers                                  # noqa: E402
try:
    from stable_twin import BUILD_N                                   # noqa: E402
except ImportError:
    BUILD_N = 10

# 화면 없이 돌리므로 GL 용 꾸민 도형은 만들지 않는다 (ModelMesh.build 가 매번 부르는 것, CPU 절약)
scene_v2.styled = lambda *a, **k: []

log = logging.getLogger('twin')


def setup_log():
    os.makedirs(LOG_DIR, exist_ok=True)
    h = logging.handlers.RotatingFileHandler(os.path.join(LOG_DIR, 'server.log'), maxBytes=2 << 20, backupCount=2,
                                             encoding='utf-8')
    h.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s'))
    log.addHandler(h)
    s = logging.StreamHandler(sys.stdout)
    s.setFormatter(logging.Formatter('%(asctime)s %(message)s', '%H:%M:%S'))
    log.addHandler(s)
    log.setLevel(logging.INFO)


# ---------------------------------------------------------------- JSON 도우미
def clean(v):
    """numpy -> 파이썬, NaN/inf -> None (JSON 에 NaN 을 넣지 않는다)."""
    if isinstance(v, dict):
        return {str(k): clean(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [clean(x) for x in v]
    if isinstance(v, np.ndarray):
        return [clean(x) for x in v.tolist()]
    if isinstance(v, (bool, np.bool_)):
        return bool(v)
    if isinstance(v, (int, np.integer)):
        return int(v)
    if isinstance(v, (float, np.floating)):
        f = float(v)
        return f if math.isfinite(f) else None
    return v


def num(v, nd=None):
    """숫자 하나 (없거나 NaN 이면 None)."""
    if v is None or isinstance(v, (str, bytes)):
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(f):
        return None
    return round(f, nd) if nd is not None else f


def dumps(obj):
    return json.dumps(clean(obj), ensure_ascii=False, separators=(',', ':'), allow_nan=False).encode()


def box_json(b):
    c = np.asarray(b.get('color', (0.5, 0.5, 0.5)), float).reshape(-1)[:3]
    bid = b.get('id')
    return dict(id=int(bid) if bid is not None else None,
                shape='cylinder' if b.get('shape') == 'cylinder' else 'box',
                x0=num(b['x0'], 4), x1=num(b['x1'], 4), y0=num(b['y0'], 4), y1=num(b['y1'], 4),
                bottom=num(b['bottom'], 4), top=num(b['top'], 4),
                color=[num(min(max(x, 0.0), 1.0), 3) for x in c],
                held=bool(b.get('held', False)), alert=bool(scene_v3.is_alert(b.get('src', b))) if b.get('kind') != 'side' and ('src' in b or 'kind' not in b) else False,
                kind=b.get('kind') if b.get('kind') in ('side', 'item', 'bottle') else 'box')


def sensors_json(s, age):
    """원본 이름(SW_g ... imu1_*) -> TwinSensors. 센서가 없는 프레임(None)이면 전부 None."""
    s = s or {}
    cx, cy = num(s.get('cog_x_pct'), 2), num(s.get('cog_y_pct'), 2)
    flags = s.get('flags') or ''
    return dict(total_g=num(s.get('total_g'), 1),
                loads=dict(NW=num(s.get('NW_g'), 1), NE=num(s.get('NE_g'), 1),
                           SW=num(s.get('SW_g'), 1), SE=num(s.get('SE_g'), 1)),
                cog=dict(x_pct=cx, y_pct=cy) if cx is not None and cy is not None else None,
                tension_n=num(s.get('c1_N'), 3),
                tension_stuck='stuck' in str(flags),
                roll=num(s.get('imu1_roll_deg'), 2), pitch=num(s.get('imu1_pitch_deg'), 2),
                yaw_rate=num(s.get('imu1_gz_dps'), 2),
                ax=num(s.get('imu1_ax_G'), 4), ay=num(s.get('imu1_ay_G'), 4),
                age_s=num(age, 3),
                # 2026-10-04 부터 (센서 파이 analysis/): 뒤 벨트 c2, 벨트별 필요 장력, mm 무게중심, 적재 kg
                load_kg=num(s.get('load_kg'), 3) if s.get('load_kg') is not None else num((s.get('total_g') or 0) / 1000, 3) if s.get('total_g') is not None else None,
                max_kg=num(s.get('rated_kg')) or MAX_KG,
                # 2026-10-08 판정 (팀 analysis/total.py): 적정 적재중량과 주의 · 경고 · 위험 % , 셀 합 그대로의 무게
                rated_kg=num(s.get('rated_kg')),
                load_pct=s.get('load_pct'),
                measured_kg=num(s.get('measured_kg'), 3),
                tension_max_n=num(s.get('tension_max_N')),
                bed_mm=dict(l=BED_L_MM, w=BED_W_MM),
                cog_mm=dict(x=num(s.get('cog_x_mm'), 1), y=num(s.get('cog_y_mm'), 1)) if s.get('cog_x_mm') is not None else None,
                cog_static_mm=dict(x=num(s.get('cog_static_x_mm'), 1), y=num(s.get('cog_static_y_mm'), 1)) if s.get('cog_static_x_mm') is not None else None,
                cog_pred_mm=dict(x=num(s.get('cog_pred_x_mm'), 1), y=num(s.get('cog_pred_y_mm'), 1)) if s.get('cog_pred_x_mm') is not None else None,
                belts=[dict(key=k, name=nm, n=num(s.get(k + '_N'), 2), min_n=num(s.get(k + '_min_N'), 2),
                            level=level4(s, s.get('lv_' + k)))
                       for k, nm in (('c1', '앞 벨트'), ('c2', '뒤 벨트')) if k == 'c1' or s.get(k + '_N') is not None])


MAX_KG = 15.0                    # 센서 파이 LOAD_MAX_KG
BED_L_MM, BED_W_MM = 250.0, 392.0  # 센서 파이 acquisition/env.sh: 앞왼-앞오 / 앞왼-뒤왼 셀 간격
CHECKS = (('load', '적재'), ('cog', '무게중심'), ('move', '화물 이동'), ('c1', '앞 벨트'), ('c2', '뒤 벨트'))
TEXT_KEY = {'적재': 'load', '무게중심': 'cog', '화물 이동': 'move', '장력 c1': 'c1', '장력 c2': 'c2',
            '장력 N(c1)': 'c1', '장력 S(c2)': 'c2'}


def level4(s, v):
    """판정 단계 -> 4단계 (0 정상 1 주의 2 경고 3 위험). 2026-10-08 판정(algo 있음)은 그대로,
    옛 3단계 녹화(0 정상 1 주의 2 위험)는 위험을 3 으로 옮긴다."""
    lv = lvl(v)
    if lv is None or lv < 2 or is_4level(s):
        return lv
    return 3


def is_4level(s):
    """2026-10-08 부터의 4단계 판정인가. 다시 계산한 녹화(algo) · 새 파이프라인 열(measured_kg) · 3(위험) 값 ·
    '경고' 문구 중 하나라도 있으면 4단계. 옛 3단계는 2 가 곧 위험이고 '경고' 라는 말이 없다."""
    s = s or {}
    if s.get('algo') or 'measured_kg' in s:
        return True
    if any(lvl(s.get(k)) == 3 for k in ('risk', 'lv_load', 'lv_cog', 'lv_move', 'lv_c1', 'lv_c2')):
        return True
    return '경고' in str(s.get('risk_text') or '')


def lvl(v):
    try:
        return int(round(float(v)))
    except (TypeError, ValueError):
        return None


def risk_json(s):
    """센서 파이 위험 판정(risk, lv_*, risk_text) -> {level, text, checks[]}. 판정이 없는 옛 녹화면 None."""
    s = s or {}
    if s.get('risk') is None:
        return None
    reasons = dict(s.get('check_text') or {})
    for part in ([] if reasons else (s.get('risk_text') or '').split(' | ')):
        part = part.strip()
        head, _, why = part.partition(': ')
        for name, key in TEXT_KEY.items():
            if head.endswith(name):
                reasons[key] = why
    checks = []
    for key, name in CHECKS:
        lv = level4(s, s.get('lv_' + key))
        if lv is None and key == 'c2' and s.get('c2_N') is None:
            continue
        checks.append(dict(key=key, name=name, level=lv, reason=reasons.get(key, '') if lv else ''))
    return dict(level=level4(s, s.get('risk')), overall=s.get('risk_overall'), text=s.get('risk_text') or '', checks=checks)


COG_SIDE = (('앞쪽', 'front'), ('뒤쪽', 'back'), ('왼쪽', 'left'), ('오른쪽', 'right'))
BELT_SIDE = {'c1': 'front', 'c2': 'back'}


def reason_clip(key, reason):
    """판정 key + 이유 문구(팀 analysis 의 Check.text) -> 이유 음성 이름 (server/sounds/reason/<이름>.mp3). 모르면 None.
    무게중심은 '왼쪽 쏠림 …' 의 방향, 장력은 '부족' · '풀림' (약함) / '과조임' · '상한 넘음' (셈) 으로 고른다."""
    reason = reason or ''
    if key in ('load', 'move'):
        return key
    if key == 'cog':
        return next(('cog_' + side for word, side in COG_SIDE if word in reason), None)
    if key in BELT_SIDE:
        if '부족' in reason or '풀림' in reason:
            return 'belt_%s_low' % BELT_SIDE[key]
        if '과조임' in reason or '상한' in reason:
            return 'belt_%s_high' % BELT_SIDE[key]
    return None


EMPTY_TWIN = dict(state='building', built=0, changed_pct=0.0, pending_pct=0.0, shaking=False)


# ---------------------------------------------------------------- 소리
class VoiceFeed:
    """디스플레이 스피커로 낸 음성 기록. 보기 전용 브라우저가 /api/voice 로 기다렸다가 같은 소리를 낸다."""

    OLD_KIND = {'warning': '2', 'caution': '1'}     # 예전 화면이 부르던 이름

    def __init__(self):
        self.seq, self.kind, self.cv = 0, None, threading.Condition()
        self.wav = {}

    def push(self, kind):
        """kind = Voice.sound_id ('<단계>' 또는 '<단계>_<이유 음성 이름>', 예 '3_belt_back_low')"""
        with self.cv:
            self.seq += 1
            self.kind = kind
            self.cv.notify_all()

    def wait(self, after, timeout=20.0):
        with self.cv:
            if after >= 0:
                self.cv.wait_for(lambda: self.seq > after, timeout)
            return dict(seq=self.seq, kind=self.kind, severe=bool(self.kind) and self.kind[0] in '23')

    @classmethod
    def parse(cls, voice, kind):
        """'<단계>[_<이유>]' -> (단계, 이유 음성 이름 | None). 모르는 이름이면 None."""
        m = re.fullmatch(r'([123])(?:_([a-z_]+))?', cls.OLD_KIND.get(kind, kind or ''))
        if not m or (m.group(2) and m.group(2) not in voice.reason):
            return None
        return int(m.group(1)), m.group(2)

    def wav_bytes(self, voice, kind):
        """Voice 가 만든 소리(신호음 + 단계 음성 + 이유 음성)를 그대로 WAV 로 (처음 한 번 만들고 둔다)"""
        if kind not in self.wav:
            import wave
            arr = pygame.sndarray.array(voice.sound(*self.parse(voice, kind)))
            sr, _, nch = pygame.mixer.get_init()
            buf = io.BytesIO()
            with wave.open(buf, 'wb') as w:
                w.setnchannels(nch if arr.ndim == 2 else 1)
                w.setsampwidth(2)
                w.setframerate(sr)
                w.writeframes(np.ascontiguousarray(arr.astype('<i2')).tobytes())
            self.wav[kind] = buf.getvalue()
        return self.wav[kind]


VOICE = VoiceFeed()
VOICE_JS = '''<script>
/* 보기 전용 화면: 디스플레이 스피커가 위험/경고/주의 음성(+ 이유)을 낼 때 이 브라우저에서도 같은 소리 (화면을 한 번 누른 뒤부터) */
(function () {
  var seq = null, on = false, au = {};
  function audio(k) {
    if (!au[k]) { au[k] = new Audio('/api/voice.wav?kind=' + encodeURIComponent(k)); au[k].preload = 'auto'; }
    return au[k];
  }
  function unlock() {
    if (on) return; on = true;
    ['1', '2', '3'].forEach(audio);
  }
  addEventListener('pointerdown', unlock, true); addEventListener('keydown', unlock, true);
  function play(k) {
    Object.keys(au).forEach(function (o) { if (o !== k) { au[o].pause(); } });
    var a = audio(k); a.currentTime = 0; a.play().catch(function () {});
  }
  function loop() {
    fetch('/api/voice?after=' + (seq === null ? -1 : seq), { cache: 'no-store' })
      .then(function (r) { return r.json(); })
      .then(function (d) { if (seq !== null && d.seq > seq && on && d.kind) play(d.kind); seq = d.seq; loop(); })
      .catch(function () { setTimeout(loop, 3000); });
  }
  loop();
})();
</script>'''.encode()


class Beep:
    """볼륨을 바꾼 뒤 새 크기로 짧고 부드러운 '띡'. 경보(채널 0)가 울리는 중이면 내지 않는다."""

    def __init__(self, voice):
        self.voice, self.snd, self.ch = voice, None, None
        try:
            if not pygame.mixer.get_init():
                pygame.mixer.init()
            sr, _, nch = pygame.mixer.get_init()
            t = np.arange(int(sr * .11)) / sr
            env = np.minimum(t / .004, 1) * np.exp(-t / .035)                 # 빠른 시작 + 짧은 감쇠
            sig = (np.sin(2 * np.pi * 1180 * t) + .25 * np.sin(2 * np.pi * 2360 * t)) * env * .22
            arr = (sig * 32000).astype(np.int16)
            if nch > 1:
                arr = np.repeat(arr[:, None], nch, 1)
            self.snd = pygame.sndarray.make_sound(np.ascontiguousarray(arr))
            self.ch = pygame.mixer.Channel(1)
        except Exception as e:
            log.warning('beep off: %s', e)

    def play(self):
        if self.snd is None:
            return
        try:
            if getattr(self.voice, 'ok', False) and self.voice.ch.get_busy():
                return
            self.ch.play(self.snd)
        except Exception as e:
            log.warning('beep: %s', e)


# ---------------------------------------------------------------- 볼륨 (wpctl)
class Volume:
    SINK = '@DEFAULT_AUDIO_SINK@'
    MAX = 1.5

    def __init__(self, beep):
        self.beep = beep
        self.level, self.muted = 0.0, False
        self.lock = threading.Lock()
        self.env = dict(os.environ, XDG_RUNTIME_DIR=os.environ.get('XDG_RUNTIME_DIR', '/run/user/1000'))
        self.read()
        threading.Thread(target=self.poll, daemon=True, name='volume').start()

    def wpctl(self, *args):
        r = subprocess.run(['wpctl', *args], env=self.env, capture_output=True, text=True, timeout=3)
        if r.returncode != 0:
            raise RuntimeError('wpctl %s: %s' % (' '.join(args), (r.stderr or r.stdout).strip()))
        return r.stdout

    def read(self):
        try:
            out = self.wpctl('get-volume', self.SINK)
            m = re.search(r'Volume:\s*([\d.]+)', out)
            if m:
                with self.lock:
                    self.level = round(float(m.group(1)), 3)
                    self.muted = '[MUTED]' in out
        except Exception as e:
            log.warning('volume read: %s', e)
        return self.get()

    def get(self):
        with self.lock:
            return dict(level=self.level, muted=self.muted)

    def poll(self):
        while True:
            time.sleep(2.0)
            self.read()

    def act(self, action, value=None):
        """up/down = 5 %. up · set 은 음소거를 푼다 (눌렀는데 소리가 안 나면 고장으로 보여서)."""
        s = self.SINK
        if action == 'up':
            self.wpctl('set-volume', '-l', str(self.MAX), s, '5%+'); self.wpctl('set-mute', s, '0')
        elif action == 'down':
            self.wpctl('set-volume', '-l', str(self.MAX), s, '5%-')
        elif action == 'mute':
            self.wpctl('set-mute', s, '1')
        elif action == 'unmute':
            self.wpctl('set-mute', s, '0')
        elif action == 'toggle':
            self.wpctl('set-mute', s, 'toggle')
        elif action == 'set':
            v = num(value)
            if v is None:
                raise ValueError('value 가 필요합니다 (0..1.5)')
            v = min(max(v, 0.0), self.MAX)
            self.wpctl('set-volume', '-l', str(self.MAX), s, '%.3f' % v)
            if v > 0:
                self.wpctl('set-mute', s, '0')
        else:
            raise ValueError('알 수 없는 action: %r' % action)
        st = self.read()
        if action != 'mute' and not st['muted']:
            self.beep.play()
        log.info('volume %s %s -> %.2f%s', action, '' if value is None else value, st['level'],
                 ' muted' if st['muted'] else '')
        return st


# ---------------------------------------------------------------- 수신 (일시정지 중에도 연결 유지)
class Receiver(tv.Receiver):
    """twin_view.Receiver 와 같지만, 일시정지 중(hold)에는 조용해도 TCP 연결을 끊지 않는다.

    원래 Receiver 는 5 s 동안 아무것도 안 오면 연결을 끊는다. 보내는 쪽이 일시정지하면 보내지 않으므로
    5 s 넘게 멈췄다 재개하면 새로 접속해서 처음부터 다시 보냈다 (재개가 '처음부터'처럼 동작).
    """
    QUIET = 5.0                    # 일시정지가 아닐 때 이만큼 조용하면 끊는다 (원래와 같음)
    HOLD_MAX = 600.0               # 일시정지 중이라도 이보다 오래 조용하면 끊는다 (전원이 나간 상대)

    def __init__(self, port, transport='tcp'):
        super().__init__(port, transport)
        self.hold = False

    def read_n(self, conn, n):
        """n 바이트. 끊기면 None (이유는 self.why).
        멈춘 동안(hold)은 조용한 시간을 세지 않는다 -> 재개하면 다시 5 s 여유 (안 그러면 재개 직후 바로 끊겼다)."""
        data, quiet, held = bytearray(), 0.0, 0.0
        while len(data) < n:
            try:
                part = conn.recv(n - len(data))
            except socket.timeout:
                if self.hold:
                    quiet, held = 0.0, held + 1.0
                    if held >= self.HOLD_MAX:
                        self.why = '일시정지 중 %.0f s 동안 조용함' % held
                        return None
                else:
                    quiet += 1.0
                    if quiet >= self.QUIET:
                        self.why = '%.0f s 동안 조용함' % quiet
                        return None
                continue
            if not part:
                self.why = '상대가 연결을 닫음'
                return None
            data.extend(part)
            quiet = held = 0.0
        return bytes(data)

    def messages(self):
        if self.transport != 'tcp':
            yield from super().messages()
            return
        while True:
            conn, addr = self.sock.accept()
            self.peer = addr[0]            # 보내는 쪽 주소 (일시정지 · 처음부터 신호를 돌려보낼 곳)
            log.info('sender connected %s:%d', addr[0], addr[1])
            conn.settimeout(1.0)
            self.why, n = '?', 0
            try:
                while True:
                    prefix = self.read_n(conn, 4)
                    if prefix is None:
                        break
                    length = struct.unpack('!I', prefix)[0]
                    if not 4 <= length <= 2_000_000:
                        self.why = '잘못된 길이 %d' % length
                        break
                    blob = self.read_n(conn, length)
                    if blob is None:
                        break
                    n += 1
                    yield blob, time.time()
            except OSError as e:
                self.why = repr(e)
            finally:
                conn.close()
                log.info('sender disconnected %s after %d frames: %s', addr[0], n, self.why)


# ---------------------------------------------------------------- 처리 (twin_app.py 의 화면 없는 판)
class Registration:
    """카메라 <-> 적재판 위치 맞추기. 카메라와 판의 상대 위치는 잰 적이 없으므로, 로드셀이 잰 무게중심을 기준으로 맞춘다.
    가감속이 없을 때 '카메라로 본 화물 부피 중심' 이 '로드셀 무게중심(하중 이동을 뺀 값)' 위에 오도록 하는 이동량 (cx, cy):
        판 X = 카메라 x - cx,   판 Z(뒤 +) = -(카메라 y - cy)
    카메라는 판에 고정이라 이 값은 상수다 -> 프레임마다 낸 값의 중앙값을 쓰고 run/reg.json 에 저장해 다음 실행에도 쓴다.
    화물이 고르게 차 있다고 보는 근사라 오차는 수 cm 정도."""
    N = 900
    MOVE = .004

    def __init__(self):
        self.path = os.path.join(RUN_DIR, 'reg.json')
        self.est = collections.deque(maxlen=self.N)
        self.value = None
        self.saved_t = 0.0
        self.locked = False
        try:
            with open(self.path) as f:
                d = json.load(f)
            self.value = (float(d['cx']), float(d['cy']))
            self.locked = os.environ.get('TWIN_REG_LEARN', '') != '1'
            self.est.extend([self.value] * min(int(d.get('n', 0)), 200))
        except (OSError, ValueError, KeyError, TypeError):
            pass

    def update(self, boxes, s):
        # 이미 맞춘 값(run/reg.json)이 있으면 고정한다: 카메라 거치대는 그대로인데, 로드셀 하나가 틀리게 읽히면
        # (영점 · 부호) 무게중심이 틀려 화물이 엉뚱한 데로 밀려 그려진다. 다시 배우려면 reg.json 을 지우거나 TWIN_REG_LEARN=1
        if self.locked:
            return
        s = s or {}
        cx_mm = s.get('cog_static_x_mm', s.get('cog_x_mm'))
        cy_mm = s.get('cog_static_y_mm', s.get('cog_y_mm'))
        ax, ay, gz = s.get('imu1_ax_G'), s.get('imu1_ay_G'), s.get('imu1_gz_dps')
        quiet = all(v is not None for v in (ax, ay)) and abs(ax) < .03 and abs(ay) < .03 and abs(gz or 0) < 3
        quiet = quiet and not lvl(s.get('lv_move'))     # 화물이 밀린 중이면 무게중심과 부피 중심이 어긋난다
        solid = [b for b in boxes if b.get('kind') in ('box', 'side') and b['top'] - b['bottom'] >= .06]
        if quiet and cx_mm is not None and cy_mm is not None and solid and (s.get('total_g') or 0) > 1000:
            v = [(b['x1'] - b['x0']) * (b['y1'] - b['y0']) * (b['top'] - b['bottom']) for b in solid]
            tot = sum(v)
            xc = sum(w * (b['x0'] + b['x1']) / 2 for w, b in zip(v, solid)) / tot
            yc = sum(w * (b['y0'] + b['y1']) / 2 for w, b in zip(v, solid)) / tot
            X = -BED_L_MM / 2000 + float(cx_mm) / 1000
            Z = -BED_W_MM / 2000 + float(cy_mm) / 1000
            self.est.append((xc - X, yc + Z))
        if len(self.est) >= 20:
            m = (float(np.median([e[0] for e in self.est])), float(np.median([e[1] for e in self.est])))
            if self.value is None or max(abs(m[0] - self.value[0]), abs(m[1] - self.value[1])) > self.MOVE:
                self.value = m
            if time.time() - self.saved_t > 30:
                self.saved_t = time.time()
                try:
                    with open(self.path, 'w') as f:
                        json.dump(dict(cx=self.value[0], cy=self.value[1], n=len(self.est)), f)
                except OSError:
                    pass

    def json(self):
        return None if self.value is None else dict(cx=round(self.value[0], 4), cy=round(self.value[1], 4), n=len(self.est), locked=self.locked)


class Twin:
    def __init__(self, port):
        self.rx = Receiver(port, 'tcp')
        self.rx.start()
        self.mesh, self.alerts = tv.StableBoxMesh(), tv.Alerts()
        self.layers = StackLayers()    # 쌓인 화물을 층(상자)별로 + 옆 짐
        self.boxes, self.base = [], 0.0
        self.reg = Registration()
        self.voice = hud_v3.Voice()
        self.ctrl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.lock = threading.Lock()
        self.paused = False
        self.frame = None              # 마지막으로 반영한 프레임 (5 s 안 오면 None = 연결 끊김)
        self.session = None            # 마지막으로 반영한 프레임의 보내는 쪽 세션
        self.snap = None               # 프레임에서 나온 TwinState 부분
        self.active = {}               # Alerts.active 복사본 (다른 스레드에서 읽는다)
        self.camera_seq = 0
        self.rgb, self.jpeg = None, (None, None)
        self.jpeg_lock = threading.Lock()
        self.build_ms = collections.deque(maxlen=100)
        self.err_t = 0.0
        self.resume_t = 0.0            # 일시정지를 푼 시각 (오래 멈췄다 풀어도 바로 '연결 끊김'으로 보지 않게)
        self.mode_local = 'drive'      # 센서 파이가 모드를 안 보낼 때(옛 송신기 · 녹화 재생) 이 서버가 드는 모드
        self.mode_by = None            # 마지막 프레임의 모드를 누가 들었나: 'sensor' | 'display'
        threading.Thread(target=self.loop, daemon=True, name='twin').start()

    # 보내는 쪽(TCP 상대)의 UDP 5011 로 제어 신호
    def send(self, msg, port=CTRL_PORT):
        peer = getattr(self.rx, 'peer', None)
        if not peer:
            return False
        try:
            self.ctrl.sendto(msg.encode(), (peer, port))
            return True
        except OSError as e:
            log.warning('control send %s: %s', msg, e)
            return False

    def control(self, cmd):
        if cmd == 'rebase':
            # 기준 재설정: 화물 이동 판정의 기준을 지금 상태로 (센서 파이 sensor_feed 가 Analyzer.rebase). 화면 상태는 안 바뀐다.
            # 적재 · 주행 모드와 같은 UDP 5023 으로 보낸다. 여러 번 눌러도 1초에 한 번만 보낸다
            now = time.time()
            if now - getattr(self, 'rebase_t', 0.0) < 1.0:
                return dict(ok=True, sent=False, dup=True)
            self.rebase_t = now
            live = bool(self.snap and self.snap.get('source') == 'live')
            sent = self.send('rebase', MODE_PORT) if live else False
            log.info('control rebase (sent %s, live %s)', sent, live)
            return dict(ok=True, sent=sent, live=live)
        with self.lock:
            if cmd == 'toggle':
                cmd = 'resume' if self.paused else 'pause'
            if cmd == 'pause':
                self.paused = True
            elif cmd == 'resume':
                self.paused = False
                self.resume_t = time.time()
            else:
                raise ValueError('알 수 없는 cmd: %r' % cmd)
            paused = self.paused
            self.rx.hold = paused                         # 멈춘 동안 TCP 연결을 끊지 않는다
        sent = self.send(cmd)
        log.info('control %s (sent %s) paused=%s', cmd, sent, paused)
        return dict(ok=True, paused=paused, sent=sent)

    def set_mode(self, mode):
        """적재 · 주행 단추 -> 센서 파이 UDP 5023. 바뀐 모드는 다음 프레임 sensors.mode 로 돌아온다.
        센서 파이가 모드를 모르면(sensors 에 mode 없음) 이 서버의 모드만 바꾼다 (소리만 켜고 끈다)."""
        if mode not in ('load', 'drive'):
            raise ValueError('알 수 없는 mode: %r' % mode)
        self.mode_local = mode
        sent = self.send('mode ' + mode, MODE_PORT)
        log.info('mode %s (sent %s, by %s)', mode, sent, self.mode_by)
        return dict(ok=True, mode=mode, sent=sent, by=self.mode_by or 'display')

    def loop(self):
        while True:
            try:
                self.step()
            except Exception:
                log.exception('twin loop')
                time.sleep(.5)
            time.sleep(.005)

    def step(self):
        f, new = self.rx.take()
        now = time.time()
        if new and f is not None:
            sess = f['hdr'].get('session')
            with self.lock:
                if self.paused and sess != self.session:
                    self.paused = False                   # 보내는 쪽이 새로 시작했으면 일시정지 풀기
                    self.rx.hold = False
                    log.info('new sender session while paused -> resume')
                paused = self.paused
            if not paused:
                self.process(f, sess)
        with self.lock:
            if self.frame is not None and not self.paused and now - max(self.frame['recv_t'], self.resume_t) > 5.0:
                self.frame, self.snap, self.rgb, self.active = None, None, None, {}
                log.info('no frame for 5 s -> disconnected')

    def process(self, f, sess):
        hdr = f['hdr']
        t0 = time.perf_counter()
        try:
            self.mesh.build(f, 0)
        except Exception:
            if time.time() - self.err_t > 10:
                log.exception('mesh.build')
                self.err_t = time.time()
        try:
            self.boxes, floor = self.layers.update(f, self.mesh.boxes)
        except Exception:
            if time.time() - self.err_t > 10:
                log.exception('layers')
                self.err_t = time.time()
            self.boxes, floor = list(self.mesh.boxes or []), None
        self.base = floor if floor is not None else (self.mesh.base or 0.0)
        sc = hdr.get('script') or {}
        if sc.get('boxes') is not None:              # 영상용 장면(make_scene.py): 정해 준 반듯한 상자를 그대로 (깊이 추정 대신)
            self.boxes = [dict(x) for x in sc['boxes']]
            if sc.get('base') is not None:
                self.base = float(sc['base'])
        try:
            self.reg.update(self.boxes, hdr.get('sensors'))
        except Exception:
            log.exception('registration')
        self.build_ms.append((time.perf_counter() - t0) * 1000)
        info = dict(self.mesh.twin.info)
        self.alerts.update(hdr, info)
        rk = risk_json(hdr.get('sensors'))
        if rk is not None:          # 팀 파이프라인 판정(Hold 로 이미 걸러짐)을 그대로. 소리는 단계(주의 · 경고 · 위험) + 이유 음성
            now = time.time()
            self.alerts.active = {c['key']: (c['name'] + (' · ' + c['reason'] if c['reason'] else ''), now + 1.0, c['level'] >= 2,
                                             c['level'], reason_clip(c['key'], c['reason']))
                                  for c in rk['checks'] if c['level']}
        # 적재 모드는 소리만 끈다 (울리던 경보도 바로 끊는다). 주행으로 바꾸는 순간 이미 떠 있는 경고 · 주의는 새로 뜬 것으로 울린다
        mode = (hdr.get('sensors') or {}).get('mode')
        if mode in ('load', 'drive'):
            self.mode_by = 'sensor'
        else:
            mode, self.mode_by = self.mode_local, 'display'
        played = self.voice.played
        if mode == 'load':
            self.voice.update({})
            if getattr(self.voice, 'ok', False) and self.voice.ch.get_busy():
                self.voice.ch.stop()
        else:
            self.voice.update(self.alerts.active)
        if self.voice.played != played:
            VOICE.push(self.voice.sound_id)
        ready = info.get('state') == 'ready'
        g = hdr.get('grid')
        grid = None
        if g:
            grid = dict(x0=num(g['x0'], 4), x1=num(g['x0'] + g['nx'] * g['cell'], 4),
                        y0=num(g['y0'], 4), y1=num(g['y0'] + g['ny'] * g['cell'], 4))
        send_t = num(hdr.get('send_t'))
        snap = dict(
            source='replay' if 'replay' in (hdr.get('src') or ()) else 'live',   # 녹화 재생기(replay_runs.py)는 src 에 replay
            mode=mode, mode_by=self.mode_by,
            run=hdr.get('run'), scenario=(hdr.get('scenario') or '').replace('_', ' ') or None,
            t_run=num(hdr.get('t_run'), 3), dur_run=num(hdr.get('dur_run'), 3),
            grid=grid, base=num(self.base, 4) or 0.0,
            boxes=[box_json(b) for b in self.boxes],
            sensors=sensors_json(hdr.get('sensors'), hdr.get('sensor_age')),
            reg=self.reg.json(),
            risk=risk_json(hdr.get('sensors')),
            twin=dict(state='ready' if ready else 'building',
                      built=int(info.get('built', BUILD_N if ready else 0)),
                      changed_pct=num(info.get('changed_pct'), 2) or 0.0,
                      pending_pct=num(info.get('pending_pct'), 2) or 0.0,
                      shaking=bool(info.get('shaking', False))),
            latency_ms=num((f['recv_t'] - send_t) * 1000, 1) if send_t else None)
        with self.lock:
            self.frame, self.session, self.snap = f, sess, snap
            self.active = dict(self.alerts.active)
            self.rgb = f.get('rgb')
            self.camera_seq += 1

    def state(self, volume):
        now = time.time()
        with self.lock:
            snap, paused, active, seq = self.snap, self.paused, dict(self.active), self.camera_seq
        alerts = [dict(kind=k, text=v[0], severe=bool(v[2])) for k, v in active.items() if paused or v[1] >= now]
        alerts.sort(key=lambda a: (not a['severe'], a['kind']))
        st = dict(t=round(now, 3), connected=snap is not None, paused=paused,
                  source=snap['source'] if snap else 'none', mode=None, mode_by=None,
                  run=None, scenario=None, t_run=None, dur_run=None, grid=None,
                  base=num(self.base, 4) or 0.0, boxes=[], sensors=sensors_json(None, None), risk=None,
                  twin=dict(EMPTY_TWIN), alerts=alerts if snap else [],
                  link=dict(hz=round(self.rx.rate(), 2), lost=int(self.rx.lost),
                            latency_ms=snap['latency_ms'] if snap else None, frames=int(self.rx.ok)),
                  camera_seq=seq, volume=volume, reg=self.reg.json())
        if snap:
            for k in ('mode', 'mode_by', 'run', 'scenario', 't_run', 'dur_run', 'grid', 'base', 'boxes', 'sensors', 'risk', 'twin', 'reg'):
                st[k] = snap[k]
        return st

    def camera_jpeg(self):
        """(seq, JPEG 바이트) 또는 (seq, None). 같은 seq 면 다시 만들지 않는다."""
        with self.lock:
            rgb, seq = self.rgb, self.camera_seq
        if rgb is None:
            return seq, None
        with self.jpeg_lock:
            if self.jpeg[0] != seq:
                buf = io.BytesIO()
                Image.fromarray(np.ascontiguousarray(rgb, np.uint8)).save(buf, 'JPEG', quality=80)
                self.jpeg = (seq, buf.getvalue())
            return self.jpeg

    def status(self):
        with self.lock:
            paused, conn = self.paused, self.snap is not None
        bm = float(np.median(self.build_ms)) if self.build_ms else 0.0
        return 'recv %.1f Hz ok %d lost %d build %.1f ms connected %s paused %s' % (
            self.rx.rate(), self.rx.ok, self.rx.lost, bm, conn, paused)


# ---------------------------------------------------------------- SSE 방송 (모든 접속자에게 같은 JSON)
class Hub:
    PERIOD = 0.1

    def __init__(self, twin, volume):
        self.twin, self.volume = twin, volume
        self.cond = threading.Condition()
        self.msg, self.mid = None, 0
        self.clients = 0
        threading.Thread(target=self.loop, daemon=True, name='hub').start()

    def state_bytes(self):
        return dumps(self.twin.state(self.volume.get()))

    def loop(self):
        nxt = time.monotonic()
        while True:
            nxt += self.PERIOD
            if self.clients > 0:
                try:
                    b = self.state_bytes()
                    with self.cond:
                        self.msg, self.mid = b, self.mid + 1
                        self.cond.notify_all()
                except Exception:
                    log.exception('hub')
            d = nxt - time.monotonic()
            if d > 0:
                time.sleep(d)
            else:
                nxt = time.monotonic()

    def wait(self, last, timeout):
        with self.cond:
            if self.mid == last:
                self.cond.wait(timeout)
            return self.msg, self.mid


# ---------------------------------------------------------------- 키오스크 닫기
def kiosk_pids():
    """launch.sh 가 띄운 크로미움 본체 PID (같은 --user-data-dir, --type= 없는 것).
    크로미움 본체는 프로세스 이름을 고쳐 써서 cmdline 이 NUL 이 아니라 공백 하나로 이어진 문자열이다 -> 공백으로 나눈다."""
    want = '--user-data-dir=' + KIOSK_PROFILE
    out = []
    for d in os.listdir('/proc'):
        if not d.isdigit():
            continue
        try:
            with open('/proc/%s/cmdline' % d, 'rb') as f:
                toks = f.read().replace(b'\0', b' ').decode(errors='ignore').split()
        except OSError:
            continue
        if toks and os.path.basename(toks[0]).startswith('chromium') and want in toks \
                and not any(t.startswith('--type=') for t in toks):
            out.append(int(d))
    return out


def close_kiosk():
    pids = set()
    try:
        with open(os.path.join(RUN_DIR, 'chromium.pid')) as f:
            p = int(f.read().strip() or 0)
        if p and p in kiosk_pids():
            pids.add(p)
    except (OSError, ValueError):
        pass
    pids.update(kiosk_pids())
    for p in pids:
        try:
            os.kill(p, signal.SIGTERM)
        except OSError:
            pass
    log.info('exit: kiosk chromium %s', sorted(pids) or 'none')
    return sorted(pids)


# ---------------------------------------------------------------- HTTP
MIME = {'.html': 'text/html; charset=utf-8', '.htm': 'text/html; charset=utf-8',
        '.js': 'text/javascript; charset=utf-8', '.mjs': 'text/javascript; charset=utf-8',
        '.css': 'text/css; charset=utf-8', '.json': 'application/json; charset=utf-8', '.map': 'application/json',
        '.webmanifest': 'application/manifest+json', '.txt': 'text/plain; charset=utf-8',
        '.svg': 'image/svg+xml', '.png': 'image/png', '.jpg': 'image/jpeg', '.jpeg': 'image/jpeg',
        '.webp': 'image/webp', '.gif': 'image/gif', '.ico': 'image/x-icon', '.avif': 'image/avif',
        '.ttf': 'font/ttf', '.otf': 'font/otf', '.woff': 'font/woff', '.woff2': 'font/woff2',
        '.mp4': 'video/mp4', '.m4v': 'video/mp4', '.webm': 'video/webm', '.ogv': 'video/ogg',
        '.mp3': 'audio/mpeg', '.wav': 'audio/wav', '.ogg': 'audio/ogg',
        '.wasm': 'application/wasm', '.glb': 'model/gltf-binary', '.gltf': 'model/gltf+json',
        '.hdr': 'application/octet-stream', '.bin': 'application/octet-stream'}
QUIET = ('/api/camera.jpg', '/api/stream', '/api/state', '/api/volume', '/api/voice')
DISCONNECT = (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, socket.timeout, TimeoutError)


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    server_version = 'CargoTwin/1.0'
    timeout = 60
    app = None                     # App (main 에서 넣는다)

    # 요청마다 찍으면 카메라 사진 10 Hz 때문에 로그가 넘친다 -> 오류 · 드문 요청만
    def log_message(self, fmt, *args):
        pass

    def log_request(self, code='-', size='-'):
        path = urlsplit(self.path).path
        try:
            c = int(code)
        except (TypeError, ValueError):
            c = 0
        if c >= 400 or not (path.startswith(QUIET) or path.startswith('/assets/')):
            log.info('%s %s %s %s', self.client_address[0], self.command, self.path[:120], code)

    def cors(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')

    def send_json(self, obj, code=200):
        body = obj if isinstance(obj, bytes) else dumps(obj)
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.cors()
        self.end_headers()
        if self.command != 'HEAD':
            self.wfile.write(body)

    def send_empty(self, code, extra=()):
        self.send_response(code)
        for k, v in extra:
            self.send_header(k, v)
        if code not in (204, 304):                          # 204/304 에는 Content-Length 를 붙이지 않는다
            self.send_header('Content-Length', '0')
        self.cors()
        self.end_headers()

    def body_json(self):
        n = int(self.headers.get('Content-Length') or 0)
        if n <= 0:
            return {}
        if n > 1 << 20:
            raise ValueError('너무 큰 요청')
        raw = self.rfile.read(n)
        obj = json.loads(raw.decode('utf-8') or '{}')
        if not isinstance(obj, dict):
            raise ValueError('JSON 객체가 아닙니다')
        return obj

    def guard(fn):
        @functools.wraps(fn)
        def wrap(self):
            try:
                fn(self)
            except DISCONNECT:
                self.close_connection = True
            except Exception as e:
                log.exception('%s %s', self.command, self.path)
                try:
                    self.send_json(dict(ok=False, error=str(e)), 500)
                except Exception:
                    self.close_connection = True
        return wrap

    @guard
    def do_OPTIONS(self):
        self.send_empty(204)

    @guard
    def do_HEAD(self):
        self.static(urlsplit(self.path).path, head=True)

    @guard
    def do_GET(self):
        path = urlsplit(self.path).path
        a = self.app
        if path == '/api/stream':
            return self.sse()
        if path == '/api/state':
            return self.send_json(a.hub.state_bytes())
        if path == '/api/camera.jpg':
            seq, jpg = a.twin.camera_jpeg()
            if jpg is None:
                return self.send_empty(204, [('Cache-Control', 'no-store')])
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(jpg)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Camera-Seq', str(seq))
            self.cors()
            self.end_headers()
            self.wfile.write(jpg)
            return
        if path == '/api/volume':
            return self.send_json(a.volume.get())
        if path == '/api/voice':
            q = dict(p.split('=', 1) for p in urlsplit(self.path).query.split('&') if '=' in p)
            try:
                after = int(q.get('after', '-1'))
            except ValueError:
                after = -1
            return self.send_json(VOICE.wait(after))
        if path == '/api/voice.wav':
            kind = dict(p.split('=', 1) for p in urlsplit(self.path).query.split('&') if '=' in p).get('kind')
            if not getattr(a.twin.voice, 'ok', False) or VoiceFeed.parse(a.twin.voice, kind) is None:
                return self.send_json(dict(ok=False, error='소리 없음'), 404)
            body = VOICE.wav_bytes(a.twin.voice, kind)
            self.send_response(200)
            self.send_header('Content-Type', 'audio/wav')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'max-age=3600')
            self.cors()
            self.end_headers()
            if self.command != 'HEAD':
                self.wfile.write(body)
            return
        if path.startswith('/api/'):
            return self.send_json(dict(ok=False, error='없는 API'), 404)
        self.static(path)

    @guard
    def do_POST(self):
        path = urlsplit(self.path).path
        a = self.app
        try:
            body = self.body_json()
        except ValueError as e:
            return self.send_json(dict(ok=False, error='잘못된 JSON: %s' % e), 400)
        try:
            if path == '/api/control':
                return self.send_json(a.twin.control(str(body.get('cmd', ''))))
            if path == '/api/mode':
                return self.send_json(a.twin.set_mode(str(body.get('mode', ''))))
            if path == '/api/volume':
                return self.send_json(a.volume.act(str(body.get('action', '')), body.get('value')))
        except ValueError as e:
            return self.send_json(dict(ok=False, error=str(e)), 400)
        if path == '/api/exit':
            self.send_json(dict(ok=True))                  # 먼저 응답하고 닫는다
            threading.Timer(0.3, close_kiosk).start()
            return
        if path == '/api/stats':
            log.info('stats %s', json.dumps(body, ensure_ascii=False)[:600])
            return self.send_json(dict(ok=True))
        self.send_json(dict(ok=False, error='없는 API'), 404)

    # -- SSE
    def sse(self):
        a = self.app
        self.close_connection = True
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
        self.send_header('Cache-Control', 'no-cache, no-transform')
        self.send_header('Connection', 'close')
        self.send_header('X-Accel-Buffering', 'no')
        self.cors()
        self.end_headers()
        hub = a.hub
        with hub.cond:
            hub.clients += 1
        log.info('sse open %s (clients %d)', self.client_address[0], hub.clients)
        sent = 0
        try:
            self.wfile.write(b'retry: 1000\n\n' + b'data: ' + hub.state_bytes() + b'\n\n')
            last, ka = hub.mid, time.monotonic()
            while not a.stopping:
                msg, mid = hub.wait(last, 1.0)
                if mid != last and msg is not None:
                    self.wfile.write(b'data: ' + msg + b'\n\n')
                    last = mid
                    sent += 1
                if time.monotonic() - ka >= 5.0:
                    self.wfile.write(b': keepalive\n\n')
                    ka = time.monotonic()
        except DISCONNECT + (OSError,):
            pass
        finally:
            with hub.cond:
                hub.clients -= 1
            log.info('sse closed %s after %d msgs (clients %d)', self.client_address[0], sent, hub.clients)

    # -- 정적 파일 (SPA)
    def static(self, path, head=False):
        rel = unquote(path).lstrip('/')
        root = os.path.realpath(DIST)
        full = os.path.realpath(os.path.join(root, rel)) if rel else os.path.join(root, 'index.html')
        if os.path.commonpath([root, full]) != root:
            return self.send_json(dict(ok=False, error='금지된 경로'), 403)
        if os.path.isdir(full):
            full = os.path.join(full, 'index.html')
        if not os.path.isfile(full):
            last = rel.rsplit('/', 1)[-1]
            if '.' in last:                                  # 없는 파일(.js 등)은 404, 그 밖의 경로는 SPA
                return self.send_json(dict(ok=False, error='파일 없음: %s' % rel), 404)
            full = os.path.join(root, 'index.html')
            if not os.path.isfile(full):
                return self.send_json(dict(ok=False, error='dist/index.html 없음 (웹 빌드를 복사하세요)'), 404)
        self.send_file(full, head)

    def send_file(self, full, head):
        st = os.stat(full)
        size = st.st_size
        ext = os.path.splitext(full)[1].lower()
        ctype = MIME.get(ext, 'application/octet-stream')
        etag = '"%x-%x"' % (st.st_mtime_ns, size)
        relp = os.path.relpath(full, os.path.realpath(DIST)).replace(os.sep, '/')
        if relp.startswith('assets/') and ext in ('.js', '.css', '.woff2', '.woff', '.ttf'):
            cache = 'public, max-age=31536000, immutable'   # vite 가 해시를 붙인 파일
        else:
            cache = 'no-cache'                              # index.html, 동영상 등: 매번 확인 (ETag)
        start, end, code = 0, size - 1, 200
        rng = self.headers.get('Range')
        if rng and self.headers.get('If-Range') not in (None, etag):
            rng = None
        if rng:
            m = re.fullmatch(r'\s*bytes\s*=\s*(\d*)\s*-\s*(\d*)\s*', rng)
            if m and (m.group(1) or m.group(2)):
                if m.group(1):
                    start = int(m.group(1))
                    end = min(int(m.group(2)), size - 1) if m.group(2) else size - 1
                else:                                       # bytes=-N (끝에서 N 바이트)
                    start = max(size - int(m.group(2)), 0)
                if start >= size or start > end:
                    return self.send_empty(416, [('Content-Range', 'bytes */%d' % size), ('Accept-Ranges', 'bytes')])
                code = 206
        elif self.headers.get('If-None-Match') == etag:
            self.send_response(304)
            self.send_header('ETag', etag)
            self.send_header('Cache-Control', cache)
            self.end_headers()
            return
        length = end - start + 1 if size else 0
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(length))
        self.send_header('Accept-Ranges', 'bytes')
        self.send_header('ETag', etag)
        self.send_header('Last-Modified', email.utils.formatdate(st.st_mtime, usegmt=True))
        self.send_header('Cache-Control', cache)
        if code == 206:
            self.send_header('Content-Range', 'bytes %d-%d/%d' % (start, end, size))
        self.end_headers()
        if head or not length:
            return
        with open(full, 'rb') as f:
            f.seek(start)
            left = length
            while left > 0:
                chunk = f.read(min(256 << 10, left))
                if not chunk:
                    break
                self.wfile.write(chunk)
                left -= len(chunk)


VIEWER_META = b'<meta name="scss-viewer" content="1" />'
VIEWER_MAX_SSE = 40              # 파이가 버틸 만큼만 (보기 한 명당 상태 약 10 Hz)


class ViewerHandler(Handler):
    """보기 전용 (다른 사람들이 인터넷 · LAN 으로 보는 주소). 화면 · 상태 · 카메라는 키오스크와 같고,
    키오스크를 바꾸는 요청(일시정지 · 음량 · 끝내기)은 막는다. 화면은 index.html 의 meta 로 보기 전용인 줄 안다."""
    server_version = 'CargoTwinViewer/1.0'

    @Handler.guard
    def do_POST(self):
        path = urlsplit(self.path).path
        # 본문은 쓰지 않아도 다 읽어 둔다 (안 읽으면 같은 연결의 다음 요청 앞에 붙어 501 로 깨진다)
        n = int(self.headers.get('Content-Length') or 0)
        if n > 1 << 20:
            self.close_connection = True
            return self.send_json(dict(ok=False, error='너무 큰 요청'), 413)
        if n > 0:
            self.rfile.read(n)
        if path == '/api/stats':                         # 보는 사람 브라우저의 프레임 속도 보고: 받기만 한다
            return self.send_json(dict(ok=True))
        self.send_json(dict(ok=False, error='보기 전용 주소입니다'), 403)

    @Handler.guard
    def do_GET(self):
        path = urlsplit(self.path).path
        if path == '/api/stream' and self.app.hub.clients >= VIEWER_MAX_SSE:
            return self.send_json(dict(ok=False, error='보는 사람이 너무 많습니다. 잠시 뒤 다시 열어 주세요'), 503)
        Handler.do_GET.__wrapped__(self)

    def send_file(self, full, head):
        if os.path.basename(full) != 'index.html':
            return Handler.send_file(self, full, head)
        with open(full, 'rb') as f:
            body = f.read().replace(b'</head>', VIEWER_META + VOICE_JS + b'</head>', 1)
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        if not head:
            self.wfile.write(body)


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True
    request_queue_size = 32

    def handle_error(self, request, client_address):
        e = sys.exc_info()[1]
        if not isinstance(e, DISCONNECT + (OSError,)):
            log.exception('http %s', client_address)


class App:
    def __init__(self, tcp_port):
        self.stopping = False
        self.twin = Twin(tcp_port)
        self.volume = Volume(Beep(self.twin.voice))
        self.hub = Hub(self.twin, self.volume)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--http', type=int, default=int(os.environ.get('TWIN_HTTP_PORT', 8090)))
    ap.add_argument('--tcp', type=int, default=5010)
    ap.add_argument('--view', type=int, default=int(os.environ.get('TWIN_VIEW_PORT', 8091)),
                    help='보기 전용 HTTP 포트 (0 이면 끔)')
    args = ap.parse_args()
    setup_log()
    os.makedirs(RUN_DIR, exist_ok=True)
    try:
        app = App(args.tcp)
        Handler.app = app
        srv = Server(('0.0.0.0', args.http), Handler)
        view = Server(('0.0.0.0', args.view), ViewerHandler) if args.view else None
    except OSError as e:                                     # 포트를 다른 프로그램(옛 twin_app.py 등)이 쓰는 중
        log.error('시작 실패: %s (tcp %d / http %d 를 쓰는 프로그램을 먼저 끄세요)', e, args.tcp, args.http)
        sys.exit(1)
    log.info('twin_server: http :%d  tcp :%d  dist %s%s  volume %s', args.http, args.tcp, DIST,
             '' if os.path.isdir(DIST) else ' (없음)', app.volume.get())

    def on_term(sig, _frm):
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, on_term)
    signal.signal(signal.SIGHUP, on_term)

    def status():
        while True:
            time.sleep(30)
            log.info('%s  sse %d', app.twin.status(), app.hub.clients)
    threading.Thread(target=status, daemon=True, name='status').start()
    if view:
        threading.Thread(target=view.serve_forever, kwargs=dict(poll_interval=0.5), daemon=True, name='viewer').start()
        log.info('보기 전용: http :%d (일시정지 · 음량 · 끝내기 막음)', args.view)
    try:
        srv.serve_forever(poll_interval=0.5)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        app.stopping = True
        if app.twin.paused:
            app.twin.send('resume')                          # 보내는 쪽을 멈춘 채로 두지 않는다
        srv.server_close()
        if view:
            view.server_close()
        log.info('twin_server stopped')


if __name__ == '__main__':
    main()
