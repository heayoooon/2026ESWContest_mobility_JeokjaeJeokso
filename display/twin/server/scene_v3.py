"""디지털 트윈 3D 장면 v3 (SF 대시보드 스타일, OpenGL 고정 파이프라인).

- 배경: 남색 그라데이션 + 받침판 아래로 퍼지는 바닥 격자 + 3 s 마다 퍼지는 레이더 파동
- 받침판: 유리판(반투명 윗면 · 옆면) + 빛나는 테두리 + 모서리 꺾쇠. 도식이다 (스캔한 카트가 아니다)
- 블록 3가지 표현: 0 = 실사(카메라 색을 옅게 입힌 흰 모형), 1 = 높이색, 2 = 홀로그램(반투명 + 밝은 모서리)
  변화가 확정된 블록(빨갛게 물든 블록)은 모서리가 빨갛게 맥동한다
"""
import math
import numpy as np
from OpenGL.GL import *  # noqa: F401,F403
from cargo_model import box_geometry, cylinder_geometry
from scene_v2 import tone, FACE_N, LIGHT

CYAN = (.36, .86, 1.0)
_cache = {}


def background(t):
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity(); glOrtho(0, 1, 0, 1, -1, 1)
    glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity()
    glDisable(GL_DEPTH_TEST)
    glBegin(GL_QUADS)
    glColor3f(.030, .075, .135); glVertex2f(0, 0); glVertex2f(1, 0)
    glColor3f(.012, .025, .052); glVertex2f(1, 1); glVertex2f(0, 1)
    glEnd()
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)   # 가운데 은은한 빛
    glBegin(GL_TRIANGLE_FAN)
    glColor4f(.10, .30, .50, .22); glVertex2f(.5, .42)
    glColor4f(.10, .30, .50, 0)
    for a in np.linspace(0, 2 * math.pi, 41):
        glVertex2f(.5 + .55 * math.cos(a), .42 + .62 * math.sin(a))
    glEnd()
    glDisable(GL_BLEND)
    glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix(); glMatrixMode(GL_MODELVIEW)
    glEnable(GL_DEPTH_TEST)


def ground(cx, cy, z, t, r=1.1, step=.1):
    """받침판 아래 바닥 격자 (중심에서 멀수록 흐려짐) + 퍼지는 파동."""
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); glDepthMask(GL_FALSE)
    key = ('ground', round(cx, 3), round(cy, 3), round(z, 3))
    if key not in _cache:                                # 정점 배열은 한 번만 만든다 (매 프레임 glVertex 수천 번은 느리다)
        pts = []
        n = 12
        for k in np.arange(-r, r + 1e-6, step):
            for a0, b0, a1, b1 in ((cx + k, cy - r, cx + k, cy + r), (cx - r, cy + k, cx + r, cy + k)):
                for i in range(n):
                    pts.append((a0 + (a1 - a0) * i / n, b0 + (b1 - b0) * i / n))
                    pts.append((a0 + (a1 - a0) * (i + 1) / n, b0 + (b1 - b0) * (i + 1) / n))
        p = np.array(pts, np.float32)
        d = np.hypot(p[:, 0] - cx, p[:, 1] - cy) / r
        verts = np.ascontiguousarray(np.column_stack([p, np.full(len(p), z, np.float32)]), np.float32)
        cols = np.ascontiguousarray(np.column_stack([np.tile([.20, .55, .80], (len(p), 1)),
                                                     .30 * np.clip(1 - d, 0, 1) ** 1.5]), np.float32)
        _cache[key] = (verts, cols)
    verts, cols = _cache[key]
    glLineWidth(1)
    glEnableClientState(GL_VERTEX_ARRAY); glEnableClientState(GL_COLOR_ARRAY)
    glVertexPointer(3, GL_FLOAT, 0, verts); glColorPointer(4, GL_FLOAT, 0, cols)
    glDrawArrays(GL_LINES, 0, len(verts))
    glDisableClientState(GL_COLOR_ARRAY); glDisableClientState(GL_VERTEX_ARRAY)
    ph = (t % 3.0) / 3.0                                  # 레이더 파동
    rr = .12 + ph * r
    glLineWidth(2)
    glColor4f(.35, .85, 1.0, .45 * (1 - ph)); glBegin(GL_LINE_LOOP)
    for a in np.linspace(0, 2 * math.pi, 72, endpoint=False):
        glVertex3f(cx + rr * math.cos(a), cy + rr * math.sin(a), z + .0005)
    glEnd()
    glLineWidth(1); glDepthMask(GL_TRUE); glDisable(GL_BLEND)


def platform(x0, x1, y0, y1, z, t):
    m, th = .03, .04
    X0, X1, Y0, Y1 = x0 - m, x1 + m, y0 - m, y1 + m
    zb = z - th
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glDepthMask(GL_FALSE)
    glBegin(GL_QUADS)                                     # 옆면 (유리)
    for (ax, ay), (bx, by), k in (((X0, Y0), (X1, Y0), 1.0), ((X1, Y0), (X1, Y1), .8), ((X1, Y1), (X0, Y1), .6), ((X0, Y1), (X0, Y0), .7)):
        glColor4f(.08 * k, .30 * k, .50 * k, .55); glVertex3f(ax, ay, zb); glVertex3f(bx, by, zb)
        glColor4f(.16 * k, .52 * k, .80 * k, .70); glVertex3f(bx, by, z); glVertex3f(ax, ay, z)
    glEnd()
    glBegin(GL_QUADS)                                     # 윗면
    glColor4f(.10, .30, .48, .60); glVertex3f(X0, Y0, z); glVertex3f(X1, Y0, z)
    glColor4f(.06, .20, .36, .60); glVertex3f(X1, Y1, z); glVertex3f(X0, Y1, z)
    glEnd()
    glDepthMask(GL_TRUE)
    glColor4f(.45, .80, 1.0, .13); glBegin(GL_LINES)     # 5 cm 격자
    for x in np.arange(X0 + .05, X1, .05):
        glVertex3f(x, Y0, z + .0008); glVertex3f(x, Y1, z + .0008)
    for y in np.arange(Y0 + .05, Y1, .05):
        glVertex3f(X0, y, z + .0008); glVertex3f(X1, y, z + .0008)
    glEnd()
    pulse = .75 + .25 * math.sin(t * 2.2)
    for w, a in ((7, .10), (3.5, .25), (1.5, .95)):      # 빛나는 테두리 (위 · 아래)
        glLineWidth(w)
        for zz, k in ((z + .001, 1.0), (zb, .45)):
            glColor4f(CYAN[0], CYAN[1], CYAN[2], a * k * pulse); glBegin(GL_LINE_LOOP)
            for x, y in ((X0, Y0), (X1, Y0), (X1, Y1), (X0, Y1)):
                glVertex3f(x, y, zz)
            glEnd()
    glLineWidth(3); glColor4f(.75, .95, 1.0, .95)         # 모서리 꺾쇠
    L = .045
    glBegin(GL_LINES)
    for x, y, sx, sy in ((X0, Y0, 1, 1), (X1, Y0, -1, 1), (X1, Y1, -1, -1), (X0, Y1, 1, -1)):
        zz = z + .003
        glVertex3f(x, y, zz); glVertex3f(x + sx * L, y, zz)
        glVertex3f(x, y, zz); glVertex3f(x, y + sy * L, zz)
        glVertex3f(x, y, zz); glVertex3f(x, y, zz + L * .8)
    glEnd()
    glLineWidth(1); glDisable(GL_BLEND)


def is_alert(b):
    c = np.asarray(b.get('color', (0, 0, 0)), float)
    return c[0] > .5 and c[1] < .42 and c[2] < .42           # 확정 변화로 빨갛게 물든 블록


def styled(boxes, mode):
    key = (mode, tuple((b.get('id'), round(b['x0'], 4), round(b['x1'], 4), round(b['y0'], 4), round(b['y1'], 4),
                        round(b['top'], 4), round(b['bottom'], 4), tuple(np.round(b['color'], 3))) for b in boxes))
    if key in _cache:
        return _cache[key]
    out = []
    for b in boxes:
        alert = is_alert(b)
        if b.get('shape') == 'cylinder':
            v, tri, edges = cylinder_geometry(b)
            verts = np.ascontiguousarray(v[tri], np.float32)
            tt = verts.reshape(-1, 3, 3)
            n = np.cross(tt[:, 1] - tt[:, 0], tt[:, 2] - tt[:, 0]); n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-7)
            lam = (.55 + .5 * np.abs(n @ LIGHT)).repeat(3)[:, None]
            base = np.array([.25, .62, .92]) if mode != 1 else np.array([.95, .70, .25])
            cols = np.clip(lam * base, 0, 1)
            out.append(dict(verts=verts, cols=np.ascontiguousarray(cols, np.float32),
                            edges=np.ascontiguousarray(v[edges].reshape(-1, 3), np.float32), top=None, alert=alert, b=b))
            continue
        v, tri, edges = box_geometry(b)
        if mode == 1:                                    # 높이색: 낮음 청록 -> 높음 주황
            h = np.clip((b['top'] - .35) / .35, 0, 1)
            base = np.array([.15, .80, .75]) * (1 - h) + np.array([1.0, .66, .22]) * h
        else:                                            # 실사: 흰 모형에 카메라 색을 옅게
            base = np.clip(np.array([.80, .85, .90]) * .55 + tone(np.asarray(b['color'], float)) * .45, 0, 1)
        verts = np.ascontiguousarray(v[tri], np.float32)
        lam = np.clip(.45 + .60 * (FACE_N @ LIGHT), .40, 1.10); lam[0] = 1.12
        z0, z1 = b['bottom'], b['top']
        ao = .58 + .42 * np.clip((verts[:, 2] - z0) / max(z1 - z0, 1e-4), 0, 1); ao[:6] = 1.0
        cols = np.clip(lam.repeat(6)[:, None] * ao[:, None] * base, 0, 1)
        out.append(dict(verts=verts, cols=np.ascontiguousarray(cols, np.float32),
                        edges=np.ascontiguousarray(v[edges].reshape(-1, 3), np.float32),
                        top=np.ascontiguousarray(v[[4, 5, 5, 6, 6, 7, 7, 4]], np.float32), alert=alert, b=b,
                        holo=np.ascontiguousarray(np.clip(lam.repeat(6)[:, None] * np.array([.10, .42, .85]), 0, 1), np.float32)))
    if len(_cache) > 50:
        for k in [k for k in _cache if k[0] != 'ground']:
            del _cache[k]
    _cache[key] = out
    return out


def blocks(geom, base, mode, t):
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glDepthMask(GL_FALSE)
    for g in geom:                                       # 바닥 빛 (블록 밑 청록 / 경고면 빨강)
        b = g['b']
        if g['top'] is None:
            continue
        c = (1.0, .30, .25) if g['alert'] else (.25, .85, 1.0)
        for grow, a in ((.022, .10), (.008, .22)):
            glColor4f(c[0], c[1], c[2], a); glBegin(GL_QUADS)
            for x, y in ((b['x0'] - grow, b['y0'] - grow), (b['x1'] + grow, b['y0'] - grow),
                         (b['x1'] + grow, b['y1'] + grow), (b['x0'] - grow, b['y1'] + grow)):
                glVertex3f(x, y, base + .0015)
            glEnd()
    glDepthMask(GL_TRUE)
    glEnableClientState(GL_VERTEX_ARRAY); glEnableClientState(GL_COLOR_ARRAY)
    if mode == 2:                                        # 홀로그램: 반투명 면
        glDepthMask(GL_FALSE)
        for g in geom:
            cols = g.get('holo', g['cols'])
            rgba = np.ascontiguousarray(np.column_stack([cols, np.full(len(cols), .30, np.float32)]), np.float32)
            glVertexPointer(3, GL_FLOAT, 0, g['verts']); glColorPointer(4, GL_FLOAT, 0, rgba)
            glDrawArrays(GL_TRIANGLES, 0, len(g['verts']))
        glDepthMask(GL_TRUE)
    else:
        glDisable(GL_BLEND)
        glEnable(GL_POLYGON_OFFSET_FILL); glPolygonOffset(1, 1)
        for g in geom:
            glVertexPointer(3, GL_FLOAT, 0, g['verts']); glColorPointer(3, GL_FLOAT, 0, g['cols'])
            glDrawArrays(GL_TRIANGLES, 0, len(g['verts']))
        glDisable(GL_POLYGON_OFFSET_FILL)
        glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glDisableClientState(GL_COLOR_ARRAY)
    glEnable(GL_LINE_SMOOTH)
    pulse = .6 + .4 * math.sin(t * 6)
    for g in geom:                                       # 모서리
        if g['alert']:
            for w, a in ((6, .25 * pulse), (2.2, .95)):
                glLineWidth(w); glColor4f(1.0, .32, .26, a)
                glVertexPointer(3, GL_FLOAT, 0, g['edges']); glDrawArrays(GL_LINES, 0, len(g['edges']))
            continue
        if mode == 2:
            for w, a in ((5, .18), (1.8, .95)):
                glLineWidth(w); glColor4f(.45, .90, 1.0, a)
                glVertexPointer(3, GL_FLOAT, 0, g['edges']); glDrawArrays(GL_LINES, 0, len(g['edges']))
        else:
            glLineWidth(1.4); glColor4f(.30, .80, .95, .55)
            glVertexPointer(3, GL_FLOAT, 0, g['edges']); glDrawArrays(GL_LINES, 0, len(g['edges']))
            if g['top'] is not None:
                glLineWidth(1.2); glColor4f(1, 1, 1, .45)
                glVertexPointer(3, GL_FLOAT, 0, g['top']); glDrawArrays(GL_LINES, 0, len(g['top']))
    glDisableClientState(GL_VERTEX_ARRAY); glDisable(GL_LINE_SMOOTH); glDisable(GL_BLEND); glLineWidth(1)


def render(frame, mesh, P, V, mode, t):
    """전체 화면 3D. mesh = twin_view.StableBoxMesh (boxes, base, ready)."""
    g = frame['hdr']['grid']
    x0, y0 = g['x0'], g['y0']; x1, y1 = x0 + g['nx'] * g['cell'], y0 + g['ny'] * g['cell']
    glClearColor(.012, .025, .05, 1); glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    background(t)
    glMatrixMode(GL_PROJECTION); glLoadMatrixf(P.T)
    glMatrixMode(GL_MODELVIEW); glLoadMatrixf(V.T)
    glEnable(GL_DEPTH_TEST)
    z = mesh.base
    ground((x0 + x1) / 2, (y0 + y1) / 2, z - .042, t)
    platform(x0, x1, y0, y1, z, t)
    if getattr(mesh, 'boxes', None):
        blocks(styled(mesh.boxes, mode), z, mode, t)
