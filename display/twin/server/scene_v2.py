"""상자 모형 디지털 트윈 3D 장면 꾸미기 (OpenGL 고정 파이프라인). twin_view.py 의 상자 모형 경로가 쓴다.

- 배경: 위아래 그라데이션 + 받침판 아래 은은한 빛
- 받침판: 두께 있는 판(윗면 · 옆면 음영) + 테두리 빛 + 5 cm 격자. 도식이다 (스캔한 카트가 아니다)
- 상자: 면별 조명 + 아래로 갈수록 어두운 음영(가짜 AO) + 윗면 테두리 하이라이트 + 바닥 그림자
"""
import math
import numpy as np
from OpenGL.GL import *  # noqa: F401,F403
from cargo_model import box_geometry, cylinder_geometry

LIGHT = np.array([.35, -.55, .76], np.float32); LIGHT /= np.linalg.norm(LIGHT)
FACE_N = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0], [0, 1, 0], [-1, 0, 0], [0, 0, -1]], np.float32)   # box_geometry 면 순서


def background(vw, vh):
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity(); glOrtho(0, 1, 0, 1, -1, 1)
    glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity()
    glDisable(GL_DEPTH_TEST)
    glBegin(GL_QUADS)
    glColor3f(.105, .135, .175); glVertex2f(0, 0); glVertex2f(1, 0)
    glColor3f(.035, .048, .068); glVertex2f(1, 1); glVertex2f(0, 1)
    glEnd()
    glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix(); glMatrixMode(GL_MODELVIEW)
    glEnable(GL_DEPTH_TEST)


def tone(c):
    """카메라 색 -> 보기 좋은 색: 감마로 밝히고 채도를 조금 올린다."""
    c = np.clip(np.power(np.clip(c, 0, 1), .75) * 1.08, 0, 1)
    m = c.mean()
    return np.clip(m + (c - m) * 1.45, 0, 1)


def styled(boxes, mode, height_cmap):
    """상자 목록 -> [(삼각형 정점, 정점 색, 모서리 선, 윗면 테두리, 그림자 사각형, held)]"""
    out = []
    for b in boxes:
        if b.get('shape') == 'cylinder':
            v, tri, edges = cylinder_geometry(b)
            verts = np.ascontiguousarray(v[tri], np.float32)
            t = verts.reshape(-1, 3, 3)
            n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]); n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-7)
            shade = (.55 + .5 * np.abs(n @ LIGHT)).repeat(3)
            base = tone(np.clip(b['color'] * .6 + np.array([.10, .28, .36]), 0, 1))
            cols = np.ascontiguousarray(np.clip(shade[:, None] * base, 0, 1), np.float32)
            out.append((verts, cols, np.ascontiguousarray(v[edges].reshape(-1, 3), np.float32), None, None, b.get('held')))
            continue
        v, tri, edges = box_geometry(b)
        base = height_cmap(np.array((b['top'] - .3) / .4)) if mode == 1 else tone(np.asarray(b['color'], float))
        if b.get('held'):
            base = base * .6 + np.array([.12, .16, .18])
        verts = np.ascontiguousarray(v[tri], np.float32)                 # 6면 x 6정점
        lam = np.clip(.42 + .62 * (FACE_N @ LIGHT), .38, 1.12)           # 면별 조명
        lam[0] = 1.12                                                    # 윗면은 밝게
        z0, z1 = b['bottom'], b['top']
        ao = .55 + .45 * np.clip((verts[:, 2] - z0) / max(z1 - z0, 1e-4), 0, 1)   # 아래로 갈수록 어둡게
        ao[:6] = 1.0
        cols = np.clip(lam.repeat(6)[:, None] * ao[:, None] * base, 0, 1).astype(np.float32)
        top = np.ascontiguousarray(v[[4, 5, 5, 6, 6, 7, 7, 4]], np.float32)
        sh = (b['x0'], b['x1'], b['y0'], b['y1'])
        out.append((verts, np.ascontiguousarray(cols), np.ascontiguousarray(v[edges].reshape(-1, 3), np.float32), top, sh, b.get('held')))
    return out


def platform(x0, x1, y0, y1, z, mode):
    m = .025; th = .035
    X0, X1, Y0, Y1 = x0 - m, x1 + m, y0 - m, y1 + m
    zb = z - th
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    # 판 아래 은은한 빛 (바닥 원판)
    cx, cy = (X0 + X1) / 2, (Y0 + Y1) / 2; r = max(X1 - X0, Y1 - Y0) * .95
    glDepthMask(GL_FALSE)
    glBegin(GL_TRIANGLE_FAN)
    glColor4f(.20, .55, .70, .22); glVertex3f(cx, cy, zb - .002)
    glColor4f(.20, .55, .70, 0.0)
    for a in np.linspace(0, 2 * math.pi, 49):
        glVertex3f(cx + r * math.cos(a), cy + r * math.sin(a), zb - .002)
    glEnd()
    glDepthMask(GL_TRUE)
    # 판 옆면 (위가 밝고 아래가 어두운)
    glBegin(GL_QUADS)
    for (ax, ay), (bx, by), k in (((X0, Y0), (X1, Y0), .85), ((X1, Y0), (X1, Y1), .7), ((X1, Y1), (X0, Y1), .55), ((X0, Y1), (X0, Y0), .62)):
        glColor3f(.10 * k, .15 * k, .20 * k); glVertex3f(ax, ay, zb); glVertex3f(bx, by, zb)
        glColor3f(.20 * k, .28 * k, .36 * k); glVertex3f(bx, by, z); glVertex3f(ax, ay, z)
    glEnd()
    # 윗면
    glBegin(GL_QUADS)
    glColor3f(.135, .185, .245); glVertex3f(X0, Y0, z); glVertex3f(X1, Y0, z)
    glColor3f(.11, .155, .21); glVertex3f(X1, Y1, z); glVertex3f(X0, Y1, z)
    glEnd()
    # 격자 5 cm
    glColor4f(.45, .70, .85, .16); glLineWidth(1); glBegin(GL_LINES)
    for x in np.arange(X0 + .05, X1, .05):
        glVertex3f(x, Y0, z + .0008); glVertex3f(x, Y1, z + .0008)
    for y in np.arange(Y0 + .05, Y1, .05):
        glVertex3f(X0, y, z + .0008); glVertex3f(X1, y, z + .0008)
    glEnd()
    # 테두리 빛
    for w, a in ((6, .12), (3, .28), (1.4, .9)):
        glLineWidth(w); glColor4f(.30, .78, .95, a); glBegin(GL_LINE_LOOP)
        for x, y in ((X0, Y0), (X1, Y0), (X1, Y1), (X0, Y1)):
            glVertex3f(x, y, z + .001)
        glEnd()
    glLineWidth(1)
    glDisable(GL_BLEND)


def boxes(geom, base, mode):
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    # 바닥 그림자 (두 겹으로 부드럽게). 판 위에 살짝 띄워 그린다
    glDepthMask(GL_FALSE)
    for _, _, _, _, sh, _ in geom:
        if sh is None:
            continue
        x0, x1, y0, y1 = sh
        for grow, a in ((.018, .16), (.006, .30)):
            glColor4f(0, 0, 0, a); glBegin(GL_QUADS)
            for x, y in ((x0 - grow + .01, y0 - grow - .01), (x1 + grow + .01, y0 - grow - .01),
                         (x1 + grow + .01, y1 + grow - .01), (x0 - grow + .01, y1 + grow - .01)):
                glVertex3f(x, y, base + .0015)
            glEnd()
    glDepthMask(GL_TRUE)
    glDisable(GL_BLEND)
    # 면
    glEnable(GL_POLYGON_OFFSET_FILL); glPolygonOffset(1, 1)
    glEnableClientState(GL_VERTEX_ARRAY); glEnableClientState(GL_COLOR_ARRAY)
    for verts, cols, _, _, _, _ in geom:
        glVertexPointer(3, GL_FLOAT, 0, verts); glColorPointer(3, GL_FLOAT, 0, cols)
        glDrawArrays(GL_TRIANGLES, 0, len(verts))
    glDisableClientState(GL_COLOR_ARRAY); glDisable(GL_POLYGON_OFFSET_FILL)
    # 모서리 + 윗면 하이라이트
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); glEnable(GL_LINE_SMOOTH)
    for _, _, edges, top, _, held in geom:
        glLineWidth(1.6)
        if held or mode == 2:
            glColor4f(.38, .80, .92, .95)
        else:
            glColor4f(.04, .06, .09, .75)
        glVertexPointer(3, GL_FLOAT, 0, edges); glDrawArrays(GL_LINES, 0, len(edges))
        if top is not None and mode != 2:
            glLineWidth(1.2); glColor4f(1, 1, 1, .35)
            glVertexPointer(3, GL_FLOAT, 0, top); glDrawArrays(GL_LINES, 0, len(top))
    glDisableClientState(GL_VERTEX_ARRAY); glDisable(GL_LINE_SMOOTH); glDisable(GL_BLEND); glLineWidth(1)
