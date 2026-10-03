"""카메라 기록 — 파이프라인(pi/pipeline.py)의 두 번째 엑셀.

D435i 트윈 노드(twin/pi_twin_node.py)는 센서 Frame 에 섞지 않는다. 별도 프로세스이고 박자도
다르다(높이맵 15~30fps, HX711 10Hz). 대신 pipeline.py 가 기록하는 동안 여기서 트윈 노드의 UDP
패킷을 받아 **패킷 하나 = CSV 한 행** 으로 쓰고, 끝날 때 센서 엑셀 옆에 카메라 엑셀을 만든다.

    python3 twin/pi_twin_node.py                    # 카메라가 이 파이에 있으면 (기본 127.0.0.1)
    python3 twin/pi_twin_node.py <파이 주소>,<PC>    # 다른 노드에 있으면 이 파이로 (쉼표로 여러 곳)
    LOG_SECONDS=60 python3 pipeline.py               # -> <이름>.xlsx + <이름>_camera.xlsx

    측정 시트     한 행 = 패킷 하나의 헤더 값 전부 (stats 요약, 카메라 IMU, sensors, seq, 카메라 프레임 시각)
    트윈 기준 시트 격자(nx, ny, 칸, 원점)와 설치 보정값(기준면 높이, 월드 회전 R, intrinsics, 녹화 파일 이름 ...)
컬러 · depth 원본은 여기 말고 트윈 노드가 <시각>_d435i.db3 로 녹화한다. 카메라 프레임 시각(ms) 열이
그 파일의 depth 프레임 타임스탬프와 같아서 이걸로 두 파일을 맞춘다.
시각 · 경과(초)는 **이 파이가 받은 시각** 이고, 경과(초) 0 은 센서 엑셀과 같은 기록 시작이다.
그래서 카메라가 다른 장비에 있어도 두 장비 시계 차이가 끼지 않는다 (차이는 지연(ms) 열에만 보인다).

만들어지는 파일 (LOG_DIR/) — 패킷이 하나도 안 오면 안 만든다
    <이름>_camera.csv    받으면서 1초마다 디스크에 쓴다
    <이름>_camera.xlsx   끝날 때 CSV 로 만든다. 측정 · 요약 · 트윈 기준 시트

CSV 만 남았을 때 엑셀 다시 만들기 (트윈 기준 시트는 빠진다):
    python3 -c "from acquisition import log_camera; log_camera.build_xlsx('X_camera.csv', 'X_camera.xlsx')"

**왜 프로세스를 나누는가.** HX711 은 SCK 가 60us 넘게 HIGH 면 샘플이 깨진다. 같은 프로세스의 스레드가
패킷을 풀면 GIL 때문에 클럭 도중에 선점당할 수 있다 (acquisition/README.md 설계 메모와 같은 이유).
"""
import csv
import json
import multiprocessing
import os
import signal
import socket
import struct
import threading
import time

from acquisition.console import say
from acquisition.log_xlsx import FILL, PIP_HINT, Stat, _round, stamp
from acquisition.log_xlsx import build_xlsx as _build_xlsx

FLUSH_EVERY = 1.0           # CSV 를 디스크까지 내리는 주기(초)
START_TIMEOUT = 15.0        # 수신 프로세스가 포트를 열 때까지 기다리는 한계(초)
TILT_WARN_DEG = 2.0         # 카메라가 트윈 노드 시작 때보다 이만큼 기울면 moved (높이맵 기준이 틀어짐)

# (CSV 열 이름, 엑셀 머리글, 머리글 색 묶음). 가운데 7개는 패킷 헤더 stats 의 키 그대로다
COLS = (("time", "시각", "time"), ("t_s", "경과(초)", "time"),
        ("valid_pct", "유효 칸(%)", "load"), ("max_mm", "최고 높이(mm)", "load"),
        ("mean_mm", "평균 높이(mm)", "load"), ("area_m2", "적재 면적(m²)", "load"),
        ("volume_l", "부피(L)", "load"), ("cx_m", "부피중심 X(m)", "load"),
        ("cy_m", "부피중심 Y(m)", "load"),
        ("tilt_deg", "카메라 기울기 변화(°)", "imu"),
        ("acc_x", "카메라 가속도 x(m/s²)", "imu"), ("acc_y", "카메라 가속도 y(m/s²)", "imu"),
        ("acc_z", "카메라 가속도 z(m/s²)", "imu"),
        ("seq", "seq", "diag"), ("frame_ts", "카메라 프레임 시각(ms)", "diag"),
        ("lag_ms", "지연(ms)", "diag"), ("flags", "flags", "diag"))
STATS_KEYS = [c[0] for c in COLS[2:9]]
N_MAIN = [c[0] for c in COLS].index("seq")     # 트윈 노드 sensors 열은 이 자리(진단 열 앞)에 끼운다
SENSOR = "sensor:"          # sensors 열 이름 앞머리. 키는 첫 패킷에서 정한다
STYLE = {name: (title, FILL[grp]) for name, title, grp in COLS}
FLAG_NOTE = ("flags: gap:N = 트윈 노드 seq 가 N 개 건너뜀(놓친 패킷), restart = 트윈 노드가 다시 시작됨"
             "(격자 · 기준면이 새로 잡힘), moved = 카메라가 시작 때보다 %g° 넘게 기울어 높이맵을 못 믿음"
             "(트윈 노드 재시작), no_stats = stats 없는 예전 트윈 노드" % TILT_WARN_DEG)


def _style(h):
    if h.startswith(SENSOR):
        return "%s (트윈 노드)" % h[len(SENSOR):], FILL["load"]
    return STYLE.get(h, (h, FILL["diag"]))


def build_xlsx(csv_path, xlsx_path, info=(), notes=(), setups=()):
    """카메라 CSV -> 엑셀. 기록 도중 죽어서 CSV 만 남았을 때도 이걸로 다시 만든다.
    setups = 수신 프로세스가 모은 트윈 노드 설정 [(경과(초), seq, grid, calib)]. 있으면 트윈 기준 시트를 붙인다."""
    extra = None
    if setups:
        def extra(wb, head_cell):
            _setup_sheet(wb, head_cell, setups)
    return _build_xlsx(csv_path, xlsx_path, list(info), list(notes), style=_style, extra=extra)


def _setup_sheet(wb, head_cell, setups):
    """트윈 기준 시트: 높이맵을 3D 로 다시 세울 때 필요한 격자 · 설치 보정값.
    트윈 노드가 기록 도중 다시 시작하면 (격자 · 기준면이 새로 잡혀서) 블록이 하나 더 생긴다."""
    ws = wb.create_sheet("트윈 기준")
    ws.column_dimensions["A"].width = 26
    for col in "BCD":
        ws.column_dimensions[col].width = 13
    ws.column_dimensions["E"].width = 80
    for k, (t_s, seq, g, c) in enumerate(setups, 1):
        title = "트윈 노드 설정" + (" %d" % k if len(setups) > 1 else "")
        ws.append([head_cell(ws, title), "경과 %.3f초 (seq %s) 부터" % (t_s, seq)])
        rows = [("격자 nx, ny (칸)", g.get("nx"), g.get("ny"), None, "높이맵 한 장 = ny 행 x nx 열 (트윈 노드 패킷)"),
                ("칸 크기 (m)", g.get("cell")),
                ("격자 원점 x0, y0 (m)", g.get("x0"), g.get("y0"), None,
                 "칸 (ix, iy) 중심 = (x0 + (ix + 0.5)·칸, y0 + (iy + 0.5)·칸). 월드 좌표"),
                ("높이 단위", g.get("unit"), None, None,
                 "기준면 = 0. depth 없는 칸 = %s" % g.get("nodata"))]
        if c:
            R = c.get("R") or [[None] * 3] * 3
            intr = c.get("intr") or {}
            rows += [("기준면 높이 ref_h (m)", c.get("ref_h"), None, None,
                      "월드 z. 칸 윗면의 월드 z = ref_h + 높이(mm) / 1000"),
                     ("카메라 위치 (m)", 0, 0, 0, "월드 원점 = 카메라. 기준면에서 카메라까지 높이 = -ref_h"),
                     ("월드 회전 R  1행 (x)", *R[0], "월드 = R · 카메라 좌표. 카메라 좌표 = x 오른쪽, y 아래, z 앞"),
                     ("월드 회전 R  2행 (y)", *R[1]),
                     ("월드 회전 R  3행 (위)", *R[2], "중력 반대 방향"),
                     ("설치 때 가속도 (m/s²)", *(c.get("accel0") or [None] * 3),
                      "카메라 좌표. R 을 만든 값 (정지 상태에선 위쪽을 가리킨다)"),
                     ("intrinsics fx, fy (px)", intr.get("fx"), intr.get("fy"), None, "필터(decimation) 후 depth"),
                     ("intrinsics ppx, ppy (px)", intr.get("ppx"), intr.get("ppy")),
                     ("depth 해상도 (px)", intr.get("width"), intr.get("height")),
                     ("depth_scale (m / 단위)", c.get("depth_scale")),
                     ("최대 거리 (m)", c.get("max_range"), None, None, "이보다 먼 depth 는 버린다"),
                     ("depth 스트림", c.get("stream")),
                     ("필터", c.get("filters")),
                     ("카메라", c.get("camera")),
                     ("녹화 파일", c.get("bag") or "없음", None, None,
                      "트윈 노드가 남긴 컬러 + depth 원본 (.db3). 카메라 프레임 시각(ms) = 그 파일의 depth 타임스탬프")]
        else:
            rows.append(("보정값", "없음", None, None, "예전 트윈 노드 (패킷 헤더에 calib 없음)"))
        for r in rows:
            ws.append(list(r))
        ws.append([])


# --------------------------------------------------------------------------
# pipeline.py 쪽
# --------------------------------------------------------------------------
class CameraLog:
    """start() -> 센서 기록이 시작되면 set_t0() -> close() 가 카메라 엑셀을 만든다.
    CAM_PORT=0 이거나 포트를 못 열면 아무것도 안 하고 메모만 남긴다 (센서 기록은 그대로 간다).

    start() 는 FrameSource 보다 먼저 부른다. 수신 프로세스가 뜨는 동안(~1초) 센서 시작이 밀리지 않고,
    센서 쪽 우선순위(SCHED_FIFO)도 물려받지 않는다."""

    def __init__(self, cfg, csv_path, xlsx_path, sim=False):
        self.port = cfg.cam_port
        self.csv_path, self.xlsx_path = csv_path, xlsx_path
        self.sim = sim
        self.proc = None
        self.notes = []

    def start(self):
        if not self.port:
            return
        ctx = multiprocessing.get_context("spawn")  # PC(Windows) 와 파이에서 같은 방식으로
        self._t0 = ctx.Array("d", 2)                # [기록 시작 monotonic ns, 벽시계]. 0 이면 아직
        self._count = ctx.Value("i", 0)
        self._stop = ctx.Event()
        self._conn, child = ctx.Pipe()
        self.proc = ctx.Process(target=_run, name="camera-log", daemon=True,
                                args=(self.port, self.csv_path, self._t0, self._count,
                                      self._stop, child, self.sim))
        self.proc.start()
        child.close()
        kind, msg = self._recv(START_TIMEOUT) or ("error", "수신 프로세스가 %g초 안에 응답 없음" % START_TIMEOUT)
        if kind != "ready":
            say("! 카메라 기록 안 함 (UDP %d 못 엶: %s)" % (self.port, msg))
            self.notes.append("카메라 기록 안 함: UDP %d 못 엶 (%s)" % (self.port, msg))
            self._halt()
            return
        say("카메라  UDP %d 대기 -> %s%s" % (self.port, os.path.basename(self.xlsx_path),
                                           ("  (%s)" % msg) if msg else ""))

    @property
    def rows(self):
        return self._count.value if self.proc else 0

    def start_sim(self, scn_t0):
        """시뮬레이션: 가짜 카메라를 가짜 센서와 같은 시각표(Scenario.t0)로 띄운다."""
        if self.proc and self.sim:
            self._conn.send(("sim", scn_t0))

    def set_t0(self, t0_ns, t0_wall):
        """센서 기록 시작 시각. 이 뒤에 받은 패킷만 적고, 경과(초)를 센서 엑셀과 같은 0초에 맞춘다."""
        if self.proc:
            with self._t0.get_lock():
                self._t0[0], self._t0[1] = float(t0_ns), t0_wall

    def close(self):
        """수신을 멈추고 카메라 엑셀을 만든다. 돌려주는 값은 센서 엑셀 요약 시트에 붙일 메모."""
        if self.proc is None:
            return self.notes
        self._stop.set()
        got = self._recv(5)
        sm = got[1] if got and got[0] == "done" else None
        self._halt()
        if sm is None and not os.path.exists(self.csv_path):
            sm = {"rows": 0}
        if sm is not None and not sm["rows"]:
            say("카메라  패킷 없음 -> 카메라 엑셀 안 만듦 (twin/pi_twin_node.py 가 이 장비 UDP %d 로 보내는지)"
                % self.port)
            if sm.get("bad"):
                say("        트윈 노드 패킷이 아닌 UDP %d개는 버렸다 (ESP32 IMU 등 같은 포트를 쓰는 것)"
                    % sm["bad"])
            self.notes.append("카메라 패킷 없음 (UDP %d)" % self.port)
            return self.notes

        info, notes = summary_info(self.port, self.sim, sm)
        try:
            import openpyxl  # noqa: F401
            has_xlsx = True
        except ImportError:
            has_xlsx = False
        if has_xlsx:
            say("카메라 엑셀 만드는 중...")
            n = build_xlsx(self.csv_path, self.xlsx_path, info, notes, sm["setups"] if sm else ())
            say("카메라 엑셀: %s" % self.xlsx_path)
        else:
            n = sm["rows"] if sm else None
            say("! openpyxl 이 없어 카메라 엑셀은 못 만들었다.\n  %s" % PIP_HINT)
        say("카메라 CSV : %s" % self.csv_path)
        if sm:
            say("카메라 %d프레임  %.1ffps  놓침 %d  flags %d"
                % (sm["rows"], sm["fps"], sm["lost"], sm["flagged"]))
        self.notes.append("카메라: %s (%s프레임) — 경과(초)가 같은 0초"
                          % (os.path.basename(self.xlsx_path if has_xlsx else self.csv_path),
                             n if n is not None else "?"))
        return self.notes

    def _recv(self, timeout):
        try:
            return self._conn.recv() if self._conn.poll(timeout) else None
        except (EOFError, OSError):             # 수신 프로세스가 먼저 죽었다
            return None

    def _halt(self):
        self._stop.set()
        self.proc.join(3)
        if self.proc.is_alive():
            self.proc.terminate()
            self.proc.join(1)
        self.proc = None


def summary_info(port, is_sim, sm):
    """요약 시트 위쪽 (항목, 값) 과 메모. sm 은 수신 프로세스가 돌려준 통계 (죽었으면 None)."""
    info = [("모드", "시뮬레이션 (가짜 카메라, sensors/sim.py)" if is_sim else "실측 (twin/pi_twin_node.py)"),
            ("받은 포트", "UDP %d" % port)]
    notes = [FLAG_NOTE,
             "시각 · 경과(초) = 이 장비가 받은 시각. 경과(초) 0 = 센서 엑셀과 같은 기록 시작",
             "카메라 가속도 = D435i 내장 IMU, 카메라 좌표 (x 오른쪽, y 아래, z 앞). 정지 상태에선 위쪽을 가리킨다. "
             "카메라 프레임 시각 = RealSense 프레임 타임스탬프(ms)",
             "부피중심 = 칸 높이로 가중한 중심 (화물 밀도가 고르다고 보면 형상 무게중심). "
             "좌표는 카메라 격자 기준이라 로드셀 무게중심 좌표와 원점 · 방향이 다르다",
             "컬러 · depth 원본은 트윈 노드의 녹화 파일(<시각>_d435i.db3, 트윈 기준 시트에 이름)에 있다. "
             "카메라 프레임 시각(ms) 열 = 그 파일의 depth 프레임 타임스탬프"]
    if sm is None:
        notes.append("수신 프로세스가 정상 종료하지 못해 통계 없이 CSV 로만 만들었다")
        return info, notes
    info += [("보낸 곳", ", ".join(sm["senders"]) or "-"),
             ("받은 프레임", sm["rows"]),
             ("놓친 프레임 (seq 건너뜀)", sm["lost"]),
             ("트윈 노드 재시작", sm["restarts"]),
             ("flags 있는 행", sm["flagged"])]
    if sm["bad"]:
        info.append(("버린 UDP (트윈 패킷 아님)", sm["bad"]))
    if sm["lag"]:
        info.append(("지연(ms) 평균 / 최대", "%.1f / %.1f   (보낸 시각 -> 받은 시각. 다른 장비면 시계 차이 포함)"
                     % (sm["lag"][0], sm["lag"][1])))
    g = sm["grid"]
    if g:
        info += [("", ""),
                 ("높이맵 격자", "%d x %d 칸, %g cm" % (g["nx"], g["ny"], g["cell"] * 100)),
                 ("격자 원점 (x0, y0)", "%g, %g m" % (g["x0"], g["y0"]))]
    if sm["occ_mm"] is not None:
        info.append(("화물 칸 기준", "기준면 + %d mm 이상 (기준면 = 트윈 노드 시작 때 가장 흔한 높이)"
                     % sm["occ_mm"]))
    return info, notes


# --------------------------------------------------------------------------
# 수신 프로세스
# --------------------------------------------------------------------------
class _Writer:
    """트윈 패킷 -> CSV 한 행. 첫 행을 쓸 때 파일을 만든다 (패킷이 안 오면 파일도 없다).
    트윈 노드 설정(격자 · 보정값)은 처음과 바뀔 때만 모아 둔다 (트윈 기준 시트)."""

    def __init__(self, csv_path):
        self.csv_path = csv_path
        self.f = self.w = None
        self.setups, self._setup_key = [], None
        self.sensor_keys = []
        self.rows = self.bad = self.lost = self.restarts = self.flagged = 0
        self.last_seq = self.grid = self.occ_mm = None
        self.first_t = self.last_t = None
        self.senders = []
        self.lag = Stat()
        self.last_flush = 0.0

    def add(self, pkt, addr, t_s, wall):
        try:
            hl = struct.unpack("!I", pkt[:4])[0]
            hdr = json.loads(pkt[4:4 + hl])
            seq, sent = int(hdr["seq"]), float(hdr["t"])
        except Exception:                       # 트윈 노드 패킷이 아니다
            self.bad += 1
            return
        flags = []
        if self.last_seq is not None:
            if seq <= self.last_seq:
                flags.append("restart")
                self.restarts += 1
            elif seq > self.last_seq + 1:
                flags.append("gap:%d" % (seq - self.last_seq - 1))
                self.lost += seq - self.last_seq - 1
        self.last_seq = seq
        st = hdr.get("stats")
        if st is None:
            flags.append("no_stats")
            st = {}
        imu = hdr.get("imu") or {}
        tilt = imu.get("tilt_change_deg")
        acc = (list(imu.get("accel") or ()) + [None] * 3)[:3]
        sensors = hdr.get("sensors") or {}
        if tilt is not None and tilt > TILT_WARN_DEG:
            flags.append("moved")
        lag = (wall - sent) * 1000
        self.lag.add(lag)
        self.grid = hdr.get("grid") or self.grid
        self.occ_mm = st.get("occ_mm", self.occ_mm)
        if addr[0] not in self.senders:
            self.senders.append(addr[0])
        key = json.dumps([hdr.get("grid"), hdr.get("calib")], sort_keys=True)
        if key != self._setup_key:              # 처음 · 트윈 노드 재시작
            self._setup_key = key
            self.setups.append((t_s, seq, hdr.get("grid") or {}, hdr.get("calib")))

        if self.w is None:
            self.f = open(self.csv_path, "w", newline="", encoding="utf-8-sig")
            self.w = csv.writer(self.f, lineterminator="\n")
            self.sensor_keys = list(sensors)
            names = [c[0] for c in COLS]
            self.w.writerow(names[:N_MAIN] + [SENSOR + k for k in self.sensor_keys] + names[N_MAIN:])
        self.w.writerow([stamp(wall), round(t_s, 3)] + [st.get(k) for k in STATS_KEYS]
                        + [_round(tilt, 2)] + [_round(a, 3) for a in acc]
                        + [sensors.get(k) for k in self.sensor_keys]
                        + [seq, _round(hdr.get("frame_ts"), 3), round(lag, 1), "|".join(flags)])
        self.rows += 1
        self.flagged += bool(flags)
        self.first_t = t_s if self.first_t is None else self.first_t
        self.last_t = t_s
        self.flush()

    def flush(self, force=False):
        now = time.monotonic()
        if self.f and (force or now - self.last_flush >= FLUSH_EVERY):
            self.last_flush = now
            self.f.flush()
            os.fsync(self.f.fileno())           # 전원이 나가도 1초 치만 잃게

    def close(self):
        if self.f:
            self.flush(force=True)
            self.f.close()

    def summary(self):
        span = (self.last_t - self.first_t) if self.rows > 1 else 0.0
        return {"rows": self.rows, "bad": self.bad, "lost": self.lost, "restarts": self.restarts,
                "flagged": self.flagged, "senders": self.senders, "grid": self.grid,
                "occ_mm": self.occ_mm, "setups": self.setups, "fps": (self.rows - 1) / span if span > 0 else 0.0,
                "lag": (self.lag.mean, self.lag.max) if self.lag.n else None}


def _sim_camera(port, scn_t0, stop):
    """가짜 카메라 (sensors/sim.py SimCamera). 실측 트윈 노드처럼 127.0.0.1 로 패킷을 보낸다."""
    from sensors import sim
    cam = sim.SimCamera(sim.Scenario(scn_t0))

    def loop():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        nxt = time.monotonic()
        while not stop.is_set():
            s.sendto(cam.packet(), ("127.0.0.1", port))
            nxt += 1.0 / sim.CAM_FPS
            time.sleep(max(0.0, nxt - time.monotonic()))
        s.close()

    threading.Thread(target=loop, name="sim-camera", daemon=True).start()


def _run(port, csv_path, t0, count, stop, conn, sim):
    """수신 프로세스 본체. t0 = [기록 시작 monotonic ns, 벽시계] (공유), count = 쓴 행 수 (공유)."""
    for name in ("SIGINT", "SIGHUP"):           # Ctrl+C · SSH 끊김은 부모가 받아 stop 으로 알려준다
        sig = getattr(signal, name, None)
        if sig is not None:
            signal.signal(sig, signal.SIG_IGN)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("0.0.0.0", port))
    except OSError as e:
        conn.send(("error", "%s (twin/twin_receiver.py 같은 다른 수신기가 이미 쓰는 중?)" % e))
        return
    sock.settimeout(0.2)
    note = None
    if sim:
        try:
            import numpy  # noqa: F401  가짜 카메라(SimCamera)가 쓴다
            from sensors.sim import CAM_FPS
            note = "가짜 카메라 %dfps" % CAM_FPS
        except ImportError as e:
            note = "가짜 카메라 못 띄움: %s (pip install numpy)" % e
            sim = False
    conn.send(("ready", note))

    parent = multiprocessing.parent_process()
    w = _Writer(csv_path)
    try:
        while not stop.is_set():
            if parent is not None and not parent.is_alive():
                break                           # 부모가 죽었다 (kill -9 등). CSV 까지만 남긴다
            if sim and conn.poll():             # 부모가 start_sim() 으로 시각표를 보냈다
                try:
                    kind, scn_t0 = conn.recv()
                except EOFError:                # 그 사이 부모가 죽었다
                    break
                if kind == "sim":
                    _sim_camera(port, scn_t0, stop)
                    sim = False
            try:
                pkt, addr = sock.recvfrom(65535)
            except socket.timeout:
                w.flush()
                continue
            now_ns = time.monotonic_ns()
            with t0.get_lock():
                start_ns, start_wall = t0[0], t0[1]
            if not start_ns or now_ns < start_ns:
                continue                        # 센서 기록 시작 전
            t_s = (now_ns - start_ns) / 1e9
            w.add(pkt, addr, t_s, start_wall + t_s)
            count.value = w.rows
    finally:
        w.close()
        sock.close()
        try:
            conn.send(("done", w.summary()))
        except (OSError, EOFError):
            pass
