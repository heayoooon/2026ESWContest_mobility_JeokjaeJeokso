#!/usr/bin/env python3
"""센서 파이프라인: 수집 -> 판정 -> 기록. 파이에서 센서를 돌릴 때는 이것 하나를 실행한다.

    python3 pipeline.py                      # 파이. Ctrl+C 까지
    LOG_SECONDS=60 python3 pipeline.py       # 60초
    python pipeline.py --sim                 # PC(VS Code). 파이 없이 가짜 센서로

    acquisition/source.py     센서 -> Frame (약 10Hz, acquisition/frame.py)
    analysis/analyzer.py      Frame -> Result (총하중, 무게중심, 판정 4개 -> 위험도 정상/주의/위험)
    acquisition/log_xlsx.py   Frame + Result -> CSV / 엑셀 (판정 열 + '판정' 시트)
    acquisition/log_camera.py 트윈 노드(D435i) UDP 패킷 -> <이름>_camera 엑셀 (따로, 별도 프로세스)

핀/라벨/역할/판정 기준은 전부 환경변수다 (acquisition/config.py). 예) 바닥 4모서리 + 벨트 2개
    HX_DT=5,6,13,19,26,20 HX_LABELS=FL,FR,RL,RR,T1,T2 python3 pipeline.py

위험도는 화면 맨 앞 [정상]/[주의]/[위험] 으로 보이고, 판정 하나의 알림 레벨이 바뀔 때마다 한 줄씩 찍힌다.
    적재 (LOAD_MAX_KG) · 무게중심 구역 · 화물 이동 (h 추정 + IMU 예측) · 벨트 장력 (N쪽 / S쪽 무게)
"""
import os
import signal
import sys
import time

from acquisition.config import Config
from acquisition.console import say, status
from acquisition.log_camera import CameraLog
from acquisition.log_xlsx import XlsxSink
from acquisition.source import FrameSource
from analysis.analyzer import Analyzer
from analysis.risk import CAUTION, DANGER, OK
from analysis.risk import name as risk_name

STATUS_EVERY = 0.5          # 화면 갱신(초)
SIM_BED = ("250", "392")    # --sim 에서 BED_L/BED_W 를 안 주면 (env.sh 실측값과 같게)


def risk_head(result):
    """[위험 장력 T1 / 주의 무게중심] 처럼 레벨별로 어떤 판정이 걸렸는지."""
    if result.level is None:
        return "[판정 불가]"
    parts = []
    for lv in (DANGER, CAUTION):
        names = [ch.name for ch in result.checks if ch.level == lv]
        if names:
            parts.append("%s %s" % (risk_name(lv), ",".join(names)))
    return "[%s]" % (" / ".join(parts) or risk_name(OK))


def status_text(frame, result, cam_rows):
    show = [risk_head(result)]
    if result.load_kg is not None:
        show.append("적재 %.1fkg" % result.load_kg)
    elif result.total_g is not None:
        show.append("하중 %.0fg" % result.total_g)
    c = result.cog
    if c:
        show.append("CoG %.0f,%.0f%s" % (c.x, c.y, c.unit))
    tr = result.transfer
    if tr and tr.ky is not None and tr.kx is not None:
        show.append("h 앞뒤%.0f 좌우%.0f%s" % (abs(tr.ky), abs(tr.kx), "mm" if c and c.unit == "mm" else "%/G"))
    tension = {l: v for l, v in frame.tension_n.items() if v is not None}
    show += ["%s %.1fN%s" % (l, v, "(>=%.1f)" % result.tension_min[l] if result.tension_min.get(l) else "")
             for l, v in tension.items()]
    if result.total_g is None and not tension:  # 보정이 없으면 raw 라도
        show += ["%s=%d" % (l, v) for l, v in frame.raw.items() if v is not None]
    for l, r in frame.imu.items():              # 이번 구간에 못 읽은 IMU 는 - 로
        show.append("%s ax%+.2f ay%+.2f" % (l, r.ax, r.ay) if r else "%s -" % l)
    if cam_rows:
        show.append("카메라 %d" % cam_rows)
    return "  ".join(show)


def main():
    args = sys.argv[1:]
    if any(a != "--sim" for a in args):
        sys.exit(__doc__)
    is_sim = "--sim" in args
    if is_sim and not os.environ.get("HX_LABELS") and not os.environ.get("HX_DT"):
        os.environ["HX_LABELS"] = "FL,FR,RL,RR,T1,T2"   # 가짜 센서는 최종 배치(4모서리+벨트2)로
    if is_sim and not os.environ.get("BED_L") and not os.environ.get("BED_W"):
        os.environ["BED_L"], os.environ["BED_W"] = SIM_BED  # 무게중심 · h 를 mm 로

    cfg = Config()
    name = cfg.log_name
    notes = []
    if is_sim:
        if not os.environ.get("LOG_NAME"):
            name = "sim_" + name
        say("*** 시뮬레이션: 가짜 센서 값이다. 파이에서 실측할 때는 --sim 을 빼라 ***")
        notes.append("시뮬레이션 데이터 (가짜 센서). 실측 아님.")
    say(cfg.describe())
    say(cfg.describe_judge())
    say()

    cam = CameraLog(cfg, os.path.join(cfg.log_dir, name + "_camera.csv"),
                    os.path.join(cfg.log_dir, name + "_camera.xlsx"), sim=is_sim)
    cam.start()                                 # 센서보다 먼저 (수신 프로세스가 뜨는 ~1초를 기록 전에)
    src = FrameSource(cfg, sim=is_sim)
    if is_sim:
        cam.start_sim(src.hx.scn.t0)            # 가짜 카메라를 가짜 센서와 같은 시각표로
    os.makedirs(cfg.log_dir, exist_ok=True)
    sink = XlsxSink(cfg, os.path.join(cfg.log_dir, name + ".csv"),
                    os.path.join(cfg.log_dir, name + ".xlsx"))

    # Ctrl+C / kill / SSH 끊김 모두 '멈추고 엑셀 만들기'. nohup 처럼 무시로 걸려 있으면 그대로 둔다
    for sig_name in ("SIGINT", "SIGTERM", "SIGHUP"):
        sig = getattr(signal, sig_name, None)
        if sig is not None and signal.getsignal(sig) is not signal.SIG_IGN:
            signal.signal(sig, lambda *_: src.stop())

    analyze = Analyzer(cfg)
    limit_ns = int(cfg.log_seconds * 1e9) if cfg.log_seconds > 0 else None
    last_status = 0.0
    run_notes = []
    say("\n기록 시작 -> %s\nCtrl+C 로 끝내면 엑셀을 만든다.\n" % sink.csv_path)

    try:
        for f in src.frames():
            if sink.rows == 0:                  # 카메라 행의 경과(초)를 센서 엑셀과 같은 0초에 맞춘다
                cam.set_t0(src.t0_ns, src.t0_wall)
            r = analyze(f)
            sink.write(f, r)
            for ch, old in r.changed:            # 판정 하나의 알림 레벨이 바뀐 순간 한 줄씩
                say("\n%6.1fs [%s] %s: %s  (%s -> %s)" % (f.t_s, risk_name(ch.level), ch.name, ch.text,
                                                          risk_name(old), risk_name(ch.level)))

            now = time.monotonic()
            if now - last_status >= STATUS_EVERY:
                last_status = now
                status("%6.1fs %6d행  %s%s" % (f.t_s, sink.rows, status_text(f, r, cam.rows),
                                               "  타임아웃 %d" % src.timeouts if src.timeouts else ""))
            if limit_ns and time.monotonic_ns() - src.t0_ns >= limit_ns:
                break
    except Exception as e:                      # 예상 못 한 오류라도 지금까지 것은 엑셀로 남긴다
        say("\n! 오류로 기록 중단: %s: %s" % (type(e).__name__, e))
        run_notes.append("오류로 기록 중단: %s: %s" % (type(e).__name__, e))
    finally:
        src.close()

    say("\n\n정리 중...")
    cam_notes = cam.close()
    return 0 if sink.close(src, notes + src.notes + run_notes + cam_notes) else 1


if __name__ == "__main__":
    sys.exit(main())
