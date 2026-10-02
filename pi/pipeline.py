#!/usr/bin/env python3
"""센서 파이프라인: 수집 -> 판정 -> 기록. 파이에서 센서를 돌릴 때는 이것 하나를 실행한다.

    python3 pipeline.py                      # 파이. Ctrl+C 까지
    LOG_SECONDS=60 python3 pipeline.py       # 60초
    python pipeline.py --sim                 # PC(VS Code). 파이 없이 가짜 센서로

    acquisition/source.py     센서 -> Frame (약 10Hz, acquisition/frame.py)
    analysis/analyzer.py      Frame -> Result (총하중, 무게중심. analysis/cog.py)
    acquisition/log_xlsx.py   Frame + Result -> CSV / 엑셀

핀/라벨/역할은 전부 환경변수다 (acquisition/config.py). 예) 바닥 4모서리 + 벨트 2개
    HX_DT=5,6,13,19,26,20 HX_LABELS=FL,FR,RL,RR,T1,T2 python3 pipeline.py
"""
import os
import signal
import sys
import time

from acquisition.config import Config
from acquisition.console import say, status
from acquisition.log_xlsx import XlsxSink
from acquisition.source import FrameSource
from analysis.analyzer import Analyzer

STATUS_EVERY = 0.5          # 화면 갱신(초)


def status_text(frame, result):
    show = []
    if result.total_g is not None:
        show.append("하중 %.0fg" % result.total_g)
    c = result.cog
    if c:
        show.append("CoG %.0f,%.0f%s" % (c.x, c.y, c.unit))
    tension = {l: v for l, v in frame.tension_n.items() if v is not None}
    show += ["%s %.1fN" % (l, v) for l, v in tension.items()]
    if result.total_g is None and not tension:  # 보정이 없으면 raw 라도
        show += ["%s=%d" % (l, v) for l, v in frame.raw.items() if v is not None]
    for l, r in frame.imu.items():
        if r:
            show.append("%s ax%+.2f ay%+.2f" % (l, r.ax, r.ay))
            break
    return "  ".join(show)


def main():
    args = sys.argv[1:]
    if any(a != "--sim" for a in args):
        sys.exit(__doc__)
    is_sim = "--sim" in args
    if is_sim and not os.environ.get("HX_LABELS") and not os.environ.get("HX_DT"):
        os.environ["HX_LABELS"] = "FL,FR,RL,RR,T1,T2"   # 가짜 센서는 최종 배치(4모서리+벨트2)로

    cfg = Config()
    name = cfg.log_name
    notes = []
    if is_sim:
        if not os.environ.get("LOG_NAME"):
            name = "sim_" + name
        say("*** 시뮬레이션: 가짜 센서 값이다. 파이에서 실측할 때는 --sim 을 빼라 ***")
        notes.append("시뮬레이션 데이터 (가짜 센서). 실측 아님.")
    say(cfg.describe())
    say()

    src = FrameSource(cfg, sim=is_sim)
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
            r = analyze(f)
            sink.write(f, r)

            now = time.monotonic()
            if now - last_status >= STATUS_EVERY:
                last_status = now
                status("%6.1fs %6d행  %s%s" % (f.t_s, sink.rows, status_text(f, r),
                                               "  타임아웃 %d" % src.timeouts if src.timeouts else ""))
            if limit_ns and time.monotonic_ns() - src.t0_ns >= limit_ns:
                break
    except Exception as e:                      # 예상 못 한 오류라도 지금까지 것은 엑셀로 남긴다
        say("\n! 오류로 기록 중단: %s: %s" % (type(e).__name__, e))
        run_notes.append("오류로 기록 중단: %s: %s" % (type(e).__name__, e))
    finally:
        src.close()

    say("\n\n정리 중...")
    return 0 if sink.close(src, notes + src.notes + run_notes) else 1


if __name__ == "__main__":
    sys.exit(main())
