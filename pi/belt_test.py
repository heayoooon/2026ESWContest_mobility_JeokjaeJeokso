#!/usr/bin/env python3
"""벨트 누름 추정 실물 테스트: 짐을 쌓고 벨트를 조이면, 벨트마다 추정한 k · 누름 자리 x, y 를 출력하고 끝난다.
주행은 안 한다. 추정은 판정과 같은 코드(analysis/belt_fit.py)다.

    source acquisition/env.sh && python3 belt_test.py          # 파이. 다 추정되고 5초 그대로면 끝
    python3 belt_test.py --wait                                # 자동으로 안 끝낸다. Enter 나 Ctrl+C 로 끝
    python3 belt_test.py --replay logs/<이름>.csv               # 기록(pipeline · live 의 CSV)으로 다시 돌린다

순서 (화면에도 나온다)
    1. 짐을 다 쌓고 벨트는 푼 채 손을 떼고 3초 기다린다       -> 기준 (장력 0 일 때 무게 · 무게중심)
    2. 앞 벨트를 조이고 손을 뗀다 (0.3초 넘게 가만히)          -> 앞 벨트 추정
    3. 뒤 벨트를 조이고 손을 뗀다                               -> 뒤 벨트 추정
    4. (선택) 한쪽 벨트를 반쯤 풀고 손을 뗀다                   -> 검증. 장력이 준 구간은 배우지 않아서 진짜 시험이 된다
    5. 5초 그대로 두면 끝난다
벨트는 한 번에 8N 넘게 조여야 배운다 (STEP_N). 15N 이상 조이면 더 정확하다.

끝나면 멈춘 구간마다 장력 0 기준 대비 화물 무게 · 무게중심 오차를 보정 전 / 시작값 고정 / 추정값으로 보여 준다.
'학습' 구간은 추정에 쓴 구간이라 추정값 오차가 좋게 나오고, '검증' 구간이 실제 성능이다.
기록은 pipeline.py 와 같은 CSV / 엑셀로 LOG_DIR 에 belt_<시각> 이름으로 남고, 결과는 <이름>_belt.txt 에도 적는다.
"""
import csv
import math
import os
import signal
import sys
import threading
import time

from acquisition.config import Config
from acquisition.console import say, status
from acquisition.frame import Frame, ImuReading
from acquisition.log_xlsx import XlsxSink
from acquisition.source import FrameSource
from analysis import belt_fit
from analysis.analyzer import Analyzer

DONE_S = 5.0                # 초. 벨트가 다 추정된 뒤 이만큼 멈춰 있으면 끝낸다
STATUS_EVERY = 0.5          # 화면 갱신(초)


class CsvSource:
    """기록 CSV -> Frame. pipeline.py (log_xlsx) · live (sensor_feed) CSV 둘 다 <라벨>_g, <라벨>_N, <IMU>_ax_G 열을 쓴다."""

    def __init__(self, cfg, path):
        self.cfg, self.path = cfg, path
        self.notes, self.timeouts, self.stopped = [], 0, False

    def frames(self):
        weight, tension, _ = self.cfg.roles()
        num = lambda r, k: float(r[k]) if r.get(k) not in ("", None) else None     # noqa: E731
        t0 = None
        with open(self.path, encoding="utf-8-sig") as f:
            for i, r in enumerate(csv.DictReader(f)):
                t0 = num(r, "t_s") if t0 is None else t0
                if self.stopped:
                    return
                imu = {}
                for l in self.cfg.imu_labels:
                    v = [num(r, "%s_%s" % (l, k)) for k in ("ax_G", "ay_G", "az_G", "gx_dps", "gy_dps", "gz_dps", "roll_deg", "pitch_deg")]
                    imu[l] = None if None in v else ImuReading(*v, v[0], v[1], 1)
                yield Frame(i, 0, num(r, "t_s") - t0, num(r, "wall") or 0.0, {l: num(r, l + "_g") for l in weight},
                            {l: num(r, l + "_N") for l in tension}, {}, imu, [])

    def stop(self):
        self.stopped = True

    def close(self):
        pass


def cargo_cog(mean, belts, labels, bed):
    """멈춘 구간 평균 [셀 합, X 모멘트, Y 모멘트, T...] -> (화물 g, X, Y). belts 가 None 이면 보정 전 (셀 그대로).
    cog.py 와 같은 식이다."""
    m, mx, my, tension = mean[0], mean[1], mean[2], mean[3:]
    if belts is None:
        return m, mx / m, my / m
    push = [(belts[l].g_per_n * t, belts[l].x, belts[l].y) for l, t in zip(labels, tension)]
    cargo = m - sum(p for p, _, _ in push)
    known = [(p, x) for p, x, _ in push if x is not None]
    x = (mx - sum(p * px for p, px in known)) / (m - sum(p for p, _ in known))
    y = (my - sum(p * py for p, _, py in push)) / cargo
    return cargo, x, y


def belt_text(b):
    return "k %.2f (1N 당 %.0fg)  x %s  y %.0f" % (b.k, b.g_per_n, "모름" if b.x is None else "%.0f" % b.x, b.y)


def report(cfg, fit, plateaus, bed):
    """최종 추정값 + 멈춘 구간별 오차 -> 줄 목록."""
    unit = "mm" if cfg.bed_l else "%"
    sides = cfg.belt_sides()
    out = ["", "=" * 72, "최종 추정값 (벨트마다, 위치 단위 %s, 앞왼 셀 원점 · X 앞왼->앞오 · Y 앞->뒤)" % unit]
    for l in fit.labels:
        b, s = fit.belts[l], fit.start[l]
        out.append("  %s쪽 %-4s %s   <- 시작값 k %.2f y %.0f, 조임 %d번 사용%s" % (
            sides[l][0], l, belt_text(b), s.k, s.y, b.steps, "" if b.steps else " (조임을 못 잡아 시작값 그대로)"))

    base = None
    rows = []
    for t0, t1, mean, kind in plateaus:
        tension = mean[3:]
        if max(tension) < belt_fit.RELEASE_N:
            base = mean
            rows.append((t0, t1, tension, "기준", None))
            continue
        if base is None:
            continue
        errs = []
        for belts in (None, fit.start, fit.belts):
            c0, x0, y0 = cargo_cog(base, belts, fit.labels, bed)
            c1, x1, y1 = cargo_cog(mean, belts, fit.labels, bed)
            errs.append((100 * (c1 - c0) / c0, math.hypot(x1 - x0, y1 - y0)))
        rows.append((t0, t1, tension, kind, errs))

    out += ["", "멈춘 구간별 오차 (같은 짐, 장력 0 인 바로 앞 '기준' 구간 대비. 무게중심 %%는 셀 간격 W %g 기준)" % fit.width,
            "  %-13s %-16s %-4s | %-20s | %-20s | %-20s" % ("시각", "장력 " + "/".join(fit.labels), "용도",
                                                           "보정 전", "시작값 고정", "추정값"),
            "  " + "-" * 104]
    for t0, t1, tension, kind, errs in rows:
        head = "  %5.1f~%5.1fs  %-16s %-4s" % (t0, t1, " / ".join("%.1fN" % t for t in tension), kind)
        if errs is None:
            out.append(head)
            continue
        out.append(head + "".join(" | 무게 %+5.1f%% COG %4.1f%s" % (w, d / fit.width * 100, "%") for w, d in errs))
    if not any(kind == "기준" for _, _, _, kind, _ in rows):
        out.append("  ! 장력 0 인 멈춘 구간(기준)이 없어 오차를 못 냈다. 벨트를 푼 채 3초 가만히 둔 뒤 조여 보라")
    if not any(kind == "검증" for _, _, _, kind, _ in rows):
        out.append("  (검증 구간 없음: 다 조인 뒤 한쪽 벨트를 반쯤 풀고 손을 떼면 배우지 않는 구간이 생겨 진짜 시험이 된다)")
    out.append("  학습 = 추정에 쓴 구간이라 추정값 오차가 좋게 나온다. 검증 = 배우지 않은 구간 (장력이 줌 · 8N 보다 작게 바뀜)")
    return out


def main():
    args = sys.argv[1:]
    replay = None
    if args[:1] == ["--replay"] and len(args) == 2:
        replay = args[1]
    elif any(a not in ("--wait", "--sim") for a in args):
        sys.exit(__doc__)
    wait, is_sim = "--wait" in args, "--sim" in args
    if is_sim and not os.environ.get("HX_LABELS") and not os.environ.get("HX_DT"):
        os.environ["HX_LABELS"] = "FL,FR,RL,RR,T1,T2"
    if is_sim and not os.environ.get("BED_L"):
        os.environ["BED_L"], os.environ["BED_W"] = "250", "392"
    if not replay and not os.environ.get("LOG_NAME"):
        os.environ["LOG_NAME"] = time.strftime("belt_%Y%m%d_%H%M%S")

    cfg = Config()
    if not cfg.belt_fit:
        say("! BELT_FIT=0 이라 추정을 안 한다. BELT_FIT=1 로 돌려라")
        return 1
    analyze = Analyzer(cfg)
    fit = analyze.belt_fit
    if not fit.learn:
        say("! 추정할 벨트가 없다: 모서리 셀 4개(COG_CORNERS)와 장력 셀 2개(HX_TENSION)가 다 있어야 한다")
        return 1
    bed = (cfg.bed_l, cfg.bed_w) if cfg.bed_l else None
    say(cfg.describe())
    say("시작값: " + "  ".join("%s %s" % (l, belt_text(fit.start[l])) for l in fit.labels))
    say(__doc__.split("순서")[1].split("벨트는 한 번에")[0].rstrip().replace("(화면에도 나온다)", "").strip("\n"))
    say()

    sink = None
    if replay:
        src = CsvSource(cfg, replay)
        say("기록 재생: %s" % replay)
    else:
        src = FrameSource(cfg, sim=is_sim)
        os.makedirs(cfg.log_dir, exist_ok=True)
        sink = XlsxSink(cfg, os.path.join(cfg.log_dir, cfg.log_name + ".csv"), os.path.join(cfg.log_dir, cfg.log_name + ".xlsx"))
        for sig_name in ("SIGINT", "SIGTERM", "SIGHUP"):
            sig = getattr(signal, sig_name, None)
            if sig is not None and signal.getsignal(sig) is not signal.SIG_IGN:
                signal.signal(sig, lambda *_: src.stop())
        if sys.stdin and sys.stdin.isatty():         # Enter 로도 끝낸다. 입력이 없으면(EOF, Windows 의 NUL 도 tty 로 보임) 그대로
            threading.Thread(target=lambda: sys.stdin.readline() and src.stop(), daemon=True).start()
        say("기록 -> %s\n%s\n" % (sink.csv_path, "Enter 나 Ctrl+C 로 끝낸다" if wait else
                                   "벨트가 다 추정되고 %g초 그대로면 끝난다 (Enter · Ctrl+C 로 언제든)" % DONE_S))

    plateaus = []               # [(시작, 끝, 평균, 용도)] 멈춘 구간
    prev_still, seen_rows, skipped = None, set(), 0
    last_status = 0.0
    shown = {l: fit.start[l] for l in fit.labels}      # 마지막으로 화면에 찍은 값
    try:
        for f in src.frames():
            r = analyze(f)
            if sink:
                sink.write(f, r)
            # 멈춘 구간이 끝났으면 기록. 추정기가 기준으로 받은 구간만 (손으로 누른 짧은 구간 등은 버린다).
            # 그 순간 줄이 새로 들어갔으면 학습, 아니면 검증
            if fit.still is not prev_still and prev_still is not None and prev_still.t1 - prev_still.t0 >= belt_fit.STILL_S:
                mean = prev_still.mean()
                if fit.ref == mean:
                    new = [x for x in fit.rows if id(x) not in seen_rows]
                    plateaus.append((prev_still.t0, prev_still.t1, mean, "학습" if new else "검증"))
                else:
                    skipped += 1
            seen_rows = {id(x) for x in fit.rows}
            prev_still = fit.still
            # 벨트 추정이 새로 나오거나 눈에 띄게 바뀌면 한 줄 (멈춰 있는 동안 조금씩 다듬는 건 안 찍는다)
            for l in fit.labels:
                b, old = fit.belts[l], shown[l]
                moved = abs(b.k - old.k) > 0.02 or abs(b.y - old.y) > 3 or (b.x is None) != (old.x is None) \
                    or (b.x is not None and abs(b.x - old.x) > 3)
                if b.steps != old.steps or moved:
                    say("\n%6.1fs  %s %s%s" % (f.t_s, l, belt_text(b), "  (시작값으로 돌아감: 풀림)" if not b.steps else ""))
                    shown[l] = b

            s = fit.still
            still_s = s.t1 - s.t0 if s else 0.0
            now = time.monotonic()
            if not replay and now - last_status >= STATUS_EVERY:
                last_status = now
                tension = "  ".join("%s %.1fN" % (l, f.tension_n.get(l) or 0.0) for l in fit.labels)
                status("%6.1fs  %s  셀 합 %s  %s" % (f.t_s, tension, "%.0fg" % r.total_g if r.total_g is not None else "-",
                                                    "멈춤 %.1fs" % still_s if still_s >= belt_fit.STILL_S else "움직임"))
            if not replay and not wait and all(fit.belts[l].steps for l in fit.labels) and still_s >= DONE_S:
                say("\n\n벨트가 다 추정되고 %g초 그대로라 끝낸다" % DONE_S)
                break
            if not replay and cfg.log_seconds > 0 and f.t_s >= cfg.log_seconds:
                say("\n\nLOG_SECONDS=%g 가 지나 끝낸다" % cfg.log_seconds)
                break
    finally:
        src.close()
    # 아직 이어지는 멈춘 구간도 넣는다
    s = fit.still
    if s and s.t1 - s.t0 >= belt_fit.STILL_S:
        plateaus.append((s.t0, s.t1, s.mean(), "학습" if fit.ref is not None and fit._row(s.mean(), s.a) else "검증"))

    lines = report(cfg, fit, plateaus, bed)
    if skipped:
        lines.append("  (기준으로 안 받은 멈춘 구간 %d개는 뺐다: 장력은 그대로인데 무게만 잠깐 달라짐 = 손으로 누름 등)" % skipped)
    say("\n".join(lines))
    if sink:
        sink.close(src, src.notes + ["벨트 누름 추정 테스트 (belt_test.py)"])
        txt = os.path.join(cfg.log_dir, cfg.log_name + "_belt.txt")
        with open(txt, "w", encoding="utf-8") as f:
            f.write("\n".join(lines).strip() + "\n")
        say("결과 -> %s" % txt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
