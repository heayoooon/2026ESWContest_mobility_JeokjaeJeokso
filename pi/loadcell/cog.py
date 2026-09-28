#!/usr/bin/env python3
"""로드셀 4개 → 총 무게 + 무게중심(CoG). 카트에 얹은 상태로 바로 돌리는 스크립트.

  python3 cog.py --scan            # HX711 찾아서 corners.json 뼈대 생성
  python3 cog.py --check           # 어느 모듈이 어느 모서리인지 눌러서 확인
  python3 cog.py --calibrate 1000  # 모서리마다 1000 g 분동으로 영점+스케일
  python3 cog.py                   # 총 무게 + CoG 실시간 (시작할 때 자동 재영점)
  python3 cog.py --keep-zero       # 재영점 없이 (이미 짐이 올라간 상태로 시작)
  python3 cog.py --raw             # 캘리브레이션 없이 CoG 위치만
  python3 cog.py --log cog.csv     # CSV 기록

※ CoG는 모서리 4곳의 하중 **비율**로 나온다. 즉 로드셀 4개가 각각 독립된 HX711에
  물려 있어야 한다. 4개를 콤비네이터로 묶어 HX711 하나에 넣으면 합계만 남고 분포는
  아날로그 단계에서 이미 사라져서, 소프트웨어로 복원할 방법이 없다.
"""
import argparse
import collections
import json
import os
import statistics
import sys
import time

from hx711_lib import HX711

D = os.path.dirname(os.path.abspath(__file__))
CORNERS_PATH = os.path.join(D, "corners.json")
CAL_PATH = os.path.join(D, "cal_corners.json")

NAMES = ("fl", "fr", "rl", "rr")        # 좌전, 우전, 좌후, 우후
KO = {"fl": "좌전", "fr": "우전", "rl": "좌후", "rr": "우후"}

SAFE_FRAC = 0.10        # 안전영역 = 중심에서 길이·폭의 10% 이내
MIN_G = 200.0           # 이보다 가벼우면 CoG는 0/0 이라 계산하지 않는다
NAN = float("nan")


# ── 좌표계 ─────────────────────────────────────────────────────────
# 원점 = 좌후방 모서리, X = 전방(+) 0..L, Y = 좌→우(+) 0..W.
# L, W는 적재함 크기가 아니라 **로드셀 4개의 설치 간격**이다. CoG는 지지점
# 사이의 하중 배분이라, 상판이 아무리 넓어도 셀 간격이 기준이 된다.
def corner_xy(L, W):
    return {"fl": (L, 0.0), "fr": (L, W), "rl": (0.0, 0.0), "rr": (0.0, W)}


def compute_cog(g, L, W, min_g=MIN_G):
    """모서리 하중 dict{fl,fr,rl,rr} (g) → (total, x, y, off_x, off_y, safe, valid).

        x = Σ(w_i · x_i) / Σw_i      y = Σ(w_i · y_i) / Σw_i

    순수 함수라 CSV 리플레이에도 그대로 쓸 수 있다.
    """
    xy = corner_xy(L, W)
    if any(n not in g or g[n] is None or g[n] != g[n] for n in NAMES):
        return NAN, NAN, NAN, NAN, NAN, False, False       # 채널 결측
    total = sum(g[n] for n in NAMES)
    if total < min_g:
        # 빈 카트. 총 무게는 실측값 그대로 주고 위치만 NaN으로 둔다.
        # (0을 넣으면 뒷단에서 "화물이 사라졌다"로 오판정된다)
        return total, NAN, NAN, NAN, NAN, False, False
    x = sum(g[n] * xy[n][0] for n in NAMES) / total
    y = sum(g[n] * xy[n][1] for n in NAMES) / total
    off_x, off_y = x - L / 2.0, y - W / 2.0
    safe = abs(off_x) <= L * SAFE_FRAC and abs(off_y) <= W * SAFE_FRAC
    return total, x, y, off_x, off_y, safe, True


# ── 설정 파일 ──────────────────────────────────────────────────────
def load_conf():
    if not os.path.exists(CORNERS_PATH):
        sys.exit(f"{CORNERS_PATH} 없음 → 먼저 `python3 cog.py --scan`")
    conf = json.load(open(CORNERS_PATH))
    L, W = conf.get("span_cm", [60, 40])
    pins = conf.get("corners", {})
    have = [n for n in NAMES if n in pins and pins[n].get("dt")]
    return L, W, pins, have


def open_cells(pins, have):
    cells = {}
    for n in have:
        cells[n] = HX711(pins[n]["dt"], pins[n]["sck"])
    cal = json.load(open(CAL_PATH)) if os.path.exists(CAL_PATH) else {}
    for n, hx in cells.items():
        if n in cal:
            hx.offset = cal[n]["offset"]
            hx.scale = cal[n]["scale"]
    return cells, cal


# ── --scan ─────────────────────────────────────────────────────────
def do_scan():
    """헤더에서 HX711을 찾아 corners.json 뼈대를 만든다. 기존 파일은 안 덮어쓴다."""
    import lgpio
    pins = [p for p in range(4, 28)]           # GPIO0~3 = EEPROM/I2C, 건드리지 않음
    h = lgpio.gpiochip_open(0)
    ok = []
    for p in pins:
        try:
            lgpio.gpio_claim_input(h, p)
            ok.append(p)
        except Exception:
            pass
    found = []
    for sck in ok:
        others = [p for p in ok if p != sck]
        try:
            lgpio.gpio_free(h, sck)
            lgpio.gpio_claim_output(h, sck, 0)
        except Exception:
            continue
        time.sleep(0.5)                        # 파워다운 해제 + 첫 변환
        before = {p: lgpio.gpio_read(h, p) for p in others}
        for _ in range(25):
            lgpio.gpio_write(h, sck, 1)
            lgpio.gpio_write(h, sck, 0)
        after = {p: lgpio.gpio_read(h, p) for p in others}
        for d in others:
            if before[d] == 0 and after[d] == 1:
                print(f"  HX711: SCK=GPIO{sck}  DT=GPIO{d}")
                found.append({"dt": d, "sck": sck})
        lgpio.gpio_free(h, sck)
        lgpio.gpio_claim_input(h, sck)
    lgpio.gpiochip_close(h)

    print(f"\nHX711 {len(found)}개 발견.")
    if len(found) < 4:
        print(f"⚠ CoG에는 4개가 필요하다 (모서리당 1개). 지금은 {len(found)}개뿐이라 "
              "총 무게만 나온다.")

    if os.path.exists(CORNERS_PATH):
        print(f"\n{CORNERS_PATH} 이미 있음 — 덮어쓰지 않았다. 위 핀으로 직접 고칠 것.")
        return
    conf = {
        "_주석": "span_cm = 로드셀 4개의 설치 간격 [앞뒤(L), 좌우(W)] cm. 실측해서 고칠 것.",
        "span_cm": [60, 40],
        "corners": {n: (found[i] if i < len(found) else {"dt": None, "sck": None})
                    for i, n in enumerate(NAMES)},
    }
    json.dump(conf, open(CORNERS_PATH, "w"), indent=2, ensure_ascii=False)
    print(f"{CORNERS_PATH} 생성. 발견 순서대로 fl/fr/rl/rr에 배정했을 뿐이니 "
          "`--check`로 실제 위치를 맞출 것.")


# ── --check : 채널 ↔ 모서리 매핑 확인 ──────────────────────────────
def do_check():
    L, W, pins, have = load_conf()
    cells, _ = open_cells(pins, have)
    base = {n: hx.read_raw(10) for n, hx in cells.items()}
    print("\n모서리를 하나씩 손으로 눌러본다. 누른 모서리의 막대만 길어져야 정상.")
    print("어긋나면 corners.json의 fl/fr/rl/rr 에 붙은 핀만 바꾸면 된다. Ctrl+C 종료.\n")
    try:
        while True:
            line = []
            for n in NAMES:
                if n not in cells:
                    line.append(f"{KO[n]} {'(없음)':>14s}")
                    continue
                d = cells[n].read_raw(3) - base[n]
                bar = "#" * min(12, int(abs(d) / 2000))
                line.append(f"{KO[n]} {d:+8.0f} {bar:<12s}")
            print("  " + " | ".join(line), end="\r", flush=True)
    except KeyboardInterrupt:
        print()
    finally:
        for hx in cells.values():
            hx.close()


# ── --calibrate ────────────────────────────────────────────────────
def do_calibrate(known_g):
    L, W, pins, have = load_conf()
    if len(have) < 4:
        print(f"⚠ 모서리 {len(have)}개만 설정돼 있다. 있는 것만 캘리브레이션한다.\n")
    cells, cal = open_cells(pins, have)
    print("전원 넣고 10분 이상 지난 뒤에 할 것. 워밍업 드리프트가 분동보다 크다.\n")
    try:
        input("카트를 완전히 비우고 Enter (영점): ")
        zeros = {}
        for n, hx in cells.items():
            v = [hx._read_raw_once() for _ in range(40)]
            zeros[n] = statistics.median(v)
            sd = statistics.pstdev(v)
            print(f"  {KO[n]} zero = {zeros[n]:10.0f}   σ = {sd:6.0f}"
                  + ("   ⚠ 흔들림 큼" if sd > 5000 else ""))

        out = {}
        for n, hx in cells.items():
            input(f"\n[{KO[n]}] {known_g} g 분동을 {KO[n]} 로드셀 **바로 위**에 올리고 Enter: ")
            loaded = {m: statistics.median([c._read_raw_once() for _ in range(40)])
                      for m, c in cells.items()}
            delta = loaded[n] - zeros[n]
            scale = delta / known_g
            out[n] = {"offset": zeros[n], "scale": scale}
            print(f"  Δ = {delta:+.0f} counts → scale = {scale:.3f} counts/g"
                  + ("   (음수 = A+/A- 반대. 계수에 흡수되니 그냥 두면 된다)"
                     if scale < 0 else ""))
            if abs(delta) < 1000:
                print("  ⚠ 변화량이 거의 없다. 분동이 이 셀에 안 실렸거나 배선 문제.")
            leak = max((abs(loaded[m] - zeros[m]) for m in cells if m != n), default=0)
            if abs(delta) > 0 and leak / abs(delta) > 0.3:
                print(f"  ⚠ 이웃 모서리로 {leak / abs(delta) * 100:.0f}% 샌다. "
                      "상판이 비틀리거나 로드셀이 구속돼 있다.")
            input("  분동을 치우고 Enter: ")

        json.dump(out, open(CAL_PATH, "w"), indent=2)
        print(f"\n{CAL_PATH} 저장 완료")

        if len(out) == 4:
            print("\n=== 검증: 위치를 바꿔도 합계가 같아야 한다 ===")
            for hx_n, hx in cells.items():
                hx.offset = out[hx_n]["offset"]
                hx.scale = out[hx_n]["scale"]
            totals = []
            for k in range(3):
                input(f"  [{k+1}/3] 같은 분동을 아무 위치에나 올리고 Enter: ")
                g = {n: hx.get_units(15) for n, hx in cells.items()}
                t = sum(g.values())
                totals.append(t)
                print("    " + "  ".join(f"{KO[n]}={g[n]:+7.1f}" for n in NAMES if n in g)
                      + f"   합계 {t:7.1f} g  (오차 {t - known_g:+.1f})")
                input("    분동 치우고 Enter: ")
            err = max(abs(t - known_g) for t in totals) / known_g * 100
            print(f"\n최대 오차 {err:.1f} %"
                  + ("  → 통과" if err < 5 else
                     "\n→ 위치에 따라 합계가 달라진다. 셀 높이가 안 맞아 하나가 떠 있거나,"
                     "\n  로드셀이 기구부에 구속돼 하중이 프레임으로 새고 있다."))
    finally:
        for hx in cells.values():
            hx.close()


# ── 화면 ───────────────────────────────────────────────────────────
def bed_map(x, y, safe, valid, L, W, cols=25, rows=9):
    grid = [[" "] * cols for _ in range(rows)]

    def cell(cx, cy):
        c = int(round(cy / W * (cols - 1)))
        r = int(round((1.0 - cx / L) * (rows - 1)))
        return max(0, min(rows - 1, r)), max(0, min(cols - 1, c))

    for sx in (0.5 - SAFE_FRAC, 0.5 + SAFE_FRAC):
        for sy in (0.5 - SAFE_FRAC, 0.5 + SAFE_FRAC):
            r, c = cell(sx * L, sy * W)
            grid[r][c] = "."
    r, c = cell(L / 2, W / 2)
    grid[r][c] = "+"
    if valid:
        r, c = cell(x, y)
        grid[r][c] = "O" if safe else "X"

    return (["  +" + "-" * cols + "+   <- 앞 (fl/fr)"]
            + ["  |" + "".join(row) + "|" for row in grid]
            + ["  +" + "-" * cols + "+   <- 뒤 (rl/rr)"])


def do_live(args):
    L, W, pins, have = load_conf()
    cells, cal = open_cells(pins, have)
    unit = "g"
    if args.raw:
        for hx in cells.values():
            hx.scale = 1.0
        unit = "cnt"
        print("⚠ --raw: 무게 숫자는 단위 없는 counts다. CoG 위치만 믿을 것.")
    else:
        missing = [n for n in have if n not in cal]
        if missing:
            sys.exit(f"캘리브레이션 없는 모서리: {missing} → "
                     f"`python3 cog.py --calibrate <분동 g>` 또는 --raw")

    if len(have) < 4:
        print(f"⚠ 모서리 {len(have)}개({[KO[n] for n in have]})만 연결됨 → "
              "총 무게만 나오고 CoG는 계산할 수 없다.\n")

    try:
        if not args.keep_zero:
            input("카트를 비우고 Enter (재영점): ")
            for n, hx in cells.items():
                hx.tare(30)
            print("  완료\n")

        writer = fh = None
        if args.log:
            import csv
            fh = open(args.log, "w", newline="", encoding="utf-8")
            writer = csv.writer(fh)
            writer.writerow(["t", "total", "x_cg", "y_cg"] + list(NAMES))

        hist = {n: collections.deque(maxlen=max(1, args.avg)) for n in have}
        drawn = 0
        n_read = 0
        t0 = time.monotonic()
        print("측정 시작. Ctrl+C 종료.\n")
        try:
            while True:
                for n in have:
                    hist[n].append(cells[n].get_units(args.n))
                n_read += 1
                g = {n: statistics.median(hist[n]) for n in have}
                total, x, y, ox, oy, safe, valid = compute_cog(g, L, W)
                if not valid and len(have) < 4:
                    total = sum(g.values())       # 불완전 배선이어도 합계는 유효

                if writer:
                    writer.writerow([f"{time.time():.3f}", f"{total:.1f}",
                                     f"{x:.2f}", f"{y:.2f}"]
                                    + [f"{g.get(n, NAN):.1f}" for n in NAMES])

                lines = []
                if valid:
                    lines.append(f"  W = {total:9.1f} {unit}   "
                                 f"CoG = ({x:5.1f}, {y:5.1f}) cm   "
                                 f"편차 ({ox:+5.1f}, {oy:+5.1f})  "
                                 f"[{'안전' if safe else '★이탈★'}]")
                else:
                    lines.append(f"  W = {total:9.1f} {unit}   "
                                 f"CoG = (  -  ,   -  ) cm   "
                                 f"(빈 카트 또는 모서리 부족)")
                lines.append("")
                lines += bed_map(x, y, safe, valid, L, W)
                lines.append("")
                lines.append("   " + "   ".join(
                    f"{KO[n]}={g[n]:+8.1f}" if n in g else f"{KO[n]}={'-':>8s}"
                    for n in NAMES))
                # raw + 재영점 없음 = counts의 절대값이라 부호에 의미가 없다
                neg = ([] if (args.raw and args.keep_zero)
                       else [KO[n] for n in g if g[n] < -50])
                rate = n_read / (time.monotonic() - t0)
                lines.append(f"   {rate:.1f} Hz"
                             + (f"   ⚠ 음수 {neg} — 그 모서리가 떠 있거나 영점이 틀렸다"
                                if neg else ""))

                if drawn:
                    print(f"\033[{drawn}A", end="")
                print("\n".join("\033[K" + ln for ln in lines), flush=True)
                drawn = len(lines)
        except KeyboardInterrupt:
            pass
        finally:
            if fh:
                fh.close()
                print(f"\n{args.log} 기록 완료")
    finally:
        for hx in cells.values():
            hx.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scan", action="store_true", help="HX711 탐색 → corners.json")
    ap.add_argument("--check", action="store_true", help="채널 ↔ 모서리 매핑 확인")
    ap.add_argument("--calibrate", type=float, metavar="G", help="분동 무게(g)로 캘리브레이션")
    ap.add_argument("--keep-zero", action="store_true", help="시작할 때 재영점 안 함")
    ap.add_argument("--raw", action="store_true", help="캘리브레이션 없이 위치만")
    ap.add_argument("--avg", type=int, default=5, help="이동중앙값 샘플 수 (기본 5)")
    ap.add_argument("--n", type=int, default=3, help="HX711 1회 읽기당 중앙값 샘플 수")
    ap.add_argument("--log", help="CSV 경로")
    args = ap.parse_args()

    if args.scan:
        do_scan()
    elif args.check:
        do_check()
    elif args.calibrate:
        do_calibrate(args.calibrate)
    else:
        do_live(args)


if __name__ == "__main__":
    main()
