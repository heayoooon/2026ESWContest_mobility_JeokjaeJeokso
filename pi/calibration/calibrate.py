#!/usr/bin/env python3
"""로드셀 영점(offset)과 배율(scale)을 재서 calib.json 에 저장한다.

HX711 의 원시값은 그냥 카운트다. 무게(g)로 바꾸려면 채널마다 두 숫자가 필요하다.

    g = (raw - offset) * scale_g_per_count

핀 배치는 log_sync.py 와 같은 환경변수를 쓴다 (config.py 참고).

    # 1) 영점: 판 위를 비우고 장력 벨트도 풀어둔다 (전 채널)
    python3 calibration/calibrate.py tare

    # 2) 배율: 아는 무게를 판 위에 올리고 (어디에 올려도 된다)
    python3 calibration/calibrate.py span 1000        # 1000 g

    # 바닥 셀마다 배율 따로: 같은 무게를 판 위 5군데에 차례로 올린다 (권장)
    python3 calibration/calibrate.py multi 5000

    # 장력(인장형) 셀: 아는 무게를 그 셀 고리에 매단다
    python3 calibration/calibrate.py cell T1 1000

    # 지금 값 보기
    python3 calibration/calibrate.py show

`span` 은 바닥 하중 채널의 합이 올린 무게와 맞도록 **공통 배율 하나**를 구한다. 네 셀의
감도가 정말 같을 때만 맞고, 다르면 무게를 어디 올렸느냐에 따라 총하중이 달라진다.

`multi` 는 같은 무게를 가운데와 네 모서리 쪽에 한 번씩 올려서 **셀마다 배율**을 최소제곱으로
구한다. 올린 자리마다 "합 = 무게", "무게중심 = 올린 자리" 두 조건을 같이 맞추므로 총하중과
CoG 가 둘 다 맞는다. 배율이 음수로 나오면 그 셀은 부호가 반대라는 뜻이고 배율에 그대로
들어가므로 HX_INVERT 가 필요 없다. 5군데 오차가 크게 남으면 배율 문제가 아니라
하중이 셀을 거치지 않는(판이 프레임·볼트·선에 걸린) 기구 문제다.

바닥 셀에 `cell` 을 쓰지 않는 이유: 판이 단단하면 모서리 바로 위에 올려도 그 셀이 다 받지
않는다 (네 셀 판에서 한 모서리에 P 를 올리면 대략 그 셀 3/4, 이웃 둘 1/4 씩, 대각선 -1/4).
그 셀 변화량만으로 배율을 잡으면 틀린다. `cell` 은 장력 셀용이다.

HX_TENSION 에 든 장력 채널(기본 T1,T2)은 `span`/`multi` 에서 빠지므로 `cell` 로 따로
잡는다. 장력도 g 로 보정해 두고 log_xlsx.py 가 N 으로 바꿔 적는다.
"""
import json
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # pi/

from acquisition.config import Config  # noqa: E402
from sensors.hx711_bus import HX711Bus  # noqa: E402

CALIB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calib.json")
N_SAMPLES = 20          # 평균 낼 샘플 수
SETTLE_S = 3            # 무게를 올리고 손 뗀 뒤 흔들림이 가라앉기를 기다리는 시간
MOMENT_W = 1.0          # multi: 무게중심 조건의 가중치 (좌표는 셀 간격 = 1 단위)


def load_calib():
    if not os.path.exists(CALIB_PATH):
        return {"offset": {}, "scale_g_per_count": {}}
    with open(CALIB_PATH, encoding="utf-8") as f:
        c = json.load(f)
    c.setdefault("offset", {})
    c.setdefault("scale_g_per_count", {})
    return c


def save_calib(c):
    c["updated"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    with open(CALIB_PATH, "w", encoding="utf-8") as f:
        json.dump(c, f, indent=2, ensure_ascii=False)
    print("저장: %s" % CALIB_PATH)


def stale(c, cfg, labels):
    """영점을 잡을 때와 HX_INVERT 가 달라진 채널. 원시값 부호가 뒤집혔으니 그 offset 은 못 쓴다."""
    was = set(c.get("invert", []))
    return [l for l in labels if l in c["offset"] and (l in was) != (l in cfg.hx_invert)]


def collect(cfg, n=N_SAMPLES):
    """n개 샘플을 모아 채널별 중앙값을 돌려준다. 중앙값이라 튄 샘플에 안 끌린다."""
    bus = HX711Bus(cfg.hx_sck, cfg.hx_dt, cfg.hx_labels,
                   gain=cfg.hx_gain, pull=cfg.hx_pull, chip=cfg.hx_chip,
                   invert=cfg.hx_invert)
    print(bus.describe())
    try:
        bus.sync_reset()
        bus.read(timeout=cfg.hx_timeout)        # 파워다운 직후 첫 값은 버린다
        rows = []
        while len(rows) < n:
            s = bus.read(timeout=cfg.hx_timeout)
            if "sck_high" in s.flags:           # 타이밍 깨진 샘플은 버린다
                continue
            rows.append(s.values)
            sys.stdout.write("\r  수집 %d/%d  %s   " % (
                len(rows), n,
                " ".join("%s=%d" % (l, v) for l, v in zip(cfg.hx_labels, s.values))))
            sys.stdout.flush()
        print()
    finally:
        bus.close()

    out = {}
    for i, label in enumerate(cfg.hx_labels):
        col = [r[i] for r in rows]
        out[label] = {
            "median": statistics.median(col),
            "mean": statistics.fmean(col),
            "spread": max(col) - min(col),
        }
    return out


def cmd_tare(cfg):
    print("판 위를 비우고 장력 벨트도 느슨하게 풀어두세요. 3초 뒤 시작합니다.")
    time.sleep(3)
    got = collect(cfg)
    c = load_calib()
    flipped = stale(c, cfg, got)
    for label, st in got.items():
        c["offset"][label] = st["median"]
        print("  %-4s offset=%d   (변동폭 %d)" % (label, st["median"], st["spread"]))
    for label in flipped:       # 부호가 바뀐 채널의 옛 배율은 믿을 수 없다
        c["scale_g_per_count"].pop(label, None)
    if flipped:
        print("\n  HX_INVERT 가 바뀐 채널 %s 은 배율을 지웠다 -> span 또는 cell 로 다시"
              % ", ".join(flipped))
    c["invert"] = sorted((set(c.get("invert", [])) - set(got)) |
                         {l for l in got if l in cfg.hx_invert})
    c["labels"] = list(cfg.hx_labels)
    save_calib(c)


def cmd_span(cfg, grams):
    weight, tension, _ = cfg.roles()
    if not weight:
        sys.exit("바닥 하중 채널이 없다 (HX_LABELS 가 전부 HX_TENSION 에 들어 있음)")
    c = load_calib()
    missing = [l for l in weight if l not in c["offset"]]
    if missing:
        sys.exit("영점이 없는 채널: %s  -> 먼저 `python3 calibration/calibrate.py tare`" % ", ".join(missing))
    flipped = stale(c, cfg, weight)
    if flipped:
        sys.exit("HX_INVERT 가 영점 잡을 때와 다른 채널: %s  -> 먼저 `python3 calibration/calibrate.py tare`"
                 % ", ".join(flipped))

    print("%g g 을 판 위에 올리세요. 3초 뒤 시작합니다." % grams)
    if tension:
        print("(장력 채널 %s 은 빠진다. `cell <라벨> <그램>` 으로 따로)" % ", ".join(tension))
    time.sleep(3)
    got = collect(cfg)

    total = 0.0
    for label in weight:
        d = got[label]["median"] - c["offset"][label]
        total += d
        print("  %-4s 변화량 %+d" % (label, d))
    if abs(total) < 1000:
        sys.exit("하중 채널 합 변화량이 %d 밖에 안 된다. 무게가 안 올라갔거나 배선 문제." % total)

    scale = grams / total
    for label in weight:
        c["scale_g_per_count"][label] = scale
    c["span_note"] = "하중 채널 공통 배율: %g g / 합계변화 %d" % (grams, total)
    print("\n  합계 변화량 %d  ->  공통 배율 %.6g g/count" % (total, scale))
    save_calib(c)


def cmd_cell(cfg, label, grams):
    if label not in cfg.hx_labels:
        sys.exit("모르는 채널 %r. 지금 라벨: %s" % (label, ", ".join(cfg.hx_labels)))
    c = load_calib()
    if label not in c["offset"]:
        sys.exit("%s 의 영점이 없다 -> 먼저 `python3 calibration/calibrate.py tare`" % label)
    if stale(c, cfg, [label]):
        sys.exit("%s 는 HX_INVERT 가 영점 잡을 때와 다르다 -> 먼저 `python3 calibration/calibrate.py tare`" % label)

    _, tension, _ = cfg.roles()
    where = "%s 고리에 매다세요" % label if label in tension else "%s 모서리 바로 위에 올리세요" % label
    print("%g g 을 %s. 3초 뒤 시작합니다." % (grams, where))
    time.sleep(3)
    got = collect(cfg)

    d = got[label]["median"] - c["offset"][label]
    if abs(d) < 1000:
        sys.exit("%s 변화량이 %d 밖에 안 된다. 무게가 그 셀에 안 실렸거나 배선 문제." % (label, d))
    c["scale_g_per_count"][label] = grams / d
    print("  %s 변화량 %+d  ->  배율 %.6g g/count" % (label, d, grams / d))
    save_calib(c)


# --------------------------------------------------------------------------
# multi: 셀마다 배율
# --------------------------------------------------------------------------
def corner_xy(corners):
    """COG_CORNERS(앞왼,앞오,뒤왼,뒤오) -> 셀 좌표. 셀 간격 = 1, 앞왼 셀이 원점."""
    return dict(zip(corners, [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)]))


def multi_spots(corners):
    """무게 올릴 자리: 가운데 + 모서리 셀마다 '그 셀과 가운데의 중간'."""
    xy = corner_xy(corners)
    return [("가운데", 0.5, 0.5)] + [
        ("%s 쪽" % l, 0.25 + 0.5 * xy[l][0], 0.25 + 0.5 * xy[l][1]) for l in corners]


def solve(a, b):
    """작은 정방 연립방정식 a x = b (부분 피벗 가우스 소거). 특이하면 ValueError."""
    n = len(b)
    m = [list(row) + [b[i]] for i, row in enumerate(a)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-9:
            raise ValueError("특이 행렬")
        m[col], m[piv] = m[piv], m[col]
        for r in range(n):
            if r != col:
                f = m[r][col] / m[col][col]
                for j in range(col, n + 1):
                    m[r][j] -= f * m[col][j]
    return [m[i][n] / m[i][i] for i in range(n)]


def fit_multi(deltas, spots, xy, grams):
    """자리마다 '합 = grams', '무게중심 = 올린 자리' 를 최소제곱으로 맞추는 셀별 배율.

    deltas: 자리마다 [셀별 변화량(count)], spots: 자리마다 (x, y), xy: 셀마다 (x, y).
    배율은 부호까지 풀린다 (음수 = 그 셀 부호가 반대).
    """
    n = len(xy)
    rows, rhs = [], []
    for d, (x, y) in zip(deltas, spots):
        rows.append(list(d))
        rhs.append(grams)
        # 올린 자리를 기준으로 한 모멘트 합 = 0  <=>  무게중심 = 올린 자리
        rows.append([MOMENT_W * d[i] * (xy[i][0] - x) for i in range(n)])
        rhs.append(0.0)
        rows.append([MOMENT_W * d[i] * (xy[i][1] - y) for i in range(n)])
        rhs.append(0.0)
    # 셀마다 변화량 크기가 수십 배 다를 수 있으니 열을 정규화해서 푼다
    norm = [max(abs(r[i]) for r in rows) or 1.0 for i in range(n)]
    rows = [[r[i] / norm[i] for i in range(n)] for r in rows]
    ata = [[sum(r[i] * r[j] for r in rows) for j in range(n)] for i in range(n)]
    atb = [sum(r[i] * v for r, v in zip(rows, rhs)) for i in range(n)]
    return [k / s for k, s in zip(solve(ata, atb), norm)]


def predict(d, k, xy):
    """변화량 d 와 배율 k -> (총하중 g, 무게중심 x, y). 하중이 0 근처면 무게중심은 None."""
    g = [di * ki for di, ki in zip(d, k)]
    tot = sum(g)
    if abs(tot) < 1e-9:
        return tot, None, None
    return (tot, sum(gi * p[0] for gi, p in zip(g, xy)) / tot,
            sum(gi * p[1] for gi, p in zip(g, xy)) / tot)


def ask(prompt):
    try:
        return input(prompt).strip().lower()
    except EOFError:
        sys.exit("\n입력이 없어 중단 (ssh -t 로 접속했는지 확인)")


def cmd_multi(cfg, grams):
    _, _, corners = cfg.roles()
    if not corners:
        sys.exit("COG_CORNERS(%s) 4개가 전부 하중 채널이어야 한다" % ",".join(cfg.cog_corners))
    xy_of = corner_xy(corners)
    xy = [xy_of[l] for l in corners]
    spots = multi_spots(corners)

    print("같은 %g g 을 아래 %d 군데에 한 번씩 올립니다. 무게의 중심을 그 자리에 맞추세요."
          % (grams, len(spots)))
    print("  가운데 = 네 셀의 한가운데.  '%s 쪽' = %s 셀과 가운데를 잇는 선의 딱 중간."
          % (corners[0], corners[0]))
    print("  테이프로 자리를 표시해 두면 편합니다. 올린 뒤에는 손을 떼세요 (%d초 뒤 측정)."
          % SETTLE_S)

    ask("\n판 위를 전부 비우고 Enter ")
    empty0 = collect(cfg)

    taken = []                              # (자리 이름, x, y, 측정값, 순번)
    for step, (name, x, y) in enumerate(spots, 1):
        a = ask("\n[%d/%d] %g g 을 [%s] 에 올리고 Enter (건너뛰기 s) "
                % (step, len(spots), grams, name))
        if a == "s":
            continue
        time.sleep(SETTLE_S)
        got = collect(cfg)
        d = [got[l]["median"] - empty0[l]["median"] for l in corners]
        print("  변화량  " + "  ".join("%s %+d" % (l, v) for l, v in zip(corners, d))
              + "   합 %+d" % sum(d))
        taken.append((name, x, y, got, step))

    ask("\n무게를 내리고 판 위를 비운 채 Enter ")
    time.sleep(SETTLE_S)
    empty1 = collect(cfg)

    if len(taken) < 3:
        sys.exit("자리가 %d 군데뿐이라 셀 4개 배율을 못 푼다 (3군데 이상)" % len(taken))

    # 영점이 시간에 따라 흐르면 처음/끝 빈 판 사이를 순번으로 직선 보간해 뺀다
    last = len(spots) + 1
    deltas = []
    for _, _, _, got, step in taken:
        f = step / float(last)
        deltas.append([got[l]["median"] - (empty0[l]["median"] * (1 - f) + empty1[l]["median"] * f)
                       for l in corners])
    pts = [(x, y) for _, x, y, _, _ in taken]
    try:
        k = fit_multi(deltas, pts, xy, grams)
    except ValueError:
        sys.exit("배율을 못 푼다: 어떤 셀은 어느 자리에서도 거의 안 변했다 (배선/기구 확인)")

    # ---- 결과 ----
    sens = [1.0 / abs(v) for v in k]        # count/g
    mid = statistics.median(sens)
    print("\n셀별 배율")
    for l, v, s in zip(corners, k, sens):
        note = []
        if v < 0:
            note.append("부호 반대 (배율에 반영됨)")
        if s < mid / 3:
            note.append("하중이 거의 안 옴 -> 판이 이 근처에서 다른 데 걸렸는지 확인")
        print("  %-4s %+.6g g/count   감도 %.0f count/g %s"
              % (l, v, s, "  <- " + ", ".join(note) if note else ""))

    common_sum = [sum(d) for d in deltas]
    common = grams * sum(common_sum) / sum(s * s for s in common_sum)
    print("\n%-10s %9s %7s %13s %13s %13s" % ("자리", "총하중(g)", "오차", "무게중심 x,y",
                                              "올린 자리", "공통배율이면"))
    worst_w, worst_c = 0.0, 0.0
    for (name, x, y, _, _), d, s in zip(taken, deltas, common_sum):
        tot, cx, cy = predict(d, k, xy)
        err = tot / grams - 1
        worst_w = max(worst_w, abs(err))
        if cx is None:                      # 합이 0 이라 무게중심이 없다
            cx = cy = float("nan")
            worst_c = 1.0
        else:
            worst_c = max(worst_c, abs(cx - x), abs(cy - y))
        print("%-10s %9.0f %+6.1f%% %6.2f,%5.2f %6.2f,%5.2f %+12.1f%%"
              % (name, tot, err * 100, cx, cy, x, y, (common * s / grams - 1) * 100))

    drift = [(empty1[l]["median"] - empty0[l]["median"]) * v for l, v in zip(corners, k)]
    print("\n빈 판 영점 복귀 (처음 -> 끝): " + "  ".join(
        "%s %+.0f" % (l, g) for l, g in zip(corners, drift)) + "   합 %+.0f g" % sum(drift))

    print()
    if worst_w <= 0.02 and worst_c <= 0.05:
        print("판정: 좋음. 어디 올려도 총하중 ±%.1f%%, 무게중심 ±%.2f 셀간격"
              % (worst_w * 100, worst_c))
    elif worst_w <= 0.05 and worst_c <= 0.10:
        print("판정: 쓸 만함 (총하중 최대 %.1f%%, 무게중심 최대 %.2f 셀간격 오차)."
              " 자리를 더 정확히 맞춰 한 번 더 해볼 것" % (worst_w * 100, worst_c))
    else:
        print("판정: 셀별 배율로도 총하중 %.0f%%, 무게중심 %.2f 셀간격 오차가 남는다"
              " -> 배율이 아니라 기구 문제다." % (worst_w * 100, worst_c))
        print("  하중 일부가 셀을 거치지 않고 새고 있다. 판이 프레임·벽·볼트 머리·배선에")
        print("  닿는지, 셀 변형부(가운데 휘는 부분) 아래에 틈이 있는지, 받침이 평평한지 확인.")
    if abs(sum(drift)) > 0.02 * grams:
        print("  영점이 %.0f g 안 돌아왔다 -> 판이 걸렸다 풀렸다 한다 (마찰/간섭). 위와 같은 원인."
              % sum(drift))

    if ask("\n이 배율과 끝 영점을 calib.json 에 저장할까요? [y/N] ") != "y":
        print("저장 안 함")
        return
    c = load_calib()
    for l, v in zip(corners, k):
        c["offset"][l] = empty1[l]["median"]
        c["scale_g_per_count"][l] = v
    c["invert"] = sorted((set(c.get("invert", [])) - set(corners)) |
                         {l for l in corners if l in cfg.hx_invert})
    c["labels"] = list(cfg.hx_labels)
    c["span_note"] = "셀별 배율 (multi %g g, %d군데, 최대 오차 %.1f%%)" % (
        grams, len(taken), worst_w * 100)
    c["multi"] = {                          # 나중에 다시 풀어볼 수 있게 원자료를 남긴다
        "grams": grams, "corners": list(corners),
        "empty0": [empty0[l]["median"] for l in corners],
        "empty1": [empty1[l]["median"] for l in corners],
        "spots": [{"name": n, "x": x, "y": y, "step": st,
                   "raw": [got[l]["median"] for l in corners]}
                  for n, x, y, got, st in taken],
    }
    save_calib(c)


def cmd_show(cfg):
    c = load_calib()
    _, tension, _ = cfg.roles()
    print("%-6s %-4s %12s %16s" % ("채널", "역할", "offset", "scale(g/count)"))
    for label in cfg.hx_labels:
        off = c["offset"].get(label)
        sc = c["scale_g_per_count"].get(label)
        print("%-6s %-4s %12s %16s" % (
            label, "장력" if label in tension else "하중",
            "%d" % off if off is not None else "-",
            "%.6g" % sc if sc is not None else "-"))
    if cfg.hx_invert:
        print("\n부호반전 (HX_INVERT): %s" % ", ".join(cfg.hx_invert))
    flipped = stale(c, cfg, cfg.hx_labels)
    if flipped:
        print("! HX_INVERT 가 영점 잡을 때와 다른 채널: %s -> tare 부터 다시" % ", ".join(flipped))
    if c.get("updated"):
        print("\n마지막 갱신: %s" % c["updated"])
    if not os.path.exists(CALIB_PATH):
        print("\n(%s 없음 - 아직 보정 안 함)" % CALIB_PATH)


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    cfg = Config()
    cmd = sys.argv[1]

    if cmd == "tare":
        cmd_tare(cfg)
    elif cmd == "span":
        if len(sys.argv) < 3:
            sys.exit("사용법: python3 calibration/calibrate.py span <그램>")
        cmd_span(cfg, float(sys.argv[2]))
    elif cmd == "multi":
        if len(sys.argv) < 3:
            sys.exit("사용법: python3 calibration/calibrate.py multi <그램>")
        cmd_multi(cfg, float(sys.argv[2]))
    elif cmd == "cell":
        if len(sys.argv) < 4:
            sys.exit("사용법: python3 calibration/calibrate.py cell <라벨> <그램>")
        cmd_cell(cfg, sys.argv[2], float(sys.argv[3]))
    elif cmd == "show":
        cmd_show(cfg)
    else:
        sys.exit(__doc__)
    return 0


if __name__ == "__main__":
    sys.exit(main())
