"""기록된 CSV(acquisition/logs/*.csv)를 모션 보상 + 위험 등급에 흘려서 런마다 요약한다.
기준값을 로그에 맞춰 고칠 때 쓴다. pi/ 에서:

    python3 -m algorithm.replay_logs [로그폴더] [-v]

로드셀 간격(BED_L x BED_W mm, 환경변수로 바꿀 수 있다)으로 % -> cm 를 바꾼다. 기본은 이 대차 실측 392 x 250.
"""
import csv
import glob
import os
import sys
from collections import Counter

import numpy as np

from algorithm.motion_comp import MotionCompensator, MotionParams
from algorithm.risk import RiskAssessor, RiskParams

DEFAULT_BED_MM = (392.0, 250.0)     # 나무판 59.2 x 44.8 cm 위 로드셀 간격 (가로 x 세로)
TENSION = "c1"      # 장력 셀 라벨 (HX_TENSION)
IMU = "imu1"


def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def replay(path, mp=None, rp=None, bed_mm=None):
    L, W = [v / 10 for v in (bed_mm or DEFAULT_BED_MM)]
    comp, risk = MotionCompensator(mp), RiskAssessor(rp)
    out = []
    for r in csv.DictReader(open(path, encoding="utf-8-sig")):
        t = _f(r["t_s"])
        cx, cy = _f(r["cog_x_pct"]), _f(r["cog_y_pct"])
        x = cx / 100 * L if cx is not None else None
        y = cy / 100 * W if cy is not None else None
        m = comp.update(t, x, y, _f(r[IMU + "_ax_G"]), _f(r[IMU + "_ay_G"]))
        k = risk.update(t, m, {TENSION: _f(r[TENSION + "_N"])})
        out.append((t, m, k, r))
    return comp, risk, out


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    verbose = "-v" in sys.argv
    d = args[0] if args else os.path.join(os.path.dirname(__file__), "..", "acquisition", "logs")
    bed = (float(os.environ.get("BED_L", DEFAULT_BED_MM[0])),
           float(os.environ.get("BED_W", DEFAULT_BED_MM[1])))
    print("run     초  motion상태  r_th0   k    flag샘플 최대T_slip  등급(정상/주의/경고/위험)  사유")
    for f in sorted(glob.glob(os.path.join(d, "20*[0-9].csv"))):
        comp, risk, out = replay(f, bed_mm=bed)
        if not out:
            continue
        flags = sum(1 for _, m, _, _ in out if m.flag_slip)
        tmax = max(m.t_slip_s for _, m, _, _ in out)
        lv = Counter(k.level for _, _, k, _ in out)
        why = Counter(x.split(" (")[0].split(" 벨트 장력")[0] for _, _, k, _ in out for x in k.reasons)
        r_th0 = "%.2f" % comp.r_th0 if comp.r_th0 is not None else "-"
        print("%s %4.0f  %-9s %5s %5.2f %6d %8.1f   %s   %s" % (
            os.path.basename(f)[9:15], out[-1][0], comp.state, r_th0, comp.k, flags, tmax,
            "/".join(str(lv.get(i, 0)) for i in range(4)),
            dict(why.most_common(3)) if verbose else ""))


if __name__ == "__main__":
    main()
