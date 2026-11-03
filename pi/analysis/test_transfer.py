"""pi/ 에서:  python3 -m unittest analysis.test_transfer -v

h 학습 후 고정 (COG_H_FREEZE): 가감속 장면을 만들어 TransferCheck 에 넣는다.
무게중심은 판 가운데(125, 196 mm)에서 h x a 만큼 옮겨진다 (X <- ay, Y <- ax)."""
import unittest
from types import SimpleNamespace

from analysis.cog import CoG
from analysis.transfer import TransferCheck

DT = 0.1
BED = (250.0, 392.0)
X0, Y0 = 125.0, 196.0


def cfg(**kw):
    base = dict(cog_h=190.0, cog_lag=0.0, cog_resid=7.0, cog_resid_a=20.0, cog_h_learn=1,
                move_win_s=1.0, move_alpha=0.6, move_confirm_s=1.5, move_danger_s=3.0)
    base.update(kw)
    return SimpleNamespace(**base)


def accel(t):
    """4초마다 1초 0.25G 제동(ax), 6초마다 2초 0.25G 좌회전(ay, 회전 20°/s)."""
    ax = 0.25 if t % 4.0 >= 3.0 else 0.0
    ay = 0.25 if t % 6.0 >= 4.0 else 0.0
    return ax, ay, 20.0 if ay else 0.0


def drive(tc, t0, seconds, h_x, h_y=None):
    """좌우 h_x, 앞뒤 h_y(없으면 h_x)인 화물로 seconds 초 달린다. -> 마지막 시각"""
    h_y = h_x if h_y is None else h_y
    t = t0
    for i in range(int(seconds / DT)):
        t = t0 + i * DT
        ax, ay, gz = accel(t)
        tc(t, CoG(X0 + h_x * ay, Y0 + h_y * ax, "mm"), SimpleNamespace(ax=ax, ay=ay, gz=gz))
    return t + DT


class FreezeTest(unittest.TestCase):
    def test_freezes_near_true_h(self):
        tc = TransferCheck(cfg(), BED)
        tc.freeze = True
        drive(tc, 0.0, 30.0, 120.0)
        self.assertIsNotNone(tc.x.fixed)
        self.assertIsNotNone(tc.y.fixed)
        self.assertAlmostEqual(tc.x.k(), 120.0, delta=10.0)
        self.assertAlmostEqual(tc.y.k(), 120.0, delta=10.0)

    def test_frozen_h_does_not_absorb_slide(self):
        """고정한 뒤 커브마다 허용 안에서 바깥으로 더 밀리면 (h 170 처럼 보임): 고정은 그대로, 계속 학습은 따라간다."""
        ks = {}
        for freeze in (True, False):
            tc = TransferCheck(cfg(), BED)
            tc.freeze = freeze
            t = drive(tc, 0.0, 30.0, 120.0)
            drive(tc, t, 60.0, 170.0, 120.0)
            ks[freeze] = tc.x.k()
        self.assertAlmostEqual(ks[True], 120.0, delta=10.0)
        self.assertGreater(ks[False], 150.0)

    def test_rebase_unfreezes(self):
        tc = TransferCheck(cfg(), BED)
        tc.freeze = True
        drive(tc, 0.0, 30.0, 120.0)
        tc.rebase()
        self.assertIsNone(tc.x.fixed)
        self.assertIsNone(tc.y.fixed)
        self.assertAlmostEqual(tc.x.k(), 190.0)

    def test_no_freeze_when_off(self):
        tc = TransferCheck(cfg(), BED)
        drive(tc, 0.0, 30.0, 120.0)
        self.assertIsNone(tc.x.fixed)
        self.assertIsNone(tc.y.fixed)
        self.assertAlmostEqual(tc.x.k(), 120.0, delta=10.0)


if __name__ == "__main__":
    unittest.main()
