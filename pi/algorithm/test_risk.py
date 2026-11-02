"""pi/ 에서:  python3 -m unittest algorithm.test_risk -v"""
import unittest

from algorithm.motion_comp import MotionResult
from algorithm.risk import RiskAssessor, RiskParams

DT = 0.1


def motion(flag=False, t_slip=0.0, r=0.0, state="ready"):
    return MotionResult(valid=True, state=state, x0=0, y0=0, cx=0, cy=0, cal_x=True, cal_y=True,
                        dx_cargo=0, dy_cargo=0, r=r, r_th=1.0, flag_slip=flag, t_slip_s=t_slip)


def settle(a, n=30, T=20.0, labels=("T1", "T2")):
    """기준 장력 T 로 n 샘플 돌려 기준을 확정한다."""
    res = None
    for i in range(n):
        res = a.update(i * DT, motion(), {l: T for l in labels})
    return res, n * DT


class RiskTest(unittest.TestCase):
    def test_normal(self):
        a = RiskAssessor()
        res, t = settle(a)
        self.assertEqual(res.level, 0)
        self.assertEqual(res.name, "정상")
        self.assertAlmostEqual(res.base_n["T1"], 20.0)

    def test_belt_loosening_levels(self):
        # (장력, 벨트 상태, 이동 없음일 때 등급). 일정값을 30샘플 유지해 평활 창을 채운다
        for T, belt, want in ((19.0, "정상", 0), (17.0, "약간 이완", 1),
                              (13.0, "많이 이완", 2), (9.0, "불량", 2)):
            a = RiskAssessor(RiskParams(hold_s=0.0))     # 계단 변화의 급이완 가산이 hold 로 남지 않게
            _, t = settle(a)
            for i in range(30):
                res = a.update(t + i * DT, motion(), {"T1": T, "T2": 20.0})
            self.assertEqual(res.belt_state, belt, T)
            self.assertEqual(res.level, want, T)

    def test_slip_suspected_then_confirmed(self):
        a = RiskAssessor()
        _, t = settle(a)
        ok = {"T1": 20.0, "T2": 20.0}
        r1 = a.update(t, motion(True, 0.5, 2.0), ok)
        self.assertEqual((r1.slip_state, r1.level), ("의심", 1))
        r2 = a.update(t + DT, motion(True, 2.0, 2.0), ok)
        self.assertEqual((r2.slip_state, r2.level), ("확정", 2))

    def test_confirmed_slip_with_loose_belt_is_danger(self):
        a = RiskAssessor()
        _, t = settle(a)
        for i in range(15):
            res = a.update(t + i * DT, motion(True, 3.0, 2.0), {"T1": 17.0, "T2": 20.0})
        self.assertEqual(res.level, 3)

    def test_large_displacement_is_danger_even_with_good_belt(self):
        a = RiskAssessor()
        _, t = settle(a)
        res = a.update(t, motion(True, 2.0, 8.0), {"T1": 20.0, "T2": 20.0})
        self.assertEqual(res.level, 3)

    def test_level_holds_then_drops(self):
        a = RiskAssessor(RiskParams(hold_s=1.0))
        _, t = settle(a)
        ok = {"T1": 20.0, "T2": 20.0}
        a.update(t, motion(True, 2.0, 2.0), ok)                 # 확정 -> 2
        t += DT
        res = a.update(t, motion(), ok)
        self.assertEqual(res.level, 2)                           # 바로는 안 내려감
        for i in range(15):
            res = a.update(t + (i + 1) * DT, motion(), ok)
        self.assertEqual(res.level, 0)

    def test_fast_loosening_escalates(self):
        a = RiskAssessor()
        _, t = settle(a)
        for i in range(20):                                      # 2초에 20 -> 12.4 N
            T = 20.0 - 0.4 * i
            res = a.update(t + i * DT, motion(), {"T1": T, "T2": 20.0})
        self.assertTrue(any("빠르게" in r for r in res.reasons))
        self.assertEqual(res.belt_state, "불량")                  # 많이 이완(2) 에서 한 단계 상승

    def test_unfastened(self):
        a = RiskAssessor()
        res = None
        for i in range(30):
            res = a.update(i * DT, motion(), {"T1": 20.0, "T2": 0.5})
        self.assertEqual(res.worst_belt, "T2")
        self.assertEqual(res.belt_state, "불량")
        self.assertEqual(res.level, 2)

    def test_no_tension_data(self):
        a = RiskAssessor()
        res = a.update(0.0, motion(), {"T1": None, "T2": None})
        self.assertIsNone(res.belt_state)
        self.assertIn("장력 데이터 없음", res.reasons)
        self.assertEqual(res.level, 0)

    def test_motion_not_ready_is_ignored(self):
        a = RiskAssessor()
        res = a.update(0.0, motion(True, 5.0, 9.0, state="learning"), {"T1": 20.0, "T2": 20.0})
        self.assertEqual(res.slip_state, "없음")

    def test_tension_only_without_motion(self):
        a = RiskAssessor()
        res = None
        for i in range(30):
            res = a.update(i * DT, None, {"T1": 20.0, "T2": 20.0})
        self.assertEqual(res.level, 0)

    def test_set_baseline(self):
        a = RiskAssessor()
        a.set_baseline({"T1": 40.0, "T2": 40.0})
        for i in range(15):
            res = a.update(i * DT, motion(), {"T1": 32.0, "T2": 40.0})
        self.assertEqual(res.worst_belt, "T1")
        self.assertEqual(res.level, 1)                           # 20% 감소: 약간 이완


if __name__ == "__main__":
    unittest.main()
