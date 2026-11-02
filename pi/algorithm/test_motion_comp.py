"""합성 데이터 검증. pi/ 에서:  python3 -m unittest algorithm.test_motion_comp -v

모델: 화물 무게중심 높이 H cm 이면 가속 a(G) 때 하중 무게중심이 H*a 만큼 밀린다 (C = H).
"""
import math
import unittest

import numpy as np

from algorithm.motion_comp import (MotionCompensator, MotionParams, calc_comp_coef,
                                   calc_residual)

H = 10.0        # cm/G
DT = 0.1        # 10Hz


def drive(comp, ax_fn, ay_fn, seconds, slip=None, lag=0, seed=0, t0=0.0, noise_cm=0.1):
    """slip = (시각, dx, dy): 그 시각부터 화물이 영구 이동. lag = 하중 응답이 가속보다 늦는 샘플 수."""
    rng = np.random.default_rng(seed)
    n = int(seconds / DT)
    ax_h, ay_h, out = [], [], []
    for i in range(n):
        t = t0 + i * DT
        ax, ay = ax_fn(t), ay_fn(t)
        ax_h.append(ax)
        ay_h.append(ay)
        axl = ax_h[max(0, i - lag)]
        ayl = ay_h[max(0, i - lag)]
        x = 5.0 + H * ayl + rng.normal(0, noise_cm)
        y = 8.0 + H * axl + rng.normal(0, noise_cm)
        if slip and t >= slip[0]:
            x += slip[1]
            y += slip[2]
        out.append(comp.update(t, x, y, ax + 0.02 + rng.normal(0, 0.01),
                               ay - 0.01 + rng.normal(0, 0.01)))
    return out


def zero(t):
    return 0.0


def corner(t):      # 선회: 좌우 가속 +-0.3G
    return 0.3 * math.sin(2 * math.pi * 0.5 * t)


def stop_go(t):     # 출발(+) 제동(-) 섞임
    return 0.3 * math.sin(2 * math.pi * 0.4 * t)


class CoefTest(unittest.TestCase):
    def test_gate_keeps_prev(self):
        rng = np.random.default_rng(1)
        n = 50
        ay = rng.normal(0, 0.02, n)           # 유효 가속 없음
        ax = rng.normal(0, 0.02, n)
        dX = rng.normal(0, 0.1, n)
        c = calc_comp_coef(dX, dX, ax, ay, prev=(3.0, 4.0))
        self.assertEqual(c, (3.0, 4.0))

    def test_signs_do_not_cancel(self):
        a = np.array([0.3, -0.3] * 10)        # 합이 0 이어도 최소제곱은 값이 나온다
        c = calc_comp_coef(H * a, H * a, a, a)
        self.assertAlmostEqual(c[0], H)
        self.assertAlmostEqual(c[1], H)

    def test_residual_zero_when_model_exact(self):
        r = calc_residual(5 + H * 0.2, 8 + H * 0.1, 5, 8, 0.1, 0.2, H, H)
        self.assertAlmostEqual(float(r), 0.0)


class CompensatorTest(unittest.TestCase):
    def run_scene(self, ax_fn, ay_fn, seconds=25, **kw):
        comp = MotionCompensator()
        rest = drive(comp, zero, zero, 3.0, seed=10)
        self.assertEqual(rest[-1].state, "learning")
        move = drive(comp, ax_fn, ay_fn, seconds, t0=3.0, seed=11, **kw)
        return comp, move

    def test_coef_recovered_and_no_false_alarm(self):
        comp, move = self.run_scene(stop_go, corner)
        self.assertTrue(comp.cal_x and comp.cal_y)
        self.assertAlmostEqual(comp.cx, H, delta=1.0)
        self.assertAlmostEqual(comp.cy, H, delta=1.0)
        self.assertEqual(move[-1].state, "ready")
        self.assertFalse(any(m.flag_slip for m in move))

    def test_lagged_load_response_no_sustained_flag(self):
        comp, move = self.run_scene(stop_go, corner, lag=1)
        self.assertEqual(max(m.t_slip_s for m in move), 0.0)

    def test_slip_detected_and_coef_not_polluted(self):
        comp, move = self.run_scene(stop_go, corner, seconds=40, slip=(20.0, 3.0, 0.0))
        flags = [m.flag_slip for m in move]
        self.assertFalse(any(flags[:int((20.0 - 3.0) / DT)]))      # 이동 전엔 조용
        self.assertTrue(flags[-1])
        self.assertGreater(move[-1].t_slip_s, 10.0)
        self.assertAlmostEqual(comp.cx, H, delta=1.5)               # 슬립이 계수에 안 섞임
        self.assertGreater(move[-1].dx_cargo, 2.0)

    def test_slip_detected_while_idle(self):
        comp, move = self.run_scene(stop_go, corner, seconds=15)    # 먼저 계수 학습
        idle = drive(comp, zero, zero, 6.0, t0=28.0, seed=12, slip=(29.0, 2.0, 2.0))
        self.assertTrue(idle[-1].flag_slip)

    def test_single_axis_leaves_other_uncalibrated(self):
        comp, move = self.run_scene(stop_go, zero, seconds=15)      # 직진 가감속만
        self.assertTrue(comp.cal_y)
        self.assertFalse(comp.cal_x)
        self.assertEqual(comp.cx, 0.0)

    def test_missing_input_is_skipped(self):
        comp = MotionCompensator()
        res = comp.update(0.0, None, 1.0, 0.0, 0.0)
        self.assertFalse(res.valid)
        self.assertEqual(res.state, "rest")

    def test_rest_waits_until_still(self):
        comp = MotionCompensator(MotionParams(rest_n=10))
        res = drive(comp, zero, lambda t: 0.5 * math.sin(40 * t), 2.0, seed=3)
        self.assertEqual(res[-1].state, "rest")


if __name__ == "__main__":
    unittest.main()
