import unittest
import casadi.casadi as cs
import opengen as og
import numpy as np


class ConstraintsTestCase(unittest.TestCase):

    def test_ball_inf_origin(self):
        ball = og.constraints.BallInf(None, 1)
        x = np.array([3, 2])
        x_sym = cs.SX.sym("x", 2)
        d_num = ball.distance_squared(x)
        d_sym = float(cs.substitute(ball.distance_squared(x_sym), x_sym, x))
        self.assertAlmostEqual(d_sym, d_num, 8, "computation of distance")
        correct_squared_distance = 5
        self.assertAlmostEqual(d_num, correct_squared_distance,
                               8, "expected squared distance")

    def test_ball_inf_origin_inside(self):
        ball = og.constraints.BallInf(None, 1)
        x = np.array([0.1, -0.2])
        x_sym = cs.SX.sym("x", 2)
        d_num = ball.distance_squared(x)
        d_sym = float(cs.substitute(ball.distance_squared(x_sym), x_sym, x))
        self.assertAlmostEqual(d_sym, d_num, 10, "computation of distance")
        correct_squared_distance = 0
        self.assertAlmostEqual(d_num, correct_squared_distance,
                               10, "expected squared distance")

    def test_ball_inf_xc(self):
        ball = og.constraints.BallInf([-1, -1], 0.5)
        x = np.array([1, -2])
        x_sym = cs.SX.sym("x", 2)
        d_num = ball.distance_squared(x)
        d_sym = float(cs.substitute(ball.distance_squared(x_sym), x_sym, x))
        self.assertAlmostEqual(d_sym, d_num, 8, "computation of distance")
        correct_squared_distance = 2.5
        self.assertAlmostEqual(d_num, correct_squared_distance,
                               8, "expected squared distance")

    def test_ball_euclidean_origin(self):
        ball = og.constraints.Ball2(None, 1)
        x = np.array([2, 2])
        x_sym = cs.SX.sym("x", 2)
        d_num = ball.distance_squared(x)
        d_sym = float(cs.substitute(ball.distance_squared(x_sym), x_sym, x))
        self.assertAlmostEqual(d_sym, d_num, 8, "computation of distance")
        correct_squared_distance = 8*(1-np.sqrt(2)/4)**2
        self.assertAlmostEqual(d_sym, correct_squared_distance,
                               8, "expected squared distance")

    def test_ball_euclidean_origin_inside(self):
        ball = og.constraints.Ball2(None, 1)
        x = np.array([0.2, 0.8])
        x_sym = cs.SX.sym("x", 2)
        d_num = ball.distance_squared(x)
        d_sym = float(cs.substitute(ball.distance_squared(x_sym), x_sym, x))
        self.assertAlmostEqual(d_sym, d_num, 8, "computation of distance")
        correct_squared_distance = 0.0
        self.assertAlmostEqual(d_sym, correct_squared_distance,
                               8, "expected squared distance")

    def test_ball_euclidean_xc(self):
        ball = og.constraints.Ball2([1, 2], 1)
        x = [0, 1]
        x_sym = cs.SX.sym("x", 2)
        d_num = ball.distance_squared(x)
        d_sym = cs.substitute(ball.distance_squared(x_sym), x_sym, x)
        self.assertAlmostEqual(d_sym, d_num, 8, "computation of distance")
        correct_squared_distance = (np.sqrt(2) - 1) ** 2
        self.assertAlmostEqual(d_sym, correct_squared_distance,
                               8, "expected squared distance")

    def test_ball_euclidean_xc_inside(self):
        ball = og.constraints.Ball2([1, 2], 1)
        x = [1.2, 1.55]
        x_sym = cs.SX.sym("x", 2)
        d_num = ball.distance_squared(x)
        d_sym = cs.substitute(ball.distance_squared(x_sym), x_sym, x)
        self.assertAlmostEqual(d_sym, d_num, 8, "computation of distance")
        correct_squared_distance = 0.0
        self.assertAlmostEqual(d_sym, correct_squared_distance,
                               8, "expected squared distance")

    def test_rectangle_simple(self):
        rect = og.constraints.Rectangle([-1, -2], [4, -1])
        # some basic assertions
        self.assertListEqual([0, 1], rect.idx_bound_finite_all())
        self.assert_(len(rect.idx_infinite_only_xmax()) == 0)
        self.assertTrue(len(rect.idx_infinite_only_xmin()) == 0)
        self.assertEqual(2, rect.dimension())
        # squared distance
        self.assertAlmostEqual(1, rect.distance_squared([3, 0]), 8)
        self.assertAlmostEqual(4, rect.distance_squared([0, 1]), 8)
        self.assertAlmostEqual(1, rect.distance_squared([5, -1.5]), 8)
        self.assertAlmostEqual(5, rect.distance_squared([5, 1]), 8)
        # symbolic
        x_sym = cs.SX.sym("x", 2)
        d_sym = float(cs.substitute(rect.distance_squared(x_sym), x_sym, [5, 1]))
        self.assertAlmostEqual(5, d_sym, 8)

    def test_rectangle_pos_quant(self):
        n = 3
        rect = og.constraints.Rectangle([0.0]*n, None)
        # some basic assertions
        self.assertTrue(0 == len(rect.idx_bound_finite_all()))
        self.assertTrue(0 == len(rect.idx_infinite_only_xmin()))
        self.assertEqual([*range(n)], rect.idx_infinite_only_xmax())
        # some squared distances
        self.assertAlmostEqual(0.0, rect.distance_squared([0.0]*n), 8)
        self.assertAlmostEqual(0.0, rect.distance_squared([1.0] * n), 8)
        self.assertAlmostEqual(1.0, rect.distance_squared([-1.0] + [1.0] * (n-1)), 8)
        self.assertAlmostEqual(5.0, rect.distance_squared([-1.0, -2.0, 5.0]), 8)

    def test_rectangle_semiinf_corridor(self):
        rect = og.constraints.Rectangle([-1.0, -2.0], [float('inf'), 3.0])
        self.assertEqual([0], rect.idx_infinite_only_xmax())
        self.assertAlmostEqual(0.0, rect.distance_squared([1e16, 1.5]), 8)
        self.assertAlmostEqual(1.0, rect.distance_squared([1e16, 4.0]), 8)
        self.assertAlmostEqual(4.0, rect.distance_squared([1e16, -4.0]), 8)


if __name__ == '__main__':
    unittest.main()
