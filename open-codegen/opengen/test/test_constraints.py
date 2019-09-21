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

if __name__ == '__main__':
    unittest.main()
