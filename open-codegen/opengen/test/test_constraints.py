import opengen as og
import unittest
import casadi.casadi as cs
import numpy as np
import math


class ConstraintsTestCase(unittest.TestCase):

    # -----------------------------------------------------------------------
    # Infinity Ball
    # -----------------------------------------------------------------------

    def test_ball_inf_origin(self):
        ball = og.constraints.BallInf(None, 1)
        x = np.array([3, 2])
        x_sym = cs.SX.sym("x", 2)
        d_num = ball.distance_squared(x)
        d_sym = float(cs.substitute(ball.distance_squared(x_sym), x_sym, x))
        self.assertAlmostEqual(d_sym, d_num, 8, "computation of distance")
        correct_squared_distance = 5.0
        self.assertAlmostEqual(d_num, correct_squared_distance,
                               8, "expected squared distance")
        # verify that it works with cs.MX
        x_sym_mx = cs.MX.sym("xmx", 2)
        sqdist_mx = ball.distance_squared(x_sym_mx)
        sqdist_mx_fun = cs.Function('sqd', [x_sym_mx], [sqdist_mx])
        self.assertAlmostEqual(correct_squared_distance, sqdist_mx_fun(x)[0], 5)

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

    # -----------------------------------------------------------------------
    # Euclidean Ball
    # -----------------------------------------------------------------------

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

    def test_ball_euclidean_origin_3d(self):
        ball = og.constraints.Ball2(None, 1)
        x = np.array([1, 1, 1])
        d_num = ball.distance_squared(x)
        correct_squared_distance = 0.535898384862246
        self.assertAlmostEqual(correct_squared_distance, d_num, 12, "computation of distance")

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

    # -----------------------------------------------------------------------
    # Rectangle
    # -----------------------------------------------------------------------

    def test_rectangle_simple(self):
        rect = og.constraints.Rectangle([-1, -2], [4, -1])
        # some basic assertions
        self.assertListEqual([0, 1], rect.idx_bound_finite_all())
        self.assertTrue(len(rect.idx_infinite_only_xmax()) == 0)
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

    # -----------------------------------------------------------------------
    # Second-Order Cone (SOC)
    # -----------------------------------------------------------------------

    def test_second_order_cone(self):
        soc = og.constraints.SecondOrderCone(2.0)

        # dist_C^2(0, 0, 0) = 0 [origin is in the cone]
        sq_dist = soc.distance_squared([0.0, 0.0, 0.0])
        self.assertAlmostEqual(0, sq_dist, 16)

        # dist_C^2(0, 0, 0) = 0 [close-origin]
        sq_dist = soc.distance_squared([1e-12, 1e-12, 1e-12])
        self.assertAlmostEqual(0, sq_dist, 16)

        # dist_C^2(1, 1, 0.75) = 0 [case II]
        sq_dist = soc.distance_squared([1.0, 1.0, 0.75])
        self.assertAlmostEqual(0, sq_dist, 16)

        # dist_C^2(3, 4, -11) = 146.0 [case II]
        sq_dist = soc.distance_squared([3.0, 4.0, -11.0])
        self.assertAlmostEqual(146.0, sq_dist, 16)
        sq_dist = soc.distance_squared([4.0, 3.0, -11.0])
        self.assertAlmostEqual(146.0, sq_dist, 16)

        # dist_C^2(2, 3, 0.5) = 1.357... [case III]
        sq_dist = soc.distance_squared([2.0, 3.0, 0.5])
        self.assertAlmostEqual(1.35777948981440, sq_dist, 12)

    def test_second_order_cone_symbolic(self):
        soc = og.constraints.SecondOrderCone(2.0)
        u = cs.SX.sym('u', 3, 1)
        sq_dist = soc.distance_squared(u)
        u0 = [4.0, 3.0, -11.0]

        sq_dist_sx_fun = cs.Function('sqd1', [u], [sq_dist])
        self.assertAlmostEqual(146.0, sq_dist_sx_fun(u0), 16)

        umx = cs.MX.sym('u', 3, 1)
        sq_dist_m2 = soc.distance_squared(u)
        sq_dist_mx_fun = cs.Function('sqd2', [u], [sq_dist_m2])
        self.assertAlmostEqual(146.0, sq_dist_mx_fun(u0), 16)

    def test_second_order_cone_jacobian(self):
        soc = og.constraints.SecondOrderCone()
        # Important note: the second-order cone constraint does not work with cs.MX
        #                 An exception will be raised if the user provides an SX
        u = cs.MX.sym('u', 3)
        sq_dist = soc.distance_squared(u)
        sq_dist_jac = cs.jacobian(sq_dist, u)
        sq_dist_jac_fun = cs.Function('sq_dist_jac', [u], [sq_dist_jac])
        v = sq_dist_jac_fun([0., 0., 0.])
        for i in range(3):
            self.assertFalse(math.isnan(v[i]), "v[i] is NaN")
        self.assertAlmostEqual(0, cs.norm_2(v), 12)

    # -----------------------------------------------------------------------
    # No Constraints
    # -----------------------------------------------------------------------

    def test_no_constraints(self):
        whole_rn = og.constraints.NoConstraints()
        u = [1., 2., 3., 4.]
        self.assertAlmostEqual(0.0, whole_rn.distance_squared(u), 16)
        self.assertListEqual(u, whole_rn.project(u))

    # -----------------------------------------------------------------------
    # Cartesian product of constraints
    # -----------------------------------------------------------------------

    def test_cartesian(self):
        inf = float('inf')
        ball_inf = og.constraints.BallInf(None, 1)
        ball_eucl = og.constraints.Ball2(None, 1)
        rect = og.constraints.Rectangle(xmin=[0.0, 1.0, -inf, 2.0],
                                        xmax=[1.0, inf, 10.0, 10.0])
        # Segments:
        # [0, 1]
        # [2, 3, 4]
        # [5, 6, 7, 8]
        cartesian = og.constraints.CartesianProduct(9, [1, 4, 8], [ball_inf, ball_eucl, rect])
        sq_dist = cartesian.distance_squared([5, 10, 1, 1, 1, 0.5, -1, 0, 11])
        correct_sq_distance = 102.0 + (math.sqrt(3)-1.0)**2
        self.assertAlmostEqual(correct_sq_distance, sq_dist, 12)

    def test_cartesian_sx(self):
        inf = float('inf')
        ball_inf = og.constraints.BallInf(None, 1)
        ball_eucl = og.constraints.Ball2(None, 1)
        rect = og.constraints.Rectangle(xmin=[0.0, 1.0, -inf, 2.0],
                                        xmax=[1.0, inf, 10.0, 10.0])
        cartesian = og.constraints.CartesianProduct(9, [1, 4, 8], [ball_inf, ball_eucl, rect])
        u_sx = cs.SX.sym("u", 9, 1)
        _sqd_sx = cartesian.distance_squared(u_sx)
        u_mx = cs.SX.sym("u", 9, 1)
        _sqd_mx = cartesian.distance_squared(u_mx)

    # -----------------------------------------------------------------------
    # Finite Set
    # -----------------------------------------------------------------------
    def test_finite_set_dim_card(self):
        c = og.constraints.FiniteSet()
        self.assertEqual(0, c.dimension())
        self.assertEqual(0, c.cardinality())

        c = og.constraints.FiniteSet([])
        self.assertEqual(0, c.dimension())
        self.assertEqual(0, c.cardinality())

        c = og.constraints.FiniteSet([[1,2,3], [4,5,6]])
        self.assertEqual(3, c.dimension())
        self.assertEqual(2, c.cardinality())

    def test_finite_set_fail(self):
        with self.assertRaises(Exception) as __context:
            og.constraints.FiniteSet([[1., 2.], [1., 2., 3.]])

    # -----------------------------------------------------------------------
    # Set Y (from C)
    # -----------------------------------------------------------------------


if __name__ == '__main__':
    unittest.main()
