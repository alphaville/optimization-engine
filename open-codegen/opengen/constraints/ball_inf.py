import casadi.casadi as cs
import numpy as np
from .constraint import Constraint
import opengen.functions as fn


class BallInf(Constraint):
    """Norm-ball of norm infinity translated by given vector

    Centered inf-ball around given point
    """

    def __init__(self, center=None, radius: float = 0.0):
        """Constructor for an infinity ball constraint

        Args:
            :param center: center of the ball; if this is equal to Null, the
                ball is centered at the origin

            :param radius: radius of the ball

        :return:
            New instance of Ballinf with given center and radius
        """

        if radius <= 0:
            raise Exception("The radius must be a positive number")

        if center is not None and not (isinstance(center, list)
                                       or isinstance(center, np.ndarray)):
            raise Exception("center is neither None nor a list nor np.ndarray")

        self.__center = None if center is None else np.array([float(i) for i in center])
        self.__radius = float(radius)

    @property
    def center(self):
        """Returns the center of the ball"""
        return self.__center

    @property
    def radius(self):
        """Returns the radius of the ball"""
        return self.__radius

    def norm_inf_fun_np(a):
        return np.linalg.norm(a, ord=np.inf)

    def distance_squared(self, u):
        # Function `distance` can be applied to CasADi symbols and
        # lists of numbers. However, if `u` is a symbol, we need to
        # use appropriate CasADi functions like cs.sign and cs.norm_2
        if fn.is_symbolic(u):
            # Case I: `u` is a CasADi SX symbol
            nu = u.size(1)
            v = u if self.__center is None else u - self.__center
        elif (isinstance(u, list) and all(isinstance(x, (int, float)) for x in u)) \
                or isinstance(u, np.ndarray):
            # Case II: `u` is an array of numbers or an np.ndarray
            nu = len(u)
            if self.__center is None:
                v = u
            else:
                # Note: self.__center is np.ndarray (`u` might be a list)
                z = self.__center.reshape(nu)
                u = np.array(u).reshape(nu)
                v = np.subtract(u, z)
        else:
            raise Exception("u is of invalid type")

        # Compute distance to Ball infinity:
        # dist^2(u) = norm(v)^2
        #            + SUM_i [
        #                         min{vi^2, r^2}
        #                       - 2*min{vi^2, r*|vi|}
        #                    ]
        # where v = u - xc

        squared_distance = fn.norm2(v)**2
        for i in range(nu):
            squared_distance += fn.fmin(v[i]**2, self.radius**2) \
                                - 2.0 * fn.fmin(v[i]**2, self.radius * fn.fabs(v[i]))
        return squared_distance

    def project(self, u):
        # Computes projection on infinity Ball
        raise NotImplementedError("Method `project` is not implemented")

    def is_convex(self):
        return True

    def is_compact(self):
        return True
