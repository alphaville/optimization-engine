import casadi.casadi as cs
import numpy as np
from .constraint import Constraint
import opengen.functions as fn


class SecondOrderCone(Constraint):
    """A Second-Order Cone given by C = {u = (x, r): a||x|| <= r}

    Second-order cones are used in conic optimisation to describe
    inequalities that involve quadratic terms

    """

    def __init__(self, a: float = 1.0):
        """Constructor for a Second-Order Cone set

        Args:
            :param a: parameter a

        Returns:
            New instance of a SOC with given parameter `a`
        """
        if a <= 0:
            raise Exception("Parameter `a` must be a positive number")

        self.__a = a

    @property
    def a(self):
        """Returns the value of parameter `a`"""
        return self.__a

    def distance_squared(self, u):
        """Computes the squared distance between a given point `u` and this
           second-order cone

            :param u: given point; can be a list of float, a numpy
                n-dim array (`ndarray`) or a CasADi SX/MX symbol

            :return: distance from set as a float or a CasADi symbol
        """

        if fn.is_symbolic(u):
            # Case I: `u` is a CasADi SX symbol
            nu = u.size(1)
        elif (isinstance(u, list) and all(isinstance(x, (int, float)) for x in u)) \
                or isinstance(u, np.ndarray):
            nu = len(u)
        else:
            raise Exception("Illegal Argument, `u`")

        # Partition `u = (x, r)`, where `r` is the last element of `u`
        a = self.__a
        x = u[0:nu-1]
        r = u[nu-1]
        # norm of x
        norm_x = fn.norm2(x)
        # The first branch is zero
        # Second branch:
        condition2 = r <= -norm_x
        fun2 = cs.dot(x, x) + r ** 2
        # Third branch:
        condition3 = (-norm_x < r) * (norm_x > r)
        beta = a**2/(a**2 + 1.0)
        fun3 = norm_x ** 2 + beta * (a * norm_x + r) ** 2 \
               - 2.0 * a * norm_x * (a*norm_x + r) \
               + (r - (a * norm_x + r)/(a**2 + 1.0)) ** 2

        y = cs.if_else(condition2*(1-condition3), fun2, fun3)

        # Function defined piecewise
        # y = condition2 * fun2 + condition3 * fun3
        return y

    def project(self, u):
        # Idea: Computes projection on Ball as follows
        #   Proj_B(u) = u / max{1, ||u||},
        # which avoids dividing by zero or defining the projections
        raise NotImplementedError()

