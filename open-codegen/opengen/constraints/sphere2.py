import casadi.casadi as cs
import numpy as np
from .constraint import Constraint
import opengen.functions as fn


class Sphere2(Constraint):
    """A Euclidean sphere constraint

    A constraint of the form :math:`\|u-u_0\| = r`, where :math:`u_0` is the center
    of the ball and `r` is its radius

    """

    def __init__(self, center=None, radius: float = 1.0):
        """Constructor for a Euclidean sphere constraint

        :param center: center of the sphere; if this is equal to Null, the
            sphere is centered at the origin

        :param radius: radius of the sphere

        :return: New instance of Sphere2 with given center and radius
        """
        if radius <= 0:
            raise Exception("The radius must be a positive number")

        if center is not None and not isinstance(center, (list, np.ndarray)):
            raise Exception("center is neither None nor a list nor np.ndarray")

        self.__center = None if center is None else np.array(
            [float(i) for i in center])
        self.__radius = float(radius)

    @property
    def center(self):
        """Returns the center of the ball"""
        return self.__center

    @property
    def radius(self):
        """Returns the radius of the sphere"""
        return self.__radius

    def distance_squared(self, u):
        """Computes the squared distance between a given point `u` and this sphere

            :param u: given point; can be a list of float, a numpy
                n-dim array (`ndarray`) or a CasADi SX/MX symbol

            :return: distance from set as a float or a CasADi symbol
        """
        if fn.is_symbolic(u):
            # Case I: `u` is a CasADi SX symbol
            v = u if self.__center is None else u - self.__center
        elif (isinstance(u, list) and all(isinstance(x, (int, float)) for x in u))\
                or isinstance(u, np.ndarray):
            if self.__center is None:
                v = u
            else:
                # Note: self.__center is np.ndarray (`u` might be a list)
                z = self.__center.reshape(len(u))
                u = np.array(u).reshape(len(u))
                v = np.subtract(u, z)
        else:
            raise Exception("u is of invalid type")

        return (self.__radius - fn.norm2(v))**2

    def project(self, u):
        raise NotImplementedError()

    def is_convex(self):
        return False

    def is_compact(self):
        return True
