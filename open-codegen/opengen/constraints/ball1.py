from .constraint import Constraint
from .simplex import Simplex
import numpy as np


class Ball1(Constraint):
    """Ball1 aka Norm-1 Ball

    """

    def __init__(self, radius: float = 1.0):  # unless specified, alpha=1
        """Constructor for a Ball1

        Args:
            radius: ball radius (default: 1)

        Returns:
            New instance of Ball1 with given radius
        """
        if radius <= 0:
            raise ValueError("radius must be a positive number")

        self.__radius = float(radius)

    @property
    def radius(self):
        """Returns the radius of this ball"""
        return self.__radius

    def distance_squared(self, u):
        raise NotImplementedError()

    def project(self, u):
        if np.linalg.norm(u, 1) <= self.__radius:
            return u
        else:
            n = len(u)
            simplex = Simplex(self.__radius)
            x = simplex.project(abs(u))
            z = np.zeros(n)
            for i, (ui, xi) in enumerate(zip(u, x)):
                z[i] = np.sign(ui) * xi
            return z

    def is_convex(self):
        return True

    def is_compact(self):
        return True
