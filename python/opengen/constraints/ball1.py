from .constraint import Constraint
from .simplex import Simplex
import numpy as np


class Ball1(Constraint):
    """Ball1 aka Norm-1 Ball

    Ball-1 with given radius, that is

    :math:`\mathcal{B}_{1}(x_0, r) = \{x\in{\\rm I\!R}^n {}:{} \|x - x_0\|_1 \leq r\}`

    """

    def __init__(self, center=None, radius: float = 1.0, ):  # unless specified, radius=1
        """Constructor for a Ball1

        :param radius: ball radius (default: 1)

        :return: New instance of Ball1 with given radius
        """
        if radius <= 0:
            raise ValueError("radius must be a positive number")

        if center is not None and not isinstance(center, (list, np.ndarray)):
            raise Exception("center is neither None nor a list nor np.ndarray")

        self.__radius = float(radius)
        self.__simplex = Simplex(radius)
        self.__center = None if center is None else np.array(
            [float(xi) for xi in center])

    @property
    def center(self):
        """Returns the center of the ball"""
        return self.__center

    @property
    def radius(self):
        """Returns the radius of this ball"""
        return self.__radius

    def distance_squared(self, u):
        raise NotImplementedError()

    def __proj_on_ball_centered_at_origin(self, u):
        u = np.array(u)
        if np.linalg.norm(u, 1) <= self.radius:
            return u

        n = len(u)
        x = self.__simplex.project(abs(u))
        z = np.zeros(n)
        for i, (ui, xi) in enumerate(zip(u, x)):
            z[i] = np.sign(ui) * xi
        return z

    def project(self, u):
        """Project on the current Ball-1

        :param u: vector u

        :return: projection of u onto the current ball-1

        """
        if self.center is None:
            return self.__proj_on_ball_centered_at_origin(u)

        return self.__proj_on_ball_centered_at_origin(u - self.center) + self.center

    def is_convex(self):
        return True

    def is_compact(self):
        return True
