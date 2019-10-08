import casadi.casadi as cs
import numpy as np
from .constraint import Constraint
import opengen.functions as fn


class FiniteSet(Constraint):
    """Finite sets
    """

    def __init__(self, points=None):
        """Constructor for a finite set
        """
        if points is not None and len(points) > 0:
            first_point_len = len(points[0])
            for point in points[1:]:
                point_len = len(point)
                if point_len != first_point_len:
                    raise Exception("Invalid input (points have unequal dimensions)")
        self.__points = None if points is None else [[float(x) for x in p] for p in points]

    @property
    def points(self):
        return self.__points

    def dimension(self):
        p = self.points
        if p is None or len(p) == 0:
            return 0
        else:
            return len(p[0])

    def cardinality(self):
        p = self.points
        if p is None:
            return 0
        else:
            return len(p)

    def distance_squared(self, u):
        raise NotImplementedError()

    def project(self, u):
        raise NotImplementedError()

    def is_convex(self):
        return self.cardinality() == 1 and self.dimension() > 0

    def is_compact(self):
        return True
