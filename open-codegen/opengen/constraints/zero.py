import casadi.casadi as cs
import numpy as np
from .constraint import Constraint
import opengen.functions as fn


class Zero(Constraint):
    """A Euclidean ball constraint

    A constraint of the form ||u-u0|| <= r, where u0 is the center
    of the ball and r is its radius

    """

    def __init__(self):
        """
        Constructor for set Z = {0}

        """

    def distance_squared(self, u):
        return fn.norm2_squared(u)

    def project(self, u):
        raise NotImplementedError()

    def is_convex(self):
        return True

    def is_compact(self):
        return True
