import casadi.casadi as cs
import numpy as np
from .constraint import Constraint
import opengen.functions as fn


class CartesianProduct(Constraint):

    def __init__(self):
        pass

    def distance_squared(self, u):
        raise NotImplementedError()

    def project(self, u):
        raise NotImplementedError()
