import casadi.casadi as cs
import numpy as np
from .constraint import Constraint
import opengen.functions as fn
from typing import List


class CartesianProduct(Constraint):

    def __init__(self, dimension: int, segments: List[int], constraints: List[Constraint]):
        # TODO: Check whether input arguments are valid (types, ranges, etc)
        self.__dimension = dimension
        self.__segments = segments
        self.__constraints = constraints

    @property
    def constraints(self):
        return self.__constraints

    @property
    def segments(self):
        return self.__segments

    def segment_dimension(self, i):
        if i == 0:
            return self.__segments[0] + 1
        else:
            return self.__segments[i] - self.__segments[i-1]

    def distance_squared(self, u):
        squared_distance = 0.0
        num_segments = len(self.__segments)
        idx_previous = -1
        for i in range(num_segments):
            idx_current = self.__segments[i]
            ui = u[idx_previous+1:idx_current+1]
            current_sq_dist = self.__constraints[i].distance_squared(ui)
            idx_previous = idx_current
            squared_distance += current_sq_dist

        return squared_distance

    def project(self, u):
        raise NotImplementedError()

    def is_convex(self):
        flag = True
        for c in self.__constraints:
            flag &= c.is_convex()
        return flag

