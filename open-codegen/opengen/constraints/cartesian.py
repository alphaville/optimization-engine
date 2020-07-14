from . import constraint
from typing import List


class CartesianProduct(constraint.Constraint):

    def __init__(self, segments: List[int], constraints: List[constraint.Constraint]):
        """
        Construct a Cartesian product of constraints by providing a list of sets
        and their dimensions as follows: an n-dimensional vector x can be partitioned
        into subvectors as x = (x1, x2, ..., xs), where each xi has dimension mi.
        For example consider the 5-dimensional vector x = (x_0, x_1, x_2, x_3, x_4),
        which can be partitioned into x1 = (x_0, x_1) and x2 = (x_2, x_3, x_4).
        We can associate with x1 the indices [0, 1] and with x2 the indices [2, 3, 4].
        The *segment ids* are the indices 1 and 4.

        Example:
        > ball = og.constraints.Ball2(None, 1.5)
        > rect = og.constraints.Rectangle(xmin=[-1,-2,-3], xmax=[0, 10, -1])
        > free = og.constraints.NoConstraints()
        > segment_ids = [1, 4, 9]
        > my_set = og.constraints.CartesianProduct(segment_ids, [ball, rect])

        :param segments: ids of segments
        :param constraints: list of sets
        """
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

    def is_compact(self):
        for set_i in self.__constraints:
            if not set_i.is_compact():
                return False
        return True
