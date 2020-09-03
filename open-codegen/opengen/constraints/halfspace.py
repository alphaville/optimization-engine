from .constraint import Constraint


class Halfspace(Constraint):
    """Halfspace

    A halfspace is a set of the form H = {c'x <= b}, where c is a given
    vector and b is a constant scalar.


    """

    def __init__(self, normal_vector, offset):
        self.__normal_vector = [float(x) for x in normal_vector]
        self.__offset = offset

    @property
    def normal_vector(self):
        return self.__normal_vector

    @property
    def offset(self):
        return self.__offset

    def dimension(self):
        return len(self.__normal_vector)

    def project(self, u):
        raise NotImplementedError()

    def distance_squared(self, u):
        raise NotImplementedError()

    def is_convex(self):
        return True

    def is_compact(self):
        return False
