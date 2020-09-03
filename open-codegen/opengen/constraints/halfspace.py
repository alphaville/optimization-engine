from .constraint import Constraint
import sys


class Halfspace(Constraint):
    """Halfspace

    A halfspace is a set of the form H = {c'x <= b}, where c is a given
    vector and b is a constant scalar.
    """

    def __init__(self, normal_vector, offset):
        """Construct a new halfspace H = {c'x <= b}

        :param normal_vector: vector c
        "param offset: parameter b
        """
        self.__normal_vector = [float(x) for x in normal_vector]
        self.__offset = offset

    @property
    def normal_vector(self):
        """
        Returns vector c
        :return:
        """
        return self.__normal_vector

    @property
    def offset(self):
        """
        Returns parameter b
        :return:
        """
        return self.__offset

    def dimension(self):
        """
        Dimension of the halfspace
        :return: length of normal vector
        """
        return len(self.__normal_vector)

    def project(self, u):
        raise NotImplementedError()

    def distance_squared(self, u):
        raise NotImplementedError()

    def is_convex(self):
        """
        A halfspace is a convex set
        :return:
        """
        return True

    def is_compact(self):
        """Whether the set is compact

        H is compact iff b < 0 and c = 0, in which case H is empty
        """
        eps = sys.float_info.epsilon
        # if b < 0 and c = 0, then H is empty, hence compact
        if self.offset < 0 and sum([self.normal_vector[idx]
                                    for idx in range(self.dimension())]) < eps:
            return True
        return False
