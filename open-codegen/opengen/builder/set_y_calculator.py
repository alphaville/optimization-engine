from ..constraints.rectangle import *
from ..constraints.ball_inf import *

class SetYCalculator:

    LARGE_NUM = 1e12

    def __init__(self, set_c):
        self.__set_c = set_c

    def __obtain_y_with_c_compact(self):
        return BallInf(None, SetYCalculator.LARGE_NUM)

    def __obtain_y_with_c_rectangle(self):
        c = self.__set_c
        xmin = c.xmin
        xmax = c.xmax
        if xmin is not None:
            n = len(xmin)
        elif xmax is not None:
            n = len(xmax)
        else:
            raise Exception("Fatal error: both xmin and xmax are None")

        if xmin is None:
            ymin = [0.0] * n
        else:
            ymin = [-SetYCalculator.LARGE_NUM] * n
            for i in range(n):
                if xmin[i] == float('-inf'):
                    ymin[i] = 0.0

        if xmax is None:
            ymax = [0.0] * n
        else:
            ymax = [SetYCalculator.LARGE_NUM] * n
            for i in range(n):
                if xmax[i] == float('inf'):
                    ymax[i] = 0.0

        return Rectangle(ymin, ymax)

    def obtain(self):
        if isinstance(self.__set_c, Rectangle):
            return self.__obtain_y_with_c_rectangle()
        elif self.__set_c.is_compact():
            return self.__obtain_y_with_c_compact()
        else:
            raise NotImplementedError()
