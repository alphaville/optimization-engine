from ..constraints.rectangle import Rectangle
from ..constraints.ball_inf import BallInf
from ..constraints.halfspace import Halfspace


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
        n = c.dimension()

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
        err_msg = 'The set C you chose is not supported yet ' \
                  '(report at https://github.com/alphaville/optimization-engine/issues)'
        if isinstance(self.__set_c, Rectangle):
            return self.__obtain_y_with_c_rectangle()
        if self.__set_c.is_compact():
            return self.__obtain_y_with_c_compact()
        if isinstance(self.__set_c, Halfspace):
            err_msg = 'For the time being, you cannot use a Halfspace in set C (issue #197)'

        raise NotImplementedError(err_msg)
