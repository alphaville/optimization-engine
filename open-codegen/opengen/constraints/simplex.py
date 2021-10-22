from .constraint import Constraint
import numpy as np


class Simplex(Constraint):
    """Simplex constraint

    A constraint of the form sum(y) = a, where y is the input vector
    and a is the simplex value alpha. The simplex constraint set is
    defined by alpha and the length of the input vector.

    """

    def __init__(self, alpha: float = 1.0):  # unless specified, alpha is unit vector
        """Constructor for a Simplex constraint

        Args:
            alpha: unit vector multiplier

        Returns:
            New instance of Simplex with given alpha.
        """
        if alpha <= 0:
            raise Exception("Alpha must be a positive number")

        self.__alpha = float(alpha)

    @property
    def alpha(self):
        """Returns the simplex value alpha"""
        return self.__alpha

    def distance_squared(self, u):
        raise NotImplementedError()

    def project(self, y):
        """Computes the projection of a given point `y` (in R^n) on the simplex defined by n and alpha.

            :param y: given point; must be a list of numbers (floats, ints)
            ?? at the moment can only take a list ????????????????????????????a numpy n-dim array (`ndarray`) or a CasADi SX/MX symbol

            :return: the projection point in R^n as a numpy array of float64s
        """
        def __pop_all(z, indices):
            for index in sorted(indices, reverse=True):
                del z[index]
            return z

        # inputs ----
        len_y, a = len(y), self.__alpha

        # 1 ----
        v = [y[0]]
        v_size_old = -1
        v_tilda = []
        rho = y[0] - a

        # 2 ----
        for yn in y[1:]:
            if yn > rho:
                rho += (yn - rho) / (len(v) + 1)
                if rho > yn - a:
                    v.append(yn)
                else:
                    v_tilda.extend(v)
                    v = [yn]
                    rho = yn - a

        # 3 ----
        if len(v_tilda) > 0:
            for v_tilda_i in v_tilda:
                if v_tilda_i > rho:
                    v.append(v_tilda_i)
                    rho += (v_tilda_i - rho) / len(v)

        # 4 ----
        keep_running = True
        while keep_running:
            hit_list = []
            current_len_v = len(v)
            for j, vj in enumerate(v):
                if vj <= rho:
                    hit_list += [j]
                    current_len_v -= 1
                    rho += (rho - vj) / current_len_v
            v = __pop_all(v, hit_list)
            keep_running = len(v) != v_size_old
            v_size_old = len(v)

        # 6 ----
        ufunc = np.vectorize(lambda s: max(s - rho, 0), otypes=[np.float64])
        x = ufunc(np.array(y, dtype=np.float64))

        # result ----
        return x

    def is_convex(self):
        return True

    def is_compact(self):
        return True
