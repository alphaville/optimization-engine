from .constraint import Constraint


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

        def __pop_all(z, indices):
            for index in sorted(indices, reverse=True):
                del z[index]
            return z

        # inputs ----
        len_y = len(y)
        a = self.__alpha
        # outputs ----
        x = [0] * len_y

        # 1 ----
        v = [y[0]]
        v_size_old = len(v)
        v_tilda = []
        rho = y[0] - a

        # 2 ----
        for n in range(1, len_y):
            if y[n] > rho:
                rho = rho + (y[n] - rho) / (len(v) + 1)
                if rho > y[n] - a:
                    v.append(y[n])
                else:
                    v_tilda.extend(v)
                    v = [y[n]]
                    rho = y[n] - a

        # 3 ----
        if len(v_tilda) > 0:
            for v_tilda_i in v_tilda:
                if v_tilda_i > rho:
                    v.append(v_tilda_i)
                    rho = rho + (v_tilda_i - rho) / len(v)

        # 4 ----
        keep_running = True
        while keep_running:
            hit_list = []
            current_len_v = len(v)
            for j in range(len(v)):
                if v[j] <= rho:
                    hit_list += [j]
                    current_len_v -= 1
                    rho = rho + (rho - v[j]) / current_len_v
            v = __pop_all(v, hit_list)
            keep_running = len(v) != v_size_old
            v_size_old = len(v)

        # 5 ----
        tau = rho
        K = len(v)

        # 6 ----
        for k in range(len_y):
            x[k] = max(y[k] - tau, 0)

        # result ----
        return x

    def is_convex(self):
        return True

    def is_compact(self):
        return True
