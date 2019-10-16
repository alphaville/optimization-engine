from .constraint import Constraint
import opengen.functions as fn


class Rectangle(Constraint):
    """A Rectangle (Box) constraint"""

    def __init__(self, xmin, xmax):
        """Construct a new instance of Rectangle

        Args:
            xmin: minimum bounds (can be None)
            xmax: maximum bounds (can be None)

        Raises:
            Exception: if both xmin and xmax is None
              Exception: if xmin/xmax is not None and not a list (wrong type)
            Exception: if xmin and xmax have incompatible lengths
            Exception: if xmin(i) > xmax(i) for some i (empty set)

        Returns:
             A new instance of Rectangle
        """
        # (None, None) is not allowed
        if xmin is None and xmax is None:
            raise Exception("At least one of xmin and xmax must be not None")

        # If one is not None, it must be a list
        if xmin is not None and not isinstance(xmin, list):
            raise Exception("xmin is neither None nor a list")

        if xmax is not None and not isinstance(xmax, list):
            raise Exception("xmax is neither None nor a list")

        # If BOTH are not None, they must be compatible
        if xmin is not None and xmax is not None:
            if len(xmin) != len(xmax):
                raise Exception("xmin and xmax must have equal lengths")
            for (xmin_element, xmax_element) in zip(xmin, xmax):
                if xmin_element > xmax_element:
                    raise Exception("xmin must be <= xmax")

        # Store xmin and xmax in attributes
        self.__xmin = None if xmin is None else [float(i) for i in xmin]
        self.__xmax = None if xmax is None else [float(i) for i in xmax]

    @property
    def xmin(self):
        """Minimum bound"""
        return self.__xmin

    @property
    def xmax(self):
        """Maximum bound"""
        return self.__xmax

    def dimension(self):
        if self.__xmin is not None:
            return len(self.__xmin)
        elif self.__xmax is not None:
            return len(self.__xmax)
        else:
            raise Exception("Absurd: both xmin and xmax are None!")

    def idx_bound_finite_all(self):
        idx_both_finite = []

        if self.__xmin is None or self.__xmax is None:
            return idx_both_finite

        for i in range(self.dimension()):
            xmini = self.__xmin[i]
            xmaxi = self.__xmax[i]
            if xmini > float('-inf') and xmaxi < float('inf'):
                idx_both_finite += [i]

        return idx_both_finite

    def idx_infinite_only_xmin(self):
        idx_xmin_infinite = []

        if self.__xmax is None:
            # If xmax is None (infinite), we should return
            # the empty set
            return idx_xmin_infinite

        # Hereafter, xmax is not None (but xmin can be None)

        for i in range(self.dimension()):
            xmini = self.__xmin[i] if self.__xmin is not None else float('-inf')
            xmaxi = self.__xmax[i]
            if xmini == float('-inf') and xmaxi < float('inf'):
                idx_xmin_infinite += [i]

        return idx_xmin_infinite

    def idx_infinite_only_xmax(self):
        idx_xmin_infinite = []

        if self.__xmin is None:
            # If xmin is None (-infinity), we should return
            # the empty set
            return idx_xmin_infinite

        # Hereafter, xmin is not None (xmax might be)
        for i in range(self.dimension()):
            xmini = self.__xmin[i]
            xmaxi = self.__xmax[i] if self.__xmax is not None else float('inf')
            if xmaxi == float('inf') and xmini > float('-inf'):
                idx_xmin_infinite += [i]

        return idx_xmin_infinite

    def distance_squared(self, u):
        idx1 = self.idx_infinite_only_xmin()
        idx2 = self.idx_infinite_only_xmax()
        idx3 = self.idx_bound_finite_all()

        dist_sq = 0.0
        for i in idx1:
            dist_sq += fn.fmax(0.0, u[i] - self.__xmax[i]) ** 2

        for i in idx2:
            dist_sq += fn.fmin(0.0, u[i] - self.__xmin[i]) ** 2

        for i in idx3:
            dist_sq += fn.fmin(
                fn.fmax(0.0, u[i] - self.__xmax[i]),
                u[i] - self.__xmin[i]) ** 2

        return dist_sq

    def project(self, u):
        raise NotImplementedError()

    def is_convex(self):
        return True

    def is_compact(self):
        if self.__xmin is None:
            return False

        if self.__xmax is None:
            return False

        for i in range(len(self.__xmin)):
            if self.__xmin[i] == float('-inf'):
                return False

        for i in range(len(self.__xmax)):
            if self.__xmax[i] == float('inf'):
                return False
