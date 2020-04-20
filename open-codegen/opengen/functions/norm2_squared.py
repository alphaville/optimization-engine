import casadi.casadi as cs
import numpy as np
from .is_numeric import is_numeric
from .is_symbolic import is_symbolic


def norm2_squared(u):
    if (isinstance(u, list) and all(is_numeric(x) for x in u))\
                or isinstance(u, np.ndarray):
        # if `u` is a numeric vector
        return np.dot(u, u)
    if is_symbolic(u):
        return cs.dot(u, u)
    raise Exception("Illegal argument")
