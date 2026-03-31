import casadi.casadi as cs
import numpy as np
from .is_numeric import is_numeric
from .is_symbolic import is_symbolic


def norm2(u):
    if (isinstance(u, list) and all(is_numeric(x) for x in u))\
                or isinstance(u, np.ndarray):
        # if `u` is a numeric vector
        return np.linalg.norm(u)
    if is_symbolic(u):
        return cs.norm_2(u)
    raise Exception("Illegal argument")
