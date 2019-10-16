import casadi.casadi as cs
import numpy as np
from .is_numeric import *
from .is_symbolic import *


def norm2_squared(u):
    if (isinstance(u, list) and all(is_numeric(x) for x in u))\
                or isinstance(u, np.ndarray):
        # if `u` is a numeric vector
        return np.dot(u, u)
    elif is_symbolic(u):
        return cs.dot(u, u)
    else:
        raise Exception("Illegal argument")