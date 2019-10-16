import casadi.casadi as cs
import numpy as np
from .is_numeric import *
from .is_symbolic import *


def norm2(u):
    if (isinstance(u, list) and all(is_numeric(x) for x in u))\
                or isinstance(u, np.ndarray):
        # if `u` is a numeric vector
        return np.linalg.norm(u)
    elif is_symbolic(u):
        return cs.norm_2(u)
    else:
        raise Exception("Illegal argument")