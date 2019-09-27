import casadi.casadi as cs
import numpy as np
from .is_numeric import *
from .is_symbolic import *


def fmin(u, v):
    if is_numeric(u) and is_numeric(v):
        return np.fmin(u, v)
    elif is_symbolic(u) or is_symbolic(v):
        return cs.fmin(u, v)
    else:
        raise Exception("Illegal argument")