import casadi.casadi as cs
import numpy as np
from .is_numeric import *
from .is_symbolic import *


def fmax(u, v):
    if is_numeric(u) and is_numeric(v):
        return np.fmax(u, v)
    elif is_symbolic(u) or is_symbolic(v):
        return cs.fmax(u, v)
    else:
        raise Exception("Illegal argument")