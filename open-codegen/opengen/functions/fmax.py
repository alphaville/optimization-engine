import casadi.casadi as cs
import numpy as np
from .is_numeric import is_numeric
from .is_symbolic import is_symbolic


def fmax(u, v):
    if is_numeric(u) and is_numeric(v):
        return np.fmax(u, v)
    if is_symbolic(u) or is_symbolic(v):
        return cs.fmax(u, v)
    raise Exception("Illegal argument")
