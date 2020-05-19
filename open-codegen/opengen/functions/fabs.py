import casadi.casadi as cs
import numpy as np
from .is_numeric import is_numeric
from .is_symbolic import is_symbolic


def fabs(u):
    if is_numeric(u):
        return np.fabs(u)
    if is_symbolic(u):
        return cs.fabs(u)
    raise Exception("Illegal argument")
