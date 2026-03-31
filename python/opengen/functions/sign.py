import casadi.casadi as cs
import numpy as np
from .is_numeric import is_numeric
from .is_symbolic import is_symbolic


def sign(u):
    if is_numeric(u):
        return np.sign(u)
    if is_symbolic(u):
        return cs.sign(u)
    Exception("Illegal argument")
