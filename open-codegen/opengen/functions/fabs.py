import casadi.casadi as cs
import numpy as np
from .is_numeric import *
from .is_symbolic import *


def fabs(u):
    if is_numeric(u):
        return np.fabs(u)
    elif is_symbolic(u):
        return cs.fabs(u)
    else:
        raise Exception("Illegal argument")