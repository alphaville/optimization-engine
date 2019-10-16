import casadi.casadi as cs
import numpy as np
from .is_numeric import *
from .is_symbolic import *


def sign(u):
    if is_numeric(u):
        return np.sign(u)
    elif is_symbolic(u):
        return cs.sign(u)
    else:
        raise Exception("Illegal argument")