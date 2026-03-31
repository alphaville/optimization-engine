import numpy as np


def is_numeric(u):
    return isinstance(u, (int, float)) \
           or np.isscalar(u)
