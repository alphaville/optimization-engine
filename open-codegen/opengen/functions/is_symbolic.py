import casadi.casadi as cs


def is_symbolic(u):
    return isinstance(u, cs.SX) \
           or isinstance(u, cs.MX) \
           or isinstance(u, cs.DM)

