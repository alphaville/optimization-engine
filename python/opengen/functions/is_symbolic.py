import casadi.casadi as cs


def is_symbolic(u):
    return isinstance(u, (cs.SX, cs.MX, cs.DM))
