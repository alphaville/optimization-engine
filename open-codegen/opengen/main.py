import opengen as og
import casadi.casadi as cs
import numba

x = cs.SX.sym('x', 1)
y = cs.sin(cs.cos(x))

f = cs.Function('f', [x], [y])


@numba.jit()
def myfun(s):
    return f(s)


print(myfun(3))