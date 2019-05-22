from jinja2 import Environment, FileSystemLoader
from opengen.builder import *
from casadi import *
from opengen.definitions import *


# ---Playing with CasADi in Python--------------------
def rosenbrock(u_, p_):
    if not isinstance(p_, SX) or p_.size()[0] != 2:
        raise Exception('illegal parameter p_ (must be SX of size (2,1))')
    if not isinstance(u_, SX):
        raise Exception('illegal parameter u_ (must be SX)')
    nu = u_.size()[0]
    a = p_[0]
    b = p_[1]
    ros_fun = 0
    for i in range(nu-1):
        ros_fun += b*(u_[i+1]-u_[i]**2)**2 + (a-u_[i])**2
    return ros_fun


u = SX.sym("u", 5)
p = SX.sym("p", 2)
phi = rosenbrock(u,p)
c = vertcat(norm_2(u)-1., u[0]+p[0]*u[1]-3.);
phi_fun = Function('phi', [u, p], [phi])


problem = Problem(u, p, phi)
problem.with_penalty_constraints(c)

meta = OptimizerMeta()
build_config = BuildConfiguration()
solver_config = SolverConfiguration()

builder = OpEnOptimizerBuilder(problem, meta, build_config, solver_config)

builder.build()
builder._generate_casadi_code()

print(meta.authors)
print("root = " + open_codegen_root_dir())
print("build dir = " + default_build_dir())
