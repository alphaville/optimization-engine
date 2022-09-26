import sys
import os

import casadi.casadi as cs
import opengen as og

optimizers_dir = "my_optimizers"
optimizer_name = "rosenbrock"

nu, np = 5, 2
u = cs.SX.sym("u", nu)  # decision variable (nu = 5)
p = cs.SX.sym("p", 2)  # parameter (np = 2)
phi = 0.5 * p[0] * cs.dot(u, u)
for i in range(nu):
    phi = phi + p[1] * cs.sin(u[i])


c_f1 = cs.vertcat(p[0] * u[0] - u[1], p[1]*u[2] - u[3] + 0.1)
c_f2 = cs.vertcat((u[2]+1)**2 - 8*u[4], 2*cs.sin(u[4] + 1), 5*cs.sin(u[3] + 1))

bounds = og.constraints.Halfspace([1., 2.5, 1., 5., 2.], 10.39)
problem = og.builder.Problem(u, p, phi) \
    .with_penalty_constraints(c_f2)\
    .with_aug_lagrangian_constraints(c_f1, og.constraints.Zero())\
    .with_constraints(bounds)
meta = og.config.OptimizerMeta() \
    .with_optimizer_name(optimizer_name)

tcp_config = og.config.TcpServerConfiguration(bind_port=3305)
build_config = og.config.BuildConfiguration() \
    .with_build_directory(optimizers_dir) \
    .with_build_mode(og.config.BuildConfiguration.DEBUG_MODE) \
    .with_build_python_bindings() \
    .with_tcp_interface_config()
solver_cfg = og.config.SolverConfiguration() \
    .with_tolerance(1e-6) \
    .with_preconditioning(True)

builder = og.builder.OpEnOptimizerBuilder(problem,
                                          meta,
                                          build_config,
                                          solver_cfg)
builder.build()

sys.path.insert(1, os.path.join(optimizers_dir, optimizer_name))
rosenbrock = __import__(optimizer_name)

solver = rosenbrock.solver()
result = solver.run(p=[3.5, 7.8], initial_guess=[1, 2, 3, 4, 5])
print(" ")
print(f"solution = {result.solution}")
print(f"time = {result.solve_time_ms} ms")
print(f"penalty = {result.penalty}")
print(f"status = {result.exit_status}")
