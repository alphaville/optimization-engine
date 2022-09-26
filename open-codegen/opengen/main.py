import sys
import os

import casadi.casadi as cs
import opengen as og

optimizers_dir = "my_optimizers"
optimizer_name = "rosenbrock"

nu, np = 5, 2
u = cs.SX.sym("u", nu)  # decision variable (nu = 5)
p = cs.SX.sym("p", 2)  # parameter (np = 2)
phi = og.functions.rosenbrock(u, p)
for i in range(nu):
    phi = phi + p[1] * cs.sin(u[i])

c_f1 = cs.vertcat(p[0] * u[0] - u[1], p[1]*u[2] - u[3] + 0.1)

bounds = og.constraints.Halfspace([1, 1, 1, 1, 1], 100)
problem = og.builder.Problem(u, p, phi) \
    .with_aug_lagrangian_constraints(c_f1, og.constraints.Zero())\
    .with_constraints(bounds)
meta = og.config.OptimizerMeta() \
    .with_optimizer_name(optimizer_name)

build_config = og.config.BuildConfiguration() \
    .with_build_directory(optimizers_dir) \
    .with_build_mode(og.config.BuildConfiguration.DEBUG_MODE) \
    .with_build_python_bindings()
solver_cfg = og.config.SolverConfiguration() \
    .with_tolerance(1e-6) \
    .with_max_inner_iterations(1000) \
    .with_max_outer_iterations(20) \
    .with_penalty_weight_update_factor(1.3) \
    .with_preconditioning(True)

builder = og.builder.OpEnOptimizerBuilder(problem,
                                          meta,
                                          build_config,
                                          solver_cfg)
builder.build()

sys.path.insert(1, os.path.join(optimizers_dir, optimizer_name))
rosenbrock = __import__(optimizer_name)

solver = rosenbrock.solver()
result = solver.run(p=[0.5, 8.5], initial_guess=[1, 2, 3, 4, 0])
print(" ")
print(f"solution = {result.solution}")
print(f"time = {result.solve_time_ms} ms")
print(f"penalty = {result.penalty}")
print(f"infeasibility f1 = {result.f1_infeasibility}")
print(f"infeasibility f2 = {result.f2_norm}")
print(f"status = {result.exit_status}")
print(f"inner = {result.num_inner_iterations}")
print(f"outer = {result.num_outer_iterations}")

# Precond    NonPrecond
# 125/16     273/14
