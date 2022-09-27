import sys
import os

import casadi.casadi as cs
import opengen as og

optimizers_dir = "my_optimizers"
optimizer_name = "rosenbrock"

nu, np = 5, 2
u = cs.SX.sym("u", nu)  # decision variable (nu = 5)
p = cs.SX.sym("p", 2)  # parameter (np = 2)
phi = og.functions.rosenbrock(u, p) + 1500*cs.sum1(u)

# c_f1 = cs.vertcat(p[0] * u[0] - u[1], p[1]*u[2] - u[3] + 0.1)
c_f2 = cs.vertcat(0.2 + 1.5 * u[0] - u[1], u[2] - u[3] - 0.1)

bounds = og.constraints.Ball2(None, 1.5)
problem = og.builder.Problem(u, p, phi) \
    .with_penalty_constraints(c_f2)\
    .with_constraints(bounds)
meta = og.config.OptimizerMeta()\
    .with_optimizer_name(optimizer_name)

build_config = og.config.BuildConfiguration() \
    .with_build_directory(optimizers_dir) \
    .with_build_mode(og.config.BuildConfiguration.DEBUG_MODE) \
    .with_build_python_bindings() \
    .with_open_version(local_path="/Users/3054363/Documents/Development/OpEn/")
solver_cfg = og.config.SolverConfiguration() \
    .with_tolerance(1e-6) \
    .with_max_inner_iterations(1000) \
    .with_max_outer_iterations(30) \
    .with_penalty_weight_update_factor(1.5) \
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
print(f"cost = {result.cost}")

# Preconditioned    Non-preconditioned
# -------------------------------------
# 23/3              244/26

# Solutions:
#
# [-0.06168156776090604, 0.10745271293967644, 0.11363970229300129, 0.013666212246169969, 0.00018549750799884656]
# [-0.06168061635750967, 0.10744096043712821, 0.11361445307465148, 0.013640838407880301, 0.00019045750968868237]