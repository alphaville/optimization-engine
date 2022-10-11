import casadi.casadi as cs
import opengen as og
import os
import numpy as np

TEST_DIR = ".python_test_build/benchmarkable"


def get_open_local_absolute_path():
    cwd = os.getcwd()
    return cwd.split('open-codegen')[0]


def solver_configuration(do_precond=False):
    solver_config = og.config.SolverConfiguration() \
        .with_tolerance(1e-6) \
        .with_initial_tolerance(1e-4) \
        .with_delta_tolerance(1e-5) \
        .with_max_inner_iterations(1000) \
        .with_preconditioning(do_precond)
    return solver_config


def build_configuration():
    return og.config.BuildConfiguration() \
        .with_open_version(local_path=get_open_local_absolute_path()) \
        .with_build_mode(og.config.BuildConfiguration.RELEASE_MODE) \
        .with_build_directory(TEST_DIR) \
        .with_build_python_bindings()


def benchmark1(optimizer_name, do_precond=False):
    u = cs.MX.sym("u", 5)
    p = cs.MX.sym("p", 3)
    f2 = u[0] - p[2]
    phi = og.functions.rosenbrock(u, cs.vertcat(p[0], p[1]))  # cost function
    bounds = og.constraints.Ball2(None, 1.5)
    meta = og.config.OptimizerMeta() \
        .with_optimizer_name(optimizer_name)
    problem = og.builder.Problem(u, p, phi) \
        .with_penalty_constraints(f2) \
        .with_constraints(bounds)
    build_config = build_configuration()
    solver_config = solver_configuration(do_precond)
    og.builder.OpEnOptimizerBuilder(problem, meta, build_config, solver_config).build()


def navigation_problem(with_obstacles=False):
    (nu, nx, N, L, ts) = (2, 3, 60, 0.5, 0.1)
    (xref, yref, thetaref) = (2, 2, 0.2)
    (x_obs, y_obs, r_obs) = (0, 0, 1)
    (q, qtheta, r, qN, qthetaN) = (10, 0.1, 1, 200, 2)

    u = cs.SX.sym('u', nu * N)
    z0 = cs.SX.sym('z0', nx)

    (x, y, theta) = (z0[0], z0[1], z0[2])
    cost = 0
    obstacle_avoidance = []
    for t in range(0, nu * N, nu):
        cost += q * ((x - xref) ** 2 + (y - yref) ** 2) + qtheta * (theta - thetaref) ** 2
        u_t = u[t:t + 2]
        theta_dot = (u_t[1] * cs.cos(theta) - u_t[0] * cs.sin(theta)) / L
        cost += r * cs.dot(u_t, u_t)
        x += ts * (u_t[0] + L * cs.sin(theta) * theta_dot)
        y += ts * (u_t[1] - L * cs.cos(theta) * theta_dot)
        theta += ts * theta_dot
        obs = 1000 * cs.fmax(0, r_obs**2 - (x-x_obs) ** 2 - (y-y_obs)**2)
        obstacle_avoidance = cs.vertcat(obstacle_avoidance, obs)

    cost += qN * ((x - xref) ** 2 + (y - yref) ** 2) + qthetaN * (theta - thetaref) ** 2

    umin = [-3.0] * (nu * N)
    umax = [3.0] * (nu * N)
    bounds = og.constraints.Rectangle(umin, umax)

    problem = og.builder.Problem(u, z0, cost).with_constraints(bounds)
    if with_obstacles:
        problem = problem.with_penalty_constraints(obstacle_avoidance)
    return problem


def benchmark2(optimizer_name, with_obstacles=False, do_precond=False):
    """Navigation"""
    problem = navigation_problem(with_obstacles)
    build_config = build_configuration()
    meta = og.config.OptimizerMeta() \
        .with_optimizer_name(optimizer_name)
    solver_config = og.config.SolverConfiguration() \
        .with_tolerance(1e-5) \
        .with_penalty_weight_update_factor(2) \
        .with_max_inner_iterations(1000) \
        .with_preconditioning(do_precond)
    builder = og.builder.OpEnOptimizerBuilder(problem,
                                              meta,
                                              build_config,
                                              solver_config)
    builder.build()


benchmark1("benchmark1", do_precond=False)
benchmark1("benchmark1p", do_precond=True)
benchmark2("benchmark2", do_precond=False)
benchmark2("benchmark2p", do_precond=True)
benchmark2("benchmark2o", with_obstacles=True, do_precond=False)
benchmark2("benchmark2op", with_obstacles=True, do_precond=True)