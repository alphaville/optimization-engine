import casadi.casadi as cs
import opengen as og
import os

TEST_DIR = ".python_test_build/benchmarkable"


def get_open_local_absolute_path():
    cwd = os.getcwd()
    return cwd.split('open-codegen')[0]


def solver_configuration(do_precond=False):
    solver_config = og.config.SolverConfiguration() \
        .with_tolerance(1e-6) \
        .with_initial_tolerance(1e-4) \
        .with_delta_tolerance(1e-5) \
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


benchmark1("benchmark1", False)
benchmark1("benchmark1p", True)
