import casadi.casadi as cs
import opengen as og

do_build = True

u = cs.SX.sym("u", 5)                 # decision variable (nu = 5)
p = cs.SX.sym("p", 2)                 # parameter (np = 2)
phi = og.functions.rosenbrock(u, p)   # cost function

segment_ids = [1, 4]
cstrs = [og.constraints.Halfspace([1., 2.], 1.3326),
         og.constraints.Ball2([1., 2., 3.], 1.5)]
dim = 5
bounds = og.constraints.CartesianProduct(segment_ids, cstrs)

problem = og.builder.Problem(u, p, phi)  \
        .with_constraints(bounds)

meta = og.config.OptimizerMeta()                \
    .with_version("0.0.0")                      \
    .with_authors(["P. Sopasakis", "E. Fresk"]) \
    .with_licence("CC4.0-By")                   \
    .with_optimizer_name("potato")

build_config = og.config.BuildConfiguration()  \
    .with_build_directory("my_optimizers")     \
    .with_build_mode(og.config.BuildConfiguration.RELEASE_MODE)  \
    .with_open_version('0.7.1-alpha.1') \
    .with_tcp_interface_config()
solver_config = og.config.SolverConfiguration()    \
    .with_tolerance(1e-5)                          \
    .with_delta_tolerance(1e-4)                    \
    .with_initial_penalty(890)                     \
    .with_penalty_weight_update_factor(5)

builder = og.builder.OpEnOptimizerBuilder(problem,
                                          meta,
                                          build_config,
                                          solver_config).with_generate_not_build_flag(not do_build)

builder.build()

o = og.tcp.OptimizerTcpManager('my_optimizers/potato')
o.start()
r = o.call([1.0, 50.0])
if r.is_ok():
    status = r.get()
    print(status.solution)
o.kill()

