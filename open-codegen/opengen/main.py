import logging
import casadi.casadi as cs
import opengen as og

logging.getLogger().setLevel(5)

u = cs.SX.sym("u", 5)  # decision variable (nu = 5)
p = cs.SX.sym("p", 2)  # parameter (np = 2)
phi = cs.dot(u, u)  # cost function

bounds = og.constraints.Halfspace([1., 2., 1., 5., 2.], -10.39)

problem = og.builder.Problem(u, p, phi) \
    .with_constraints(bounds)

meta = og.config.OptimizerMeta() \
    .with_optimizer_name("halfspace_optimizer")

tcp_config = og.config.TcpServerConfiguration(bind_port=3305)
build_config = og.config.BuildConfiguration() \
    .with_build_directory('my_optimizers') \
    .with_build_mode(og.config.BuildConfiguration.DEBUG_MODE) \
    .with_open_version('0.7.1-alpha') \
    .with_tcp_interface_config()

builder = og.builder.OpEnOptimizerBuilder(problem,
                                          meta,
                                          build_config,
                                          og.config.SolverConfiguration())
# builder.build()

mng = og.tcp.OptimizerTcpManager('my_optimizers/halfspace_optimizer', ip='0.0.0.0', port=3311)
mng.start()

print(mng.ping())
resp = mng.call(p=[1., 2.])
print(resp.get().solution)
mng.kill()
