import time
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
    .with_optimizer_name("halfspace_optimizer") \
    .with_authors(["P. Sopasakis", "S. Author"]).with_version("0.1.56")

tcp_config = og.config.TcpServerConfiguration(bind_port=3305)
build_config = og.config.BuildConfiguration() \
    .with_build_directory('my_optimizers') \
    .with_build_mode(og.config.BuildConfiguration.DEBUG_MODE) \
    .with_tcp_interface_config()

builder = og.builder.OpEnOptimizerBuilder(problem,
                                          meta,
                                          build_config,
                                          og.config.SolverConfiguration())
builder.build()

all_managers = []
for i in range(10):
    all_managers += [og.tcp.OptimizerTcpManager(
        optimizer_path='my_optimizers/halfspace_optimizer',
        ip='0.0.0.0',
        port=3311+i)]

for m in all_managers:
    m.start()

time.sleep(4)

for m in all_managers:
    print(m.details)
    resp = m.call(p=[1., 2.])
    print(resp.get().solution)

# mng.kill()
time.sleep(6)
for m in all_managers:
    m.kill()
