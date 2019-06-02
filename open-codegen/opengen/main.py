import opengen as og
import casadi.casadi as cs


# ---------------------------------------------------------------------------
# Build parametric optimizer
# ---------------------------------------------------------------------------
u = cs.SX.sym("u", 5)
p = cs.SX.sym("p", 2)
phi = og.functions.rosenbrock(u, p)
bounds = og.constraints.Ball2(None, 1.5)
problem = og.builder.Problem(u, p, phi) \
    .with_penalty_constraints(None) \
    .with_constraints(bounds)
build_config = og.config.BuildConfiguration() \
    .with_build_directory(".python_test_build") \
    .with_build_mode("debug")
meta = og.config.OptimizerMeta() \
    .with_optimizer_name("tcp_enabled_optimizer")
builder = og.builder.OpEnOptimizerBuilder(problem,
                                          metadata=meta,
                                          build_configuration=build_config) \
    .with_verbosity_level(1)
builder.enable_tcp_interface()
builder.build()

# ---------------------------------------------------------------------------
# Use TCP server
# ---------------------------------------------------------------------------
mng = og.tcp.OptimizerTcpManager('.python_test_build/tcp_enabled_optimizer')
mng.start()

for i in range(5000):
    mng.ping()

solution = mng.call([1.0, 2.0 + i],
                        initial_guess=[0.8, 0.7, 0.6, 0.5, 0.3],
                        buffer_len=2048)
print(solution)

mng.kill()
