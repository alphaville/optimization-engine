import opengen as og
import casadi.casadi as cs

u = cs.SX.sym("u", 5)
p = cs.SX.sym("p", 2)
phi = og.functions.rosenbrock(u, p)
c = cs.vertcat(1.5 * u[0] - u[1],
               cs.fmax(0.0, u[2] - u[3] + 0.1))
bounds = og.constraints.Ball2(None, 1.5)
problem = og.builder.Problem(u, p, phi) \
    .with_penalty_constraints(c)        \
    .with_constraints(bounds)
meta = og.config.OptimizerMeta()               \
    .with_optimizer_name("potato")             \
    .with_version('0.1.2')                     \
    .with_licence('LGPLv3')
ros_config = og.config.RosConfiguration()      \
    .with_package_name("potato_optimizer")  \
    .with_node_name("potato_controller")    \
    .with_rate(35)                          \
    .with_description("cool ROS node")
local_open = '/home/chung/NetBeansProjects/RUST/optimization-engine/'
build_config = og.config.BuildConfiguration()  \
    .with_build_directory("my_optimizers")     \
    .with_build_mode(og.config.BuildConfiguration.RELEASE_MODE)  \
    .with_open_version('0.7.0-alpha.1') \
    .with_tcp_interface_config() \
    .with_ros(ros_config)
solver_config = og.config.SolverConfiguration()    \
    .with_tolerance(1e-5)                          \
    .with_delta_tolerance(1e-4)                    \
    .with_initial_penalty(890)                     \
    .with_penalty_weight_update_factor(5)

builder = og.builder.OpEnOptimizerBuilder(problem,
                                          meta,
                                          build_config,
                                          solver_config).with_verbosity_level(3)
builder.build()


o = og.tcp.OptimizerTcpManager('my_optimizers/potato')
o.start()
r = o.call([1.0, 50.0])
if r.is_ok():
    status = r.get()
    print(status.solution)
o.kill()
