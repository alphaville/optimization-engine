# Some necessary imports
# ------------------------------------
import opengen as og

u = cs.SX.sym("u", 5)                 # decision variable (nu = 5)
p = cs.SX.sym("p", 2)                 # parameter (np = 2)
phi = og.functions.rosenbrock(u, p)   # cost function


f2 = cs.fmax(0.0, u[2] - u[3] + 0.1)

f1 = cs.vertcat(1.5 * u[0] - u[1], cs.sin(u[2] + cs.pi/5) - 0.2)
C = og.constraints.Ball2(None, 1.0)

UA = og.constraints.FiniteSet([[1, 2, 3], [1, 2, 2], [1, 2, 4], [0, 5, -1]])
UB = og.constraints.Ball2(None, 1.0)
U = og.constraints.CartesianProduct(5, [2, 4], [UA, UB])

problem = og.builder.Problem(u, p, phi)         \
    .with_constraints(U)                        \
    .with_aug_lagrangian_constraints(f1, C)

meta = og.config.OptimizerMeta()                \
    .with_version("0.0.0")                      \
    .with_authors(["P. Sopasakis", "E. Fresk"]) \
    .with_licence("CC4.0-By")                   \
    .with_optimizer_name("the_optimizer")

build_config = og.config.BuildConfiguration()  \
    .with_build_directory("my_optimizers")      \
    .with_build_mode("debug")                  \
    .with_tcp_interface_config()


pb = og.builder.ProblemBuilder()
pb.add_decision_variable(u, x)
pb.add_parameter_variable(p)

c = ogc.BallInf(radius=1.0)
d = ogc.BallInf(radius=1.0)
pb.add_constraint(u[[1, 2, 3]], c)
pb.add_constraint(u[[4, 7]], d)

print(pb.decision_variables)
