# Some necessary imports
# ------------------------------------
import opengen as og
import opengen.constraints as ogc
import casadi.casadi as cs

u = cs.SX.sym("u", 10)
x = cs.SX.sym("x", 4)
p = cs.SX.sym("p", 5)
f = cs.norm_2(u)


pb = og.builder.ProblemBuilder()
pb.add_decision_variable(u, x)
pb.add_parameter_variable(p)

c = ogc.BallInf(radius=1.0)
d = ogc.BallInf(radius=1.0)
pb.add_constraint(u[[1, 2, 3]], c)
pb.add_constraint(u[[4, 7]], d)

print(pb.decision_variables)

