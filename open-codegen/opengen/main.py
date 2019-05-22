from jinja2 import Environment, FileSystemLoader
from opengen.builder import *
from casadi import *
from opengen.definitions import *
from opengen.functions.rosenbrock import *


u = SX.sym("u", 5)
p = SX.sym("p", 2)
phi = rosenbrock(u,p)                          # cost function
c = vertcat(norm_2(u)-1.,
            u[0]+p[0]*u[1]-3.,
            (p[0]+p[1])*(u[0]+u[1]) - 2.)      # c(u; p)

problem = Problem(u, p, phi)\
    .with_penalty_constraints(c)               # Problem statement


meta = OptimizerMeta().with_version("0.0.2").with_authors(["P. Sopasakis", "E. Fresk"]).with_licence("CC4.0-By")

build_config = BuildConfiguration()
build_config.with_rebuild(True)

builder = OpEnOptimizerBuilder(problem,
                               meta=meta,
                               build_config=build_config)

builder.build()

