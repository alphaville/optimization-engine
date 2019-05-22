from jinja2 import Environment, FileSystemLoader
from opengen.config import *
from opengen.builder import *

# ---- Generate code using template (exercise)-------
file_loader = FileSystemLoader('../templates')
env = Environment(loader=file_loader)
template = env.get_template('xtmpl.rs')
output_template = template.render(tolerance=0.001)
with open("../templates/output.rs", "w") as fh:
    fh.write(output_template)
# ---------------------------------------------------


meta = OptimizerMeta()
build_config = BuildConfiguration()
solver_config = SolverConfiguration()

builder = OpEnOptimizerBuilder(meta, build_config, solver_config)

print(meta.build_name)
print(builder.solver_config.lbfgs_memory)


# ---Playing with CasADi in Python--------------------
from casadi import *
u = SX.sym("u", 5)
p = SX.sym("p", 2)
phi = u[0].sqrt()