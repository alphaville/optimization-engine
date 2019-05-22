from opengen.config.meta import *
from opengen.config.build_config import *
from opengen.config.solver_config import *
from jinja2 import Environment, FileSystemLoader


class OpEnOptimizerBuilder:

    def __init__(self,
                 problem,
                 meta=OptimizerMeta(),
                 build_config=BuildConfiguration(),
                 solver_config=SolverConfiguration()):
        self.problem = problem
        self.meta = meta
        self.build_config = build_config
        self.solver_config = solver_config

    def with_problem(self, problem):
        self.problem = problem
        return self

    def _generate_icasadi_header(self):
        file_loader = FileSystemLoader('../templates')
        env = Environment(loader=file_loader)
        template = env.get_template('icasadi_config.h')
        output_template = template.render(nu=self.problem.dim_decision_variables(),
                                          np=self.problem.dim_parameters(),
                                          ncp=self.problem.dim_constraints_penalty())
        with open("icasadi_out.h", "w") as fh:
            fh.write(output_template)

