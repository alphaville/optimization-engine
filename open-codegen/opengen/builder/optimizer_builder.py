from opengen.config.meta import *
from opengen.config.build_config import *
from opengen.config.solver_config import *
from jinja2 import Environment, FileSystemLoader
import subprocess


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


    def _prepare_target_project(self):
        # Create target directory if it does not exist
        target_dir = default_build_dir() + "/" + self.meta.build_name
        os.makedirs(target_dir, exist_ok=True)

        # Run `cargo init` in target folder
        p = subprocess.Popen(['cargo', 'init'], cwd=target_dir)
        p.wait()


    '''
    Generates the header of icasadi, with all constant definition (icasadi_header.h)
    '''
    def _generate_icasadi_header(self):
        file_loader = FileSystemLoader('../templates')
        env = Environment(loader=file_loader)
        template = env.get_template('icasadi_config.h.template')
        output_template = template.render(nu=self.problem.dim_decision_variables(),
                                          np=self.problem.dim_parameters(),
                                          ncp=self.problem.dim_constraints_penalty())
        with open("icasadi_out.h", "w") as fh:
            fh.write(output_template)


    '''
    Generates Cargo.toml for generated project
    '''
    def _generate_cargo_toml(self):
        target_dir = default_build_dir() + "/" + self.meta.build_name
        file_loader = FileSystemLoader('../templates')
        env = Environment(loader=file_loader)
        template = env.get_template('optimizer_cargo.toml.template')
        output_template = template.render(meta=self.meta)
        with open(target_dir + "/Cargo.toml", "w") as fh:
            fh.write(output_template)
        0

