from opengen.config.meta import *
from opengen.config.build_config import *
from opengen.config.solver_config import *
from jinja2 import Environment, FileSystemLoader
import subprocess
import shutil
from casadi import *
import datetime


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

    '''
    Specify problem
    '''
    def with_problem(self, problem):
        self.problem = problem
        return self

    '''
    target directory
    '''
    def _target_dir(self):
        return os.path.abspath(default_build_dir() + "/" + self.meta.optimizer_name())

    '''
    icasadi target directory
    '''
    def _icasadi_target_dir(self):
        return os.path.abspath(default_build_dir() + "/" + self.meta.optimizer_name() + "/icasadi")

    '''
    Creates necessary folders (at build/{project_name})
    Runs `cargo init` in that folder
    '''
    def _prepare_target_project(self):
        # Create target directory if it does not exist
        target_dir = self._target_dir()
        if self.build_config.rebuild():
            if os.path.exists(target_dir) and os.path.isdir(target_dir):
                shutil.rmtree(target_dir)
            os.makedirs(target_dir)
        else:
            os.makedirs(target_dir, exist_ok=True)

        # Run `cargo init` in target folder
        p = subprocess.Popen(['cargo', 'init'],
                             cwd=target_dir,
                             stderr=open(os.devnull, 'wb'),
                             stdout=open(os.devnull, 'wb'))
        p.wait()

    '''
    Copy 'icasadi' into target directory
    '''
    def _copy_icasadi_to_target(self):
        target_dir = self._target_dir()
        origin_icasadi_dir = open_root_dir()+"/icasadi/"
        target_icasadi_dir = target_dir+"/icasadi"
        os.makedirs(target_icasadi_dir, exist_ok=True)
        shutil.rmtree(target_icasadi_dir)
        shutil.copytree(origin_icasadi_dir,
                        target_icasadi_dir,
                        ignore=shutil.ignore_patterns('*.lock', 'ci*', 'target', 'auto*'))

    '''
    Generates the header of icasadi, with all constant definition (icasadi_header.h)
    '''
    def _generate_icasadi_header(self):
        file_loader = FileSystemLoader('../templates')
        env = Environment(loader=file_loader)
        template = env.get_template('icasadi_config.h.template')
        output_template = template.render(problem=self.problem,
                                          build_config=self.build_config,
                                          timestamp_created=datetime.datetime.utcnow())
        with open(self._icasadi_target_dir()+"/extern/icasadi_config.h", "w") as fh:
            fh.write(output_template)

    '''
    Generates Cargo.toml for generated project
    '''
    def _generate_cargo_toml(self):
        target_dir = self._target_dir()
        file_loader = FileSystemLoader('../templates')
        env = Environment(loader=file_loader)
        template = env.get_template('optimizer_cargo.toml.template')
        output_template = template.render(meta=self.meta)
        with open(target_dir + "/Cargo.toml", "w") as fh:
            fh.write(output_template)

    '''
    Generate CasADi code
    '''
    def _generate_casadi_code(self):
        u = self.problem.decision_variables()
        p = self.problem.parameter_variables()
        ncp = self.problem.dim_constraints_penalty()
        phi = self.problem.cost_function()
        penalty_function = self.problem.penalty_function()

        if ncp > 0:
            mu = SX.sym("mu", self.problem.dim_constraints_penalty())
            p = vertcat(p, mu)
            phi += dot(mu, penalty_function(self.problem.penalty_constraints()))

        cost_fun = Function(self.build_config.cost_function_name(), [u, p], [phi])

        grad_cost_fun = Function(self.build_config.grad_function_name(),
                                 [u, p], [jacobian(phi, u)])

        constraint_penalty_fun = Function(self.build_config.constraint_penalty_function_name(),
                                          [u, p], [self.problem.penalty_constraints()])

        cost_file_name = self.build_config.cost_function_name() + ".c"
        grad_file_name = self.build_config.grad_function_name() + ".c"
        constraints_penalty_file_name = self.build_config.constraint_penalty_function_name() + ".c"

        # code generation using CasADi
        cost_fun.generate(cost_file_name)
        grad_cost_fun.generate(grad_file_name)
        constraint_penalty_fun.generate(constraints_penalty_file_name)

        # Move generated files to target folder
        shutil.move(cost_file_name, self._icasadi_target_dir() + "/extern/auto_casadi_cost.c" )
        shutil.move(grad_file_name, self._icasadi_target_dir() + "/extern/auto_casadi_grad.c")
        shutil.move(constraints_penalty_file_name, self._icasadi_target_dir() + "/extern/auto_casadi_constraints_type_penalty.c")

    def _build_icasadi(self):
        icasadi_dir = self._icasadi_target_dir()
        command = ['cargo', 'build']
        if self.build_config.build_mode().lower() == 'release':
            command.append('--release')
        p = subprocess.Popen(command, cwd=icasadi_dir)
        p.wait()

    def _generate_main_project_code(self):
        target_dir = self._target_dir()
        file_loader = FileSystemLoader('../templates')
        env = Environment(loader=file_loader)
        template = env.get_template('optimizer.rs.template')
        output_template = template.render(solver_config=self.solver_config,
                                          problem=self.problem)
        with open(target_dir + "/src/lib.rs", "w") as fh:
            fh.write(output_template)


    '''
    Generate code and build project
    '''
    def build(self):
        self._prepare_target_project()       # create folders; init cargo project
        self._copy_icasadi_to_target()       # copy icasadi/ files to target dir
        self._generate_cargo_toml()          # generate Cargo.toml using tempalte
        self._generate_icasadi_header()      # generate icasadi_config.h
        self._generate_casadi_code()         # generate all necessary CasADi C files
        self._build_icasadi()                # build icasadi
        self._generate_main_project_code()   # generate main part of code (at build/{name}/src/main.rs)
