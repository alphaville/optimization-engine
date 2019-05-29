import subprocess
import shutil
import datetime

import opengen.config as og_cfg
import opengen.definitions as og_dfn
import casadi.casadi as cs
import os
import jinja2

_AUTOGEN_COST_FNAME = 'auto_casadi_cost.c'
_AUTOGEN_GRAD_FNAME = 'auto_casadi_grad.c'
_AUTOGEN_PNLT_CONSTRAINTS_FNAME = 'auto_casadi_constraints_type_penalty.c'
_ICASADI_CFG_HEADER_FNAME = 'icasadi_config.h'


class OpEnOptimizerBuilder:

    def __init__(self,
                 problem,
                 metadata=og_cfg.OptimizerMeta(),
                 build_configuration=og_cfg.BuildConfiguration(),
                 solver_configuration=og_cfg.SolverConfiguration()):
        self._problem = problem
        self._meta = metadata
        self._build_config = build_configuration
        self._solver_config = solver_configuration
        self._generate_not_build = False

    def with_problem(self, problem):
        """Specify problem"""
        self._problem = problem
        return self

    def with_generate_not_build_flag(self, flag):
        """Whether to build (or just generate code)

        If set to true, the code will be generated, but it will not be 
        build (mainly for debugging purposes)

        """
        self._generate_not_build = flag
        return self

    def _make_build_command(self):
        command = ['cargo', 'build']
        if self._build_config.build_mode().lower() == 'release':
            command.append('--release')
        return command

    def _target_dir(self):
        """target directory"""
        return os.path.abspath(
            os.path.join(
                self._build_config.build_dir(),
                self._meta.optimizer_name()))

    def _icasadi_target_dir(self):
        """icasadi target directory"""
        return os.path.abspath(
            os.path.join(
                self._build_config.build_dir(),
                self._meta.optimizer_name(), "icasadi"))

    def _prepare_target_project(self):
        """Creates folder structure

        Creates necessary folders (at build/{project_name})
        Runs `cargo init` in that folder

        """
        # Create target directory if it does not exist
        target_dir = self._target_dir()
        if self._build_config.rebuild():
            if os.path.exists(target_dir) and os.path.isdir(target_dir):
                shutil.rmtree(target_dir)
            os.makedirs(target_dir)
        else:
            if not os.path.exists(target_dir):
                os.makedirs(target_dir)

        # Run `cargo init` in target folder
        p = subprocess.Popen(['cargo', 'init'],
                             cwd=target_dir,
                             stderr=open(os.devnull, 'wb'),
                             stdout=open(os.devnull, 'wb'))
        p.wait()

    def _copy_icasadi_to_target(self):
        """Copy 'icasadi' into target directory"""
        origin_icasadi_dir = og_dfn.original_icasadi_dir()
        target_icasadi_dir = self._icasadi_target_dir()
        if not os.path.exists(target_icasadi_dir):
            os.makedirs(target_icasadi_dir)
        shutil.rmtree(target_icasadi_dir)
        shutil.copytree(origin_icasadi_dir,
                        target_icasadi_dir,
                        ignore=shutil.ignore_patterns(
                            '*.lock', 'ci*', 'target', 'auto*'))

    def _generate_icasadi_header(self):
        """Generates the header of icasadi, with constant definitions

        Generates icasadi_header.h
        """
        file_loader = jinja2.FileSystemLoader(og_dfn.templates_dir())
        env = jinja2.Environment(loader=file_loader)
        template = env.get_template('icasadi_config.h.template')
        output_template = template.render(problem=self._problem,
                                          build_config=self._build_config,
                                          timestamp_created=datetime.datetime.now())
        icasadi_config_h_path = os.path.abspath(
            os.path.join(
                self._icasadi_target_dir(),
                "extern", _ICASADI_CFG_HEADER_FNAME))
        with open(icasadi_config_h_path, "w") as fh:
            fh.write(output_template)

    def _generate_cargo_toml(self):
        """Generates Cargo.toml for generated project"""
        target_dir = self._target_dir()
        file_loader = jinja2.FileSystemLoader(og_dfn.templates_dir())
        env = jinja2.Environment(loader=file_loader)
        template = env.get_template('optimizer_cargo.toml.template')
        output_template = template.render(
            meta=self._meta,
            open_version=self._build_config.open_version())
        cargo_toml_path = os.path.abspath(os.path.join(target_dir, "Cargo.toml"))
        with open(cargo_toml_path, "w") as fh:
            fh.write(output_template)

    def _generate_casadi_code(self):
        """Generates CasADi code"""
        u = self._problem.decision_variables()
        p = self._problem.parameter_variables()
        ncp = self._problem.dim_constraints_penalty()
        phi = self._problem.cost_function()

        # If there are penalty-type constraints, we need to define a modified
        # cost function
        if ncp > 0:
            penalty_function = self._problem.penalty_function()
            mu = cs.SX.sym("mu", self._problem.dim_constraints_penalty())
            p = cs.vertcat(p, mu)
            phi += cs.dot(mu, penalty_function(self._problem.penalty_constraints()))

        # Define cost and its gradient as CasADi functions
        cost_fun = cs.Function(self._build_config.cost_function_name(), [u, p], [phi])
        grad_cost_fun = cs.Function(self._build_config.grad_function_name(),
                                 [u, p], [cs.jacobian(phi, u)])

        # Filenames of cost and gradient (C file names)
        cost_file_name = self._build_config.cost_function_name() + ".c"
        grad_file_name = self._build_config.grad_function_name() + ".c"

        # Code generation using CasADi (cost and gradient)
        cost_fun.generate(cost_file_name)
        grad_cost_fun.generate(grad_file_name)

        # Move generated files to target folder
        icasadi_extern_dir = os.path.join(self._icasadi_target_dir(), "extern")
        shutil.move(cost_file_name, os.path.join(icasadi_extern_dir, _AUTOGEN_COST_FNAME))
        shutil.move(grad_file_name, os.path.join(icasadi_extern_dir, _AUTOGEN_GRAD_FNAME))

        # Lastly, we generate code for the penalty constraints; if there aren't
        # any, we generate the function c(u; p) = 0 (which will not be used)
        if ncp > 0:
            penalty_constraints = self._problem.penalty_constraints()
        else:
            penalty_constraints = 0

        # Target C file name
        constraints_penalty_file_name = \
            self._build_config.constraint_penalty_function_name() + ".c"
        # Define CasADi function for c(u; q)
        constraint_penalty_fun = cs.Function(
            self._build_config.constraint_penalty_function_name(),
            [u, p], [penalty_constraints])
        # Generate code
        constraint_penalty_fun.generate(constraints_penalty_file_name)
        # Move auto-generated file to target folder
        shutil.move(constraints_penalty_file_name,
                    os.path.join(icasadi_extern_dir, _AUTOGEN_PNLT_CONSTRAINTS_FNAME))


    def _build_icasadi(self):
        icasadi_dir = self._icasadi_target_dir()
        command = self._make_build_command()
        p = subprocess.Popen(command, cwd=icasadi_dir)
        p.wait()

    def _generate_main_project_code(self):
        target_dir = self._target_dir()
        file_loader = jinja2.FileSystemLoader(og_dfn.templates_dir())
        env = jinja2.Environment(loader=file_loader)
        template = env.get_template('optimizer.rs.template')
        output_template = template.render(solver_config=self._solver_config,
                                          problem=self._problem,
                                          timestamp_created=datetime.datetime.now())
        target_scr_lib_rs_path = os.path.join(target_dir, "src", "lib.rs")
        with open(target_scr_lib_rs_path, "w") as fh:
            fh.write(output_template)

    def _build_optimizer(self):
        target_dir = self._target_dir()
        command = self._make_build_command()
        with open(os.devnull, 'w') as FNULL:
            p = subprocess.Popen(command, cwd=target_dir, stderr=FNULL, stdout=FNULL)
            process_completion = p.wait()
            if process_completion != 0:
                raise Exception('Rust build failed')

    def build(self):
        """
        Generate code and build project
        """
        self._prepare_target_project()      # create folders; init cargo project
        self._copy_icasadi_to_target()      # copy icasadi/ files to target dir
        self._generate_cargo_toml()         # generate Cargo.toml using tempalte
        self._generate_icasadi_header()     # generate icasadi_config.h
        self._generate_casadi_code()        # generate all necessary CasADi C files
        self._generate_main_project_code()  # generate main part of code (at build/{name}/src/main.rs)
        if not self._generate_not_build:
            self._build_optimizer()         # build overall project
