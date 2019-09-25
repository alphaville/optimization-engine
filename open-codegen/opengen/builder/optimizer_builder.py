import subprocess
import shutil
import datetime
import yaml
import warnings

import opengen.config as og_cfg
import opengen.definitions as og_dfn
import casadi.casadi as cs
import os
import jinja2
import logging

_AUTOGEN_COST_FNAME = 'auto_casadi_cost.c'
_AUTOGEN_GRAD_FNAME = 'auto_casadi_grad.c'
_AUTOGEN_PNLT_CONSTRAINTS_FNAME = 'auto_casadi_mapping_f2.c'
_AUTOGEN_ALM_MAPPING_F1_FNAME = 'auto_casadi_mapping_f1.c'


def make_dir_if_not_exists(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)


class OpEnOptimizerBuilder:
    """Builder for code generation

    This class is a builder that can be used to generate
    code for a parametric optimizer.
    """

    def __init__(self,
                 problem,
                 metadata=og_cfg.OptimizerMeta(),
                 build_configuration=og_cfg.BuildConfiguration(),
                 solver_configuration=og_cfg.SolverConfiguration()):
        """Constructor of OpEnOptimizerBuilder

        Args:
            problem: instance of \link opengen.builder.problem.Problem Problem \endlink
            metadata: instance of \link opengen.config.meta.OptimizerMeta OptimizerMeta \endlink
            build_configuration: instance of \link opengen.config.build_config.BuildConfiguration BuildConfiguration \endlink
            solver_configuration: instance of \link  opengen.config.solver_config.SolverConfiguration SolverConfiguration \endlink

        Returns:
            New instance of OpEnOptimizerBuilder.
        """
        self.__problem = problem
        self.__meta = metadata
        self.__build_config = build_configuration
        self.__solver_config = solver_configuration
        self.__generate_not_build = False
        self.__verbosity_level = 0

    def with_verbosity_level(self, verbosity_level):
        """Specify the verbosity level

        Args:
            verbosity_level: level of verbosity (0,1,2,3)

        Returns:
            Current builder object
        """
        self.__verbosity_level = verbosity_level
        return self

    def with_problem(self, problem):
        """Specify problem

        Args:
            problem: optimization problem data

        Returns:
            Current builder object
        """
        self.__problem = problem
        return self

    def with_generate_not_build_flag(self, flag):
        """Whether to build (or just generate code)

        If set to true, the code will be generated, but it will not be
        build (mainly for debugging purposes)

        Args:
            flag: generate and not build

        Returns:
            Current builder object
        """
        self.__generate_not_build = flag
        return self

    def __make_build_command(self):
        command = ['cargo', 'build']
        if self.__build_config.build_mode.lower() == 'release':
            command.append('--release')
        return command

    def __target_dir(self):
        """target directory"""
        return os.path.abspath(
            os.path.join(
                self.__build_config.build_dir,
                self.__meta.optimizer_name))

    def __icasadi_target_dir(self):
        """icasadi target directory"""
        return os.path.abspath(
            os.path.join(
                self.__build_config.build_dir,
                self.__meta.optimizer_name, "icasadi"))

    def __prepare_target_project(self):
        """Creates folder structure

        Creates necessary folders (at build/{project_name})
        Runs `cargo init` in that folder

        """
        logging.info("Creating necessary folders")

        # Create target directory if it does not exist
        target_dir = self.__target_dir()
        if self.__build_config.rebuild:
            if os.path.exists(target_dir) and os.path.isdir(target_dir):
                shutil.rmtree(target_dir)
            os.makedirs(target_dir)
        else:
            make_dir_if_not_exists(target_dir)

    def __copy_icasadi_to_target(self):
        """Copy 'icasadi' into target directory"""
        logging.info("Copying icasadi interface to target directory")
        origin_icasadi_dir = og_dfn.original_icasadi_dir()
        target_icasadi_dir = self.__icasadi_target_dir()
        if not os.path.exists(target_icasadi_dir):
            os.makedirs(target_icasadi_dir)
        shutil.rmtree(target_icasadi_dir)
        shutil.copytree(origin_icasadi_dir,
                        target_icasadi_dir,
                        ignore=shutil.ignore_patterns(
                            '*.lock', 'ci*', 'target', 'auto*'))

    def __generate_icasadi_c_interface(self):
        logging.info("Generating intercafe.c (C interface)")
        file_loader = jinja2.FileSystemLoader(og_dfn.templates_dir())
        env = jinja2.Environment(loader=file_loader)
        template = env.get_template('interface.c.template')
        output_template = template.render(meta=self.__meta,
                                          problem=self.__problem,
                                          build_config=self.__build_config,
                                          timestamp_created=datetime.datetime.now())
        icallocator_path = os.path.abspath(
            os.path.join(self.__icasadi_target_dir(), "extern/interface.c"))
        with open(icallocator_path, "w") as fh:
            fh.write(output_template)

    def __generate_icasadi_lib(self):
        """Generates the Rust library file of icasadi

        Generates src/lib.rs
        """
        logging.info("Generating icasadi Rust library file")
        file_loader = jinja2.FileSystemLoader(og_dfn.templates_dir())
        env = jinja2.Environment(loader=file_loader)
        template = env.get_template('icasadi_lib.rs.template')
        output_template = template.render(meta=self.__meta,
                                          problem=self.__problem,
                                          build_config=self.__build_config,
                                          timestamp_created=datetime.datetime.now())
        icasadi_lib_rs_path = os.path.abspath(
            os.path.join(self.__icasadi_target_dir(), "src/lib.rs"))
        with open(icasadi_lib_rs_path, "w") as fh:
            fh.write(output_template)

    def __generate_cargo_toml(self):
        """Generates Cargo.toml for generated project"""
        logging.info("Generating Cargo.toml for target optimizer")
        target_dir = self.__target_dir()
        file_loader = jinja2.FileSystemLoader(og_dfn.templates_dir())
        env = jinja2.Environment(loader=file_loader)
        template = env.get_template('optimizer_cargo.toml.template')
        output_template = template.render(
            meta=self.__meta,
            open_version=self.__build_config.open_version,
            activate_tcp_server=self.__build_config.tcp_interface_config is not None,
            activate_clib_generation=self.__build_config.build_c_bindings)
        cargo_toml_path = os.path.abspath(os.path.join(target_dir, "Cargo.toml"))
        with open(cargo_toml_path, "w") as fh:
            fh.write(output_template)

    def __generate_memory_code(self, cost=None, grad=None, f1=None, f2=None):
        logging.info("Generating memory")
        file_loader = jinja2.FileSystemLoader(og_dfn.templates_dir())
        env = jinja2.Environment(loader=file_loader)
        template = env.get_template('casadi_memory.h.template')
        output_template = template.render(cost=cost, grad=grad, f1=f1, f2=f2)
        memory_path = os.path.abspath(
            os.path.join(self.__icasadi_target_dir(), "extern/casadi_memory.h"))
        with open(memory_path, "w") as fh:
            fh.write(output_template)

    def __generate_casadi_code(self):
        """Generates CasADi code"""
        logging.info("Defining CasADi functions and generating C code")
        u = self.__problem.decision_variables
        p = self.__problem.parameter_variables
        ncp = self.__problem.dim_constraints_penalty()
        nalm = self.__problem.dim_constraints_aug_lagrangian()
        print("number of ALM constraints:", nalm)
        phi = self.__problem.cost_function
        print("number of PM constraints:", ncp)

        # If there are penalty-type constraints, we need to define a modified
        # cost function
        if ncp > 0:
            penalty_function = self.__problem.penalty_function
            mu = cs.SX.sym("mu", self.__problem.dim_constraints_penalty()) \
                if isinstance(p, cs.SX) else cs.MX.sym("mu", self.__problem.dim_constraints_penalty())
            p = cs.vertcat(p, mu)
            phi += cs.dot(mu, penalty_function(self.__problem.penalty_mapping_f2))

        # Define cost and its gradient as CasADi functions
        cost_fun = cs.Function(self.__build_config.cost_function_name, [u, p], [phi])
        grad_cost_fun = cs.Function(self.__build_config.grad_function_name,
                                 [u, p], [cs.jacobian(phi, u)])

        # Filenames of cost and gradient (C file names)
        cost_file_name = self.__build_config.cost_function_name + ".c"
        grad_file_name = self.__build_config.grad_function_name + ".c"

        # Code generation using CasADi (cost and gradient)
        cost_fun.generate(cost_file_name)
        grad_cost_fun.generate(grad_file_name)

        # Move generated files to target folder
        icasadi_extern_dir = os.path.join(self.__icasadi_target_dir(), "extern")
        shutil.move(cost_file_name, os.path.join(icasadi_extern_dir, _AUTOGEN_COST_FNAME))
        shutil.move(grad_file_name, os.path.join(icasadi_extern_dir, _AUTOGEN_GRAD_FNAME))

        # Next, we generate a CasADi function for the augmented Lagrangian constraints,
        # that is, mapping F1(u, p)
        if nalm > 0:
            mapping_f1 = self.__problem.penalty_mapping_f1
        else:
            mapping_f1 = 0

        # Target C file name of mapping F1(u, p)
        alm_mapping_f1_file_name = \
            self.__build_config.alm_mapping_f1_function_name + ".c"
        # Define CasADi function F1(u, p)
        alm_mapping_f1_fun = cs.Function(
            self.__build_config.alm_mapping_f1_function_name,
            [u, p], [mapping_f1])
        # Generate code for F1(u, p)
        alm_mapping_f1_fun.generate(alm_mapping_f1_file_name)
        # Move auto-generated file to target folder
        shutil.move(alm_mapping_f1_file_name,
                    os.path.join(icasadi_extern_dir, _AUTOGEN_ALM_MAPPING_F1_FNAME))

        # Lastly, we generate code for the penalty constraints; if there aren't
        # any, we generate the function c(u; p) = 0 (which will not be used)
        if ncp > 0:
            penalty_constraints = self.__problem.penalty_mapping_f2
        else:
            penalty_constraints = 0

        # Target C file name
        constraints_penalty_file_name = \
            self.__build_config.constraint_penalty_function_name + ".c"
        # Define CasADi function for c(u; q)
        constraint_penalty_fun = cs.Function(
            self.__build_config.constraint_penalty_function_name,
            [u, p], [penalty_constraints])
        # Generate code
        constraint_penalty_fun.generate(constraints_penalty_file_name)
        # Move auto-generated file to target folder
        shutil.move(constraints_penalty_file_name,
                    os.path.join(icasadi_extern_dir, _AUTOGEN_PNLT_CONSTRAINTS_FNAME))

        self.__generate_memory_code(cost_fun, grad_cost_fun, alm_mapping_f1_fun, constraint_penalty_fun)

    def __build_icasadi(self):
        icasadi_dir = self.__icasadi_target_dir()
        command = self.__make_build_command()
        p = subprocess.Popen(command, cwd=icasadi_dir)
        p.wait()

    def __generate_main_project_code(self):
        logging.info("Generating main code for target optimizer (lib.rs)")
        target_dir = self.__target_dir()
        file_loader = jinja2.FileSystemLoader(og_dfn.templates_dir())
        env = jinja2.Environment(loader=file_loader)
        template = env.get_template('optimizer.rs.template')
        output_template = template.render(solver_config=self.__solver_config,
                                          meta=self.__meta,
                                          problem=self.__problem,
                                          timestamp_created=datetime.datetime.now(),
                                          activate_clib_generation=self.__build_config.build_c_bindings)
        target_source_path = os.path.join(target_dir, "src")
        target_scr_lib_rs_path = os.path.join(target_source_path, "lib.rs")
        make_dir_if_not_exists(target_source_path)
        with open(target_scr_lib_rs_path, "w") as fh:
            fh.write(output_template)

    def __generate_build_rs(self):
        logging.info("Generating build.rs for target optimizer")
        target_dir = self.__target_dir()
        file_loader = jinja2.FileSystemLoader(og_dfn.templates_dir())
        env = jinja2.Environment(loader=file_loader)
        template = env.get_template('optimizer_build.rs.template')
        output_template = template.render(meta=self.__meta,
                                          activate_clib_generation=self.__build_config.build_c_bindings)
        target_scr_lib_rs_path = os.path.join(target_dir, "build.rs")
        with open(target_scr_lib_rs_path, "w") as fh:
            fh.write(output_template)

    def __build_optimizer(self):
        target_dir = os.path.abspath(self.__target_dir())
        command = self.__make_build_command()
        p = subprocess.Popen(command, cwd=target_dir)
        process_completion = p.wait()
        if process_completion != 0:
            raise Exception('Rust build failed')

    def __build_tcp_iface(self):
        logging.info("Building the TCP interface")
        target_dir = os.path.abspath(self.__target_dir())
        tcp_iface_dir = os.path.join(target_dir, "tcp_iface")
        command = self.__make_build_command()
        p = subprocess.Popen(command, cwd=tcp_iface_dir)
        process_completion = p.wait()
        if process_completion != 0:
            raise Exception('Rust build of TCP interface failed')

    def __initialize(self):
        logging.info("Initialising builder")
        sc = self.__solver_config
        pr = self.__problem
        ncp = pr.dim_constraints_penalty()
        if ncp > 0 and sc.initial_penalty_weights is None:
            # set default initial values
            self.__solver_config.with_initial_penalty_weights([1] * int(ncp))

    def __check_user_provided_parameters(self):
        logging.info("Checking user parameters")
        sc = self.__solver_config
        pr = self.__problem
        ncp = pr.dim_constraints_penalty()
        if 1 != len(sc.initial_penalty_weights) != ncp > 0:
            raise Exception("Initial penalty weights have incompatible dimensions with c(u, p)")

    def __generate_code_tcp_interface(self):
        logging.info("Generating code for TCP/IP interface (tcp_iface/src/main.rs)")
        logging.info("TCP server will bind at %s:%d",
                     self.__build_config.tcp_interface_config.bind_ip,
                     self.__build_config.tcp_interface_config.bind_port)
        target_dir = self.__target_dir()
        tcp_iface_dir = os.path.join(target_dir, "tcp_iface")
        tcp_iface_source_dir = os.path.join(tcp_iface_dir, "src")

        # make tcp_iface/ and tcp_iface/src
        make_dir_if_not_exists(tcp_iface_dir)
        make_dir_if_not_exists(tcp_iface_source_dir)

        # generate main.rs for tcp_iface
        file_loader = jinja2.FileSystemLoader(og_dfn.templates_dir())
        env = jinja2.Environment(loader=file_loader)
        template = env.get_template('tcp_server.rs.template')
        output_template = template.render(meta=self.__meta,
                                          tcp_server_config=self.__build_config.tcp_interface_config,
                                          timestamp_created=datetime.datetime.now())
        target_scr_lib_rs_path = os.path.join(tcp_iface_source_dir, "main.rs")
        with open(target_scr_lib_rs_path, "w") as fh:
            fh.write(output_template)

        # generate Cargo.toml for tcp_iface
        template = env.get_template('tcp_server_cargo.toml.template')
        output_template = template.render(meta=self.__meta, timestamp_created=datetime.datetime.now())
        target_scr_lib_rs_path = os.path.join(tcp_iface_dir, "Cargo.toml")
        with open(target_scr_lib_rs_path, "w") as fh:
            fh.write(output_template)

    def __generate_yaml_data_file(self):
        logging.info("Generating YAML configuration file")
        tcp_config = self.__build_config.tcp_interface_config
        metadata = self.__meta
        build_config = self.__build_config
        solver_config = self.__solver_config
        target_dir = self.__target_dir()

        target_yaml_file_path = os.path.join(target_dir, "optimizer.yml")

        tcp_details = None if tcp_config is None \
            else {'ip': tcp_config.bind_ip, 'port': tcp_config.bind_port}
        metadata_details = {'optimizer_name': metadata.optimizer_name,
                            'version': metadata.version,
                            'authors': metadata.authors,
                            'licence': metadata.licence}
        build_details = {'open_version': build_config.open_version,
                         'build_dir': build_config.build_dir,
                         'build_mode': build_config.build_mode,
                         'target_system': build_config.target_system,
                         'cost_function_name': build_config.cost_function_name,
                         'grad_function_name': build_config.grad_function_name,
                         'constraint_penalty_function_name': build_config.constraint_penalty_function_name
                         }
        solver_details = {'initial_penalty_weights': solver_config.initial_penalty_weights,
                          'lbfgs_memory': solver_config.lbfgs_memory,
                          'tolerance': solver_config.tolerance,
                          'constraints_tolerance': solver_config.constraints_tolerance,
                          'penalty_weight_update_factor': solver_config.penalty_weight_update_factor,
                          'max_outer_iterations': solver_config.max_outer_iterations,
                          'max_inner_iterations': solver_config.max_inner_iterations,
                          'max_duration_micros': solver_config.max_duration_micros
                          }
        casadi_functions = {'cost_function_name': solver_config.initial_penalty_weights,
                            'cost_gradient_name': solver_config.lbfgs_memory,
                            'constraint_penalty_function_name': solver_config.tolerance
                            }
        details = {'meta': metadata_details, 'tcp': tcp_details, 'build': build_details,
                   'solver': solver_details}
        with open(target_yaml_file_path, 'w') as outfile:
            yaml.dump(details, outfile, Dumper=yaml.Dumper)

    def enable_tcp_interface(self,
                             tcp_server_configuration=og_cfg.TcpServerConfiguration()):
        warnings.warn("deprecated (use BuildConfiguration.with_tcp_interface_config instead)",
                      DeprecationWarning)
        self.__build_config.with_tcp_interface_config(tcp_server_configuration)

    def enable_c_bindings_generation(self):
        """Generate C/C++ bindings together with statically and dynamically linkable libraries

        """
        warnings.warn("deprecated (use BuildConfiguration.with_build_c_bindings([True]/False) instead)",
                      DeprecationWarning)
        self.__build_config.with_build_c_bindings(True)

    def build(self):
        """Generate code and build project

        Raises:
            Exception: if the build process fails
            Exception: if there some parameters have wrong, inadmissible or incompatible values

        """
        logging.basicConfig(level=logging.DEBUG)
        self.__initialize()                      # initialize default value (if not provided)
        self.__check_user_provided_parameters()  # check the provided parameters
        self.__prepare_target_project()          # create folders; init cargo project
        self.__copy_icasadi_to_target()          # copy icasadi/ files to target dir
        self.__generate_cargo_toml()             # generate Cargo.toml using template
        self.__generate_icasadi_lib()            # generate icasadi lib.rs
        self.__generate_casadi_code()            # generate all necessary CasADi C files
        self.__generate_icasadi_c_interface()  # generate icasadi/extern/icallocator.c
        self.__generate_main_project_code()      # generate main part of code (at build/{name}/src/main.rs)
        self.__generate_build_rs()               # generate build.rs file
        self.__generate_yaml_data_file()

        if not self.__generate_not_build:
            logging.info("Building optimizer")
            self.__build_optimizer()             # build overall project
        if self.__build_config.tcp_interface_config is not None:
            logging.info("Generating TCP/IP server")
            self.__generate_code_tcp_interface()
            if not self.__generate_not_build:
                self.__build_tcp_iface()
