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
import pkg_resources

from .ros_builder import RosBuilder

_AUTOGEN_COST_FNAME = 'auto_casadi_cost.c'
_AUTOGEN_GRAD_FNAME = 'auto_casadi_grad.c'
_AUTOGEN_PNLT_CONSTRAINTS_FNAME = 'auto_casadi_mapping_f2.c'
_AUTOGEN_ALM_MAPPING_F1_FNAME = 'auto_casadi_mapping_f1.c'
_TCP_IFACE_PREFIX = 'tcp_iface_'
_ICASADI_PREFIX = 'icasadi_'
_ROS_PREFIX = 'ros_node_'


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
        self.__logger = logging.getLogger('opengen.builder.OpEnOptimizerBuilder')
        self.with_verbosity_level(1)

    @staticmethod
    def __get_template(name, subdir=None):
        file_loader = jinja2.FileSystemLoader(og_dfn.templates_subdir(subdir))
        env = jinja2.Environment(loader=file_loader, autoescape=True)
        return env.get_template(name)

    def with_verbosity_level(self, verbosity_level):
        """Specify the verbosity level

        :param verbosity_level: level of verbosity (0,1,2,3)

        :returns: Current builder object
        """
        stream_handler = logging.StreamHandler()
        stream_handler.setLevel(verbosity_level)
        c_format = logging.Formatter('[%(levelname)s] %(message)s')
        stream_handler.setFormatter(c_format)
        self.__logger.setLevel(verbosity_level)
        self.__logger.handlers.clear()
        self.__logger.addHandler(stream_handler)

        return self

    def with_problem(self, problem):
        """Specify problem

        :param problem: optimization problem data

        :returns: Current builder object
        """
        self.__problem = problem
        return self

    def with_generate_not_build_flag(self, flag):
        """Whether to build (or just generate code)

        If set to true, the code will be generated, but it will not be
        build (mainly for debugging purposes)

        :param flag: generate and not build

        :returns: Current builder object
        """
        self.__generate_not_build = flag
        return self

    def __make_build_command(self):
        """
        Cargo build command (possibly, with --release)

        """
        command = ['cargo', 'build', '-q']
        if self.__build_config.build_mode.lower() == 'release':
            command.append('--release')

        if self.__build_config.target_system is not None:
            command.append('--target='+self.__build_config.target_system)
        return command

    def __target_dir(self):
        """

        Target directory

        """
        return os.path.abspath(
            os.path.join(
                self.__build_config.build_dir,
                self.__meta.optimizer_name))

    def __icasadi_target_dir(self):
        """

        Returns icasadi target directory (instance of os.path)

        """
        icasadi_target_dir_name = _ICASADI_PREFIX + self.__meta.optimizer_name
        return os.path.abspath(
            os.path.join(
                self.__build_config.build_dir,
                self.__meta.optimizer_name, icasadi_target_dir_name))

    def __ros_target_dir(self):
        ros_target_dir_name = _ROS_PREFIX + self.__meta.optimizer_name
        return os.path.abspath(
            os.path.join(
                self.__build_config.build_dir,
                self.__meta.optimizer_name, ros_target_dir_name))

    def __prepare_target_project(self):
        """Creates folder structure

        Creates necessary folders
        Runs `cargo init` in that folder

        """
        self.__logger.info("Creating necessary folders")

        # Create target directory if it does not exist
        target_dir = self.__target_dir()
        if self.__build_config.rebuild:
            if os.path.exists(target_dir) and os.path.isdir(target_dir):
                shutil.rmtree(target_dir)
            os.makedirs(target_dir)
        else:
            make_dir_if_not_exists(target_dir)

    def __copy_icasadi_to_target(self):
        """

        Copy 'icasadi' folder and its contents into target directory

        """
        self.__logger.info("Copying icasadi interface to target directory")
        origin_icasadi_dir = og_dfn.original_icasadi_dir()
        target_icasadi_dir = self.__icasadi_target_dir()
        if not os.path.exists(target_icasadi_dir):
            os.makedirs(target_icasadi_dir)
        shutil.rmtree(target_icasadi_dir)
        shutil.copytree(origin_icasadi_dir,
                        target_icasadi_dir,
                        ignore=shutil.ignore_patterns(
                            '*.lock', 'ci*', 'target', 'auto*'))

    def __generate_icasadi_cargo_toml(self):
        """
        Generate icasadi's Cargo.toml file
        """
        self.__logger.info("Generating icasadi's Cargo.toml")
        icasadi_cargo_template = OpEnOptimizerBuilder.__get_template('icasadi_cargo.toml', subdir='icasadi')
        icasadi_cargo_output_template = icasadi_cargo_template.render(meta=self.__meta)
        icasadi_cargo_allocator_path = os.path.abspath(
            os.path.join(self.__icasadi_target_dir(), "Cargo.toml"))
        with open(icasadi_cargo_allocator_path, "w") as fh:
            fh.write(icasadi_cargo_output_template)

    def __generate_icasadi_c_interface(self):
        """
        Generates the C interface file interface.c

        """
        self.__logger.info("Generating intercafe.c (C interface)")
        cint_template = OpEnOptimizerBuilder.__get_template('interface.c', 'icasadi')
        cint_output_template = cint_template.render(
            meta=self.__meta,
            problem=self.__problem,
            build_config=self.__build_config,
            timestamp_created=datetime.datetime.now())
        cint_icallocator_path = os.path.abspath(
            os.path.join(self.__icasadi_target_dir(), "extern", "interface.c"))
        with open(cint_icallocator_path, "w") as fh:
            fh.write(cint_output_template)

    def __generate_icasadi_lib(self):
        """
        Generates the Rust library file of icasadi

        Generates src/lib.rs
        """
        self.__logger.info("Generating icasadi Rust library file")
        icasadi_lib_template = OpEnOptimizerBuilder.__get_template('icasadi_lib.rs', 'icasadi')
        icasadi_lib_output_template = icasadi_lib_template.render(meta=self.__meta,
                                          problem=self.__problem,
                                          build_config=self.__build_config,
                                          timestamp_created=datetime.datetime.now())
        icasadi_lib_rs_path = os.path.abspath(
            os.path.join(self.__icasadi_target_dir(), "src", "lib.rs"))
        with open(icasadi_lib_rs_path, "w") as fh:
            fh.write(icasadi_lib_output_template)

    def __generate_cargo_toml(self):
        """
        Generates Cargo.toml for generated project

        """
        self.__logger.info("Generating Cargo.toml for target optimizer")
        target_dir = self.__target_dir()
        cargo_template = OpEnOptimizerBuilder.__get_template('optimizer_cargo.toml')
        cargo_output_template = cargo_template.render(
            meta=self.__meta,
            build_config=self.__build_config,
            activate_tcp_server=self.__build_config.tcp_interface_config is not None,
            activate_clib_generation=self.__build_config.build_c_bindings)
        cargo_toml_path = os.path.abspath(os.path.join(target_dir, "Cargo.toml"))
        with open(cargo_toml_path, "w") as fh:
            fh.write(cargo_output_template)

    def __generate_memory_code(self, cost=None, grad=None, f1=None, f2=None):
        """
        Creates file casadi_memory.h with memory sizes

        :param cost: cost function (cs.Function)
        :param grad: grad function (cs.Function)
        :param f1: mapping F1  (cs.Function)
        :param f2: mapping F2  (cs.Function)
        """
        self.__logger.info("Generating casadi_memory.h")
        casadi_mem_template = OpEnOptimizerBuilder.__get_template('casadi_memory.h', 'icasadi')
        casadi_mem_output_template = casadi_mem_template.render(
            cost=cost, grad=grad,
            f1=f1, f2=f2,
            build_config=self.__build_config,
            meta=self.__meta,
            timestamp_created=datetime.datetime.now())
        memory_path = os.path.abspath(
            os.path.join(self.__icasadi_target_dir(), "extern", "casadi_memory.h"))
        with open(memory_path, "w") as fh:
            fh.write(casadi_mem_output_template)

    def __construct_function_psi(self):
        """
        Construct function psi and its gradient

        :return: cs.Function objects: psi_fun, grad_psi_fun
        """
        self.__logger.info("Defining function psi(u, xi, p) and its gradient")
        problem = self.__problem
        bconfig = self.__build_config
        u = problem.decision_variables
        p = problem.parameter_variables
        n2 = problem.dim_constraints_penalty()
        n1 = problem.dim_constraints_aug_lagrangian()
        phi = problem.cost_function
        alm_set_c = problem.alm_set_c
        f1 = problem.penalty_mapping_f1
        f2 = problem.penalty_mapping_f2

        psi = phi

        if n1 + n2 > 0:
            n_xi = n1 + 1
        else:
            n_xi = 0

        xi = cs.SX.sym('xi', n_xi, 1) if isinstance(u, cs.SX) \
            else cs.MX.sym('xi', n_xi, 1)

        # Note: In the first term below, we divide by 'max(c, 1)', instead of
        #       just 'c'. The reason is that this allows to set c=0 and
        #       retrieve the value of the original cost function
        if n1 > 0:
            sq_dist_term = alm_set_c.distance_squared(f1 + xi[1:n1+1]/cs.fmax(xi[0], 1))
            psi += xi[0] * sq_dist_term / 2

        if n2 > 0:
            psi += xi[0] * cs.dot(f2, f2) / 2

        jac_psi = cs.gradient(psi, u)

        psi_fun = cs.Function(bconfig.cost_function_name, [u, xi, p], [psi])
        grad_psi_fun = cs.Function(bconfig.grad_function_name, [u, xi, p], [jac_psi])
        return psi_fun, grad_psi_fun

    def __construct_mapping_f1_function(self) -> cs.Function:
        self.__logger.info("Defining function F1(u, p)")
        problem = self.__problem
        u = problem.decision_variables
        p = problem.parameter_variables
        n1 = problem.dim_constraints_aug_lagrangian()
        f1 = problem.penalty_mapping_f1

        if n1 > 0:
            mapping_f1 = f1
        else:
            mapping_f1 = 0

        alm_mapping_f1_fun = cs.Function(
            self.__build_config.alm_mapping_f1_function_name,
            [u, p], [mapping_f1])
        return alm_mapping_f1_fun

    def __construct_mapping_f2_function(self) -> cs.Function:
        self.__logger.info("Defining function F2(u, p)")
        problem = self.__problem
        u = problem.decision_variables
        p = problem.parameter_variables
        n2 = problem.dim_constraints_penalty()

        if n2 > 0:
            penalty_constraints = problem.penalty_mapping_f2
        else:
            penalty_constraints = 0

        alm_mapping_f2_fun = cs.Function(
            self.__build_config.constraint_penalty_function_name,
            [u, p], [penalty_constraints])

        return alm_mapping_f2_fun

    def __generate_casadi_code(self):
        """Generates CasADi C code"""
        self.__logger.info("Defining CasADi functions and generating C code")
        bconfig = self.__build_config

        # -----------------------------------------------------------------------
        psi_fun, grad_psi_fun = self.__construct_function_psi()
        cost_file_name = bconfig.cost_function_name + ".c"
        grad_file_name = bconfig.grad_function_name + ".c"
        self.__logger.info("Function psi and its gradient (C code)")
        psi_fun.generate(cost_file_name)
        grad_psi_fun.generate(grad_file_name)
        icasadi_extern_dir = os.path.join(self.__icasadi_target_dir(), "extern")
        shutil.move(cost_file_name, os.path.join(icasadi_extern_dir, _AUTOGEN_COST_FNAME))
        shutil.move(grad_file_name, os.path.join(icasadi_extern_dir, _AUTOGEN_GRAD_FNAME))

        # -----------------------------------------------------------------------
        mapping_f1_fun = self.__construct_mapping_f1_function()
        f1_file_name = bconfig.alm_mapping_f1_function_name + ".c"
        self.__logger.info("Mapping F1 (C code)")
        mapping_f1_fun.generate(f1_file_name)
        # Move auto-generated file to target folder
        shutil.move(f1_file_name,
                    os.path.join(icasadi_extern_dir, _AUTOGEN_ALM_MAPPING_F1_FNAME))

        # -----------------------------------------------------------------------
        mapping_f2_fun = self.__construct_mapping_f2_function()
        f2_file_name = bconfig.constraint_penalty_function_name + ".c"
        self.__logger.info("Mapping F2 (C code)")
        mapping_f2_fun.generate(f2_file_name)
        # Move auto-generated file to target folder
        shutil.move(f2_file_name,
                    os.path.join(icasadi_extern_dir, _AUTOGEN_PNLT_CONSTRAINTS_FNAME))

        self.__generate_memory_code(psi_fun, grad_psi_fun,
                                    mapping_f1_fun, mapping_f2_fun)

    def __generate_main_project_code(self):
        self.__logger.info("Generating main code for target optimizer (lib.rs)")
        target_dir = self.__target_dir()
        optimizer_rs_template = OpEnOptimizerBuilder.__get_template('optimizer.rs')
        optimizer_rs_output_template = optimizer_rs_template.render(
                                          solver_config=self.__solver_config,
                                          meta=self.__meta,
                                          problem=self.__problem,
                                          timestamp_created=datetime.datetime.now(),
                                          activate_clib_generation=self.__build_config.build_c_bindings,
                                          float=float)
        target_source_path = os.path.join(target_dir, "src")
        target_scr_lib_rs_path = os.path.join(target_source_path, "lib.rs")
        make_dir_if_not_exists(target_source_path)
        with open(target_scr_lib_rs_path, "w") as fh:
            fh.write(optimizer_rs_output_template)

    def __generate_build_rs(self):
        self.__logger.info("Generating build.rs for target optimizer")
        target_dir = self.__target_dir()
        build_rs_template = OpEnOptimizerBuilder.__get_template('optimizer_build.rs')
        build_rs_output_template = build_rs_template.render(
            meta=self.__meta,
            activate_clib_generation=self.__build_config.build_c_bindings)
        target_build_lib_rs_path = os.path.join(target_dir, "build.rs")
        with open(target_build_lib_rs_path, "w") as fh:
            fh.write(build_rs_output_template)

    def __build_optimizer(self):
        target_dir = os.path.abspath(self.__target_dir())
        command = self.__make_build_command()
        p = subprocess.Popen(command, cwd=target_dir)
        process_completion = p.wait()
        if process_completion != 0:
            raise Exception('Rust build failed')

    def __build_tcp_iface(self):
        self.__logger.info("Building the TCP interface")
        target_dir = os.path.abspath(self.__target_dir())
        optimizer_name = self.__meta.optimizer_name
        tcp_iface_dir_name = _TCP_IFACE_PREFIX + optimizer_name
        tcp_iface_dir = os.path.join(target_dir, tcp_iface_dir_name)
        command = self.__make_build_command()
        p = subprocess.Popen(command, cwd=tcp_iface_dir)
        process_completion = p.wait()
        if process_completion != 0:
            raise Exception('Rust build of TCP interface failed')

    def __initialize(self):
        self.__logger.info("Initialising builder")

    def __check_user_provided_parameters(self):
        self.__logger.info("Checking user parameters")

    def __generate_code_tcp_interface(self):
        self.__logger.info("Generating code for TCP/IP interface (tcp_iface/src/main.rs)")
        self.__logger.info("TCP server will bind at %s:%d",
                           self.__build_config.tcp_interface_config.bind_ip,
                           self.__build_config.tcp_interface_config.bind_port)
        target_dir = self.__target_dir()
        tcp_iface_dir_name = _TCP_IFACE_PREFIX + self.__meta.optimizer_name
        tcp_iface_dir = os.path.join(target_dir, tcp_iface_dir_name)
        tcp_iface_source_dir = os.path.join(tcp_iface_dir, "src")

        # make tcp_iface/ and tcp_iface/src
        make_dir_if_not_exists(tcp_iface_dir)
        make_dir_if_not_exists(tcp_iface_source_dir)

        # generate tcp_server.rs for tcp_iface
        tcp_rs_template = OpEnOptimizerBuilder.__get_template('tcp_server.rs', 'tcp')
        tcp_rs_output_template = tcp_rs_template.render(
            meta=self.__meta,
            tcp_server_config=self.__build_config.tcp_interface_config,
            timestamp_created=datetime.datetime.now())
        target_tcp_rs_path = os.path.join(tcp_iface_source_dir, "main.rs")
        with open(target_tcp_rs_path, "w") as fh:
            fh.write(tcp_rs_output_template)

        # generate Cargo.toml for tcp_iface
        tcp_rs_template = OpEnOptimizerBuilder.__get_template('tcp_server_cargo.toml', 'tcp')
        tcp_rs_output_template = tcp_rs_template.render(
            meta=self.__meta,
            build_config=self.__build_config,
            timestamp_created=datetime.datetime.now())
        target_tcp_rs_path = os.path.join(tcp_iface_dir, "Cargo.toml")
        with open(target_tcp_rs_path, "w") as fh:
            fh.write(tcp_rs_output_template)

    def __generate_yaml_data_file(self):
        self.__logger.info("Generating YAML configuration file")
        tcp_config = self.__build_config.tcp_interface_config
        metadata = self.__meta
        build_config = self.__build_config
        solver_config = self.__solver_config
        target_dir = self.__target_dir()

        target_yaml_file_path = os.path.join(target_dir, "optimizer.yml")

        opengen_version = pkg_resources.require("opengen")[0].version

        tcp_details = None if tcp_config is None \
            else {'ip': tcp_config.bind_ip, 'port': tcp_config.bind_port}
        metadata_details = {'optimizer_name': metadata.optimizer_name,
                            'version': metadata.version,
                            'authors': metadata.authors,
                            'licence': metadata.licence}
        build_details = {'id': build_config.id,
                         'open_version': build_config.open_version,
                         'opengen_version': opengen_version,
                         'build_dir': build_config.build_dir,
                         'build_mode': build_config.build_mode,
                         'target_system': build_config.target_system,
                         'cost_function_name': build_config.cost_function_name,
                         'grad_function_name': build_config.grad_function_name,
                         'mapping_f2': build_config.constraint_penalty_function_name,
                         'mapping_f1': build_config.alm_mapping_f1_function_name
                         }
        solver_details = {'lbfgs_memory': solver_config.lbfgs_memory,
                          'tolerance': solver_config.tolerance,
                          'constraints_tolerance': solver_config.constraints_tolerance,
                          'penalty_weight_update_factor': solver_config.penalty_weight_update_factor,
                          'max_outer_iterations': solver_config.max_outer_iterations,
                          'max_inner_iterations': solver_config.max_inner_iterations,
                          'max_duration_micros': solver_config.max_duration_micros
                          }
        details = {'meta': metadata_details, 'tcp': tcp_details, 'build': build_details,
                   'solver': solver_details,
                   "_comment": "auto-generated file; do not modify"}
        with open(target_yaml_file_path, 'w') as outfile:
            yaml.dump(details, outfile, Dumper=yaml.Dumper)

    def enable_tcp_interface(self,
                             tcp_server_configuration=og_cfg.TcpServerConfiguration()):
        warnings.warn("deprecated (use BuildConfiguration.with_tcp_interface_config instead)",
                      DeprecationWarning)
        self.__build_config.with_tcp_interface_config(tcp_server_configuration)

    def __generate_c_bindings_example(self):
        self.__logger.info("Generating example_optimizer.c (C bindings example for your convenience)")
        target_dir = self.__target_dir()
        cbind_template = OpEnOptimizerBuilder.__get_template('example_optimizer_c_bindings.c', 'c')
        cbind_output_template = cbind_template.render(
            meta=self.__meta,
            build_config=self.__build_config,
            problem=self.__problem)
        cbind_target_path = os.path.join(target_dir, "example_optimizer.c")
        with open(cbind_target_path, "w") as fh:
            fh.write(cbind_output_template)

    def __generate_c_bindings_makefile(self):
        self.__logger.info("Generating CMakeLists (do: `cmake .; make` to compile the autogenerated example)")
        target_dir = self.__target_dir()
        cbind_makefile_template = OpEnOptimizerBuilder.__get_template('example_cmakelists.txt', 'c')
        cbind_makefile_output_template \
            = cbind_makefile_template.render(meta=self.__meta,
                                             build_config=self.__build_config)
        cbind_makefile_target_path = os.path.join(target_dir, "CMakeLists.txt")
        with open(cbind_makefile_target_path, "w") as fh:
            fh.write(cbind_makefile_output_template)

    def build(self):
        """Generate code and build project

        Raises:
            Exception: if the build process fails
            Exception: if there some parameters have wrong, inadmissible or incompatible values

        """
        self.__initialize()                      # initialize default value (if not provided)
        self.__check_user_provided_parameters()  # check the provided parameters
        self.__prepare_target_project()          # create folders; init cargo project
        self.__copy_icasadi_to_target()          # copy icasadi/ files to target dir
        self.__generate_icasadi_cargo_toml()     # generate icasadi's Cargo.toml file
        self.__generate_cargo_toml()             # generate Cargo.toml using template
        self.__generate_icasadi_lib()            # generate icasadi lib.rs
        self.__generate_casadi_code()            # generate all necessary CasADi C files:
        #                                        #   - auto_casadi_cost.c
        #                                        #   - auto_casadi_grad.c
        #                                        #   - auto_casadi_mapping_f1.c
        #                                        #   - auto_casadi_mapping_f2.c
        #                                        #   - casadi_memory.h
        self.__generate_icasadi_c_interface()    # generate icasadi/extern/interface.c
        self.__generate_main_project_code()      # generate main part of code (at build/{name}/src/main.rs)
        self.__generate_build_rs()               # generate build.rs file
        self.__generate_yaml_data_file()         # create YAML file with metadata

        if not self.__generate_not_build:
            self.__logger.info("Building optimizer")
            self.__build_optimizer()             # build overall project

        if self.__build_config.tcp_interface_config is not None:
            self.__logger.info("Generating TCP/IP server")
            self.__generate_code_tcp_interface()
            if not self.__generate_not_build:
                self.__build_tcp_iface()

        if self.__build_config.build_c_bindings:
            self.__generate_c_bindings_example()
            self.__generate_c_bindings_makefile()

        if self.__build_config.ros_config is not None:
            ros_builder = RosBuilder(
                self.__meta,
                self.__build_config,
                self.__solver_config)
            ros_builder.build()
