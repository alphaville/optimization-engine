import os
import unittest
import json
import socket
import shutil
import sys
import importlib
import casadi.casadi as cs
import opengen as og
import subprocess
import logging
import numpy as np
from pathlib import PureWindowsPath





class BuildConfigurationTestCase(unittest.TestCase):

    def test_local_path_is_toml_safe_on_windows(self):
        # some windows-type path...
        windows_style_path = PureWindowsPath("C:/temp/optimization-engine")
        build_config = og.config.BuildConfiguration() \
            .with_open_version(local_path=windows_style_path)

        self.assertEqual(
            "C:/temp/optimization-engine",
            build_config.local_path
        )


class RustBuildTestCase(unittest.TestCase):

    TEST_DIR = ".python_test_build"

    @staticmethod
    def get_open_local_absolute_path():
        cwd = os.getcwd()
        return cwd.split('open-codegen')[0]

    # Which version of OpEn Rust library to test against
    OPEN_RUSTLIB_VERSION = "*"

    @classmethod
    def solverConfig(cls):
        solver_config = og.config.SolverConfiguration() \
            .with_lbfgs_memory(15) \
            .with_tolerance(1e-4) \
            .with_initial_tolerance(1e-4) \
            .with_delta_tolerance(1e-4) \
            .with_initial_penalty(15.0) \
            .with_penalty_weight_update_factor(10.0) \
            .with_max_inner_iterations(155) \
            .with_max_duration_micros(1e8) \
            .with_max_outer_iterations(50) \
            .with_sufficient_decrease_coefficient(0.05) \
            .with_cbfgs_parameters(1.5, 1e-10, 1e-12) \
            .with_preconditioning(False)
        return solver_config

    @classmethod
    def setUpPythonBindings(cls):
        u = cs.MX.sym("u", 5)  # decision variable (nu = 5)
        p = cs.MX.sym("p", 2)  # parameter (np = 2)
        phi = og.functions.rosenbrock(u, p)  # cost function
        bounds = og.constraints.Ball2(None, 1.5)  # ball centered at origin
        meta = og.config.OptimizerMeta() \
            .with_optimizer_name("python_bindings")
        problem = og.builder.Problem(u, p, phi) \
            .with_constraints(bounds)
        build_config = og.config.BuildConfiguration() \
            .with_open_version(local_path=RustBuildTestCase.get_open_local_absolute_path()) \
            .with_build_directory(RustBuildTestCase.TEST_DIR) \
            .with_build_mode(og.config.BuildConfiguration.DEBUG_MODE)\
            .with_build_python_bindings()
        og.builder.OpEnOptimizerBuilder(problem,
                                        metadata=meta,
                                        build_configuration=build_config,
                                        solver_configuration=cls.solverConfig()) \
            .build()

        target_lib = os.path.join(
            RustBuildTestCase.TEST_DIR, "python_bindings", "src", "lib.rs")
        with open(target_lib, "r", encoding="utf-8") as fh:
            solver_lib = fh.read()

        anchor = (
            '    assert_eq!(u.len(), PYTHON_BINDINGS_NUM_DECISION_VARIABLES, '
            '"Wrong number of decision variables (u)");\n'
        )
        injected_guard = (
            anchor +
            '\n'
            '    if p[0] < 0.0 {\n'
            '        return Err(SolverError::Cost("forced solver error for Python bindings test"));\n'
            '    }\n'
        )
        if anchor not in solver_lib:
            raise RuntimeError("Could not inject deterministic solver error into python_bindings")

        with open(target_lib, "w", encoding="utf-8") as fh:
            fh.write(solver_lib.replace(anchor, injected_guard, 1))

        python_bindings_dir = os.path.join(
            RustBuildTestCase.TEST_DIR, "python_bindings", "python_bindings_python_bindings")
        process = subprocess.Popen(
            ["cargo", "build"],
            cwd=python_bindings_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        _stdout, stderr = process.communicate()
        if process.returncode != 0:
            raise RuntimeError(
                "Could not rebuild Python bindings:\n{}".format(stderr.decode())
            )

        extension_dict = {'linux': ('.so', '.so'),
                          'darwin': ('.dylib', '.so'),
                          'win32': ('.dll', '.pyd')}
        (original_lib_extension,
         target_lib_extension) = extension_dict[sys.platform]
        optimizer_prefix = "lib" if sys.platform != "win32" else ""
        generated_bindings = os.path.join(
            python_bindings_dir,
            "target",
            "debug",
            "{}python_bindings{}".format(optimizer_prefix, original_lib_extension),
        )
        target_bindings = os.path.join(
            RustBuildTestCase.TEST_DIR,
            "python_bindings",
            "python_bindings{}".format(target_lib_extension))
        shutil.copyfile(generated_bindings, target_bindings)

    @classmethod
    def setUpOnlyF1(cls):
        u = cs.MX.sym("u", 5)  # decision variable (nu = 5)
        p = cs.MX.sym("p", 2)  # parameter (np = 2)
        f1 = cs.vertcat(1.5 * u[0] - u[1], u[2] - u[3])
        set_c = og.constraints.Rectangle(
            xmin=[-0.01, -0.01], xmax=[0.02, 0.03])
        phi = og.functions.rosenbrock(u, p)  # cost function
        bounds = og.constraints.Ball2(None, 1.5)  # ball centered at origin
        tcp_config = og.config.TcpServerConfiguration(bind_port=3301)
        meta = og.config.OptimizerMeta() \
            .with_optimizer_name("only_f1")
        problem = og.builder.Problem(u, p, phi) \
            .with_aug_lagrangian_constraints(f1, set_c) \
            .with_constraints(bounds)
        build_config = og.config.BuildConfiguration() \
            .with_open_version(local_path=RustBuildTestCase.get_open_local_absolute_path())  \
            .with_build_directory(RustBuildTestCase.TEST_DIR) \
            .with_build_mode(og.config.BuildConfiguration.DEBUG_MODE) \
            .with_tcp_interface_config(tcp_interface_config=tcp_config) \
            .with_build_c_bindings()  \
            .with_allocator(og.config.RustAllocator.JemAlloc)
        og.builder.OpEnOptimizerBuilder(problem,
                                        metadata=meta,
                                        build_configuration=build_config,
                                        solver_configuration=cls.solverConfig()) \
            .build()

    @classmethod
    def setUpOnlyF2(cls, is_preconditioned=False):
        u = cs.MX.sym("u", 5)  # decision variable (nu = 5)
        p = cs.MX.sym("p", 2)  # parameter (np = 2)
        f2 = cs.vertcat(0.2 + 1.5 * u[0] - u[1], u[2] - u[3] - 0.1)
        phi = og.functions.rosenbrock(u, p)
        bounds = og.constraints.Ball2(None, 1.5)
        tcp_config = og.config.TcpServerConfiguration(
            bind_port=3302 if not is_preconditioned else 3309)
        meta = og.config.OptimizerMeta() \
            .with_optimizer_name("only_f2" + ("_precond" if is_preconditioned else ""))
        problem = og.builder.Problem(u, p, phi) \
            .with_penalty_constraints(f2) \
            .with_constraints(bounds)
        build_config = og.config.BuildConfiguration() \
            .with_open_version(local_path=RustBuildTestCase.get_open_local_absolute_path())  \
            .with_build_directory(RustBuildTestCase.TEST_DIR) \
            .with_build_mode(og.config.BuildConfiguration.DEBUG_MODE) \
            .with_tcp_interface_config(tcp_interface_config=tcp_config) \
            .with_build_c_bindings()
        slv_cfg = og.config.SolverConfiguration() \
            .with_tolerance(1e-6) \
            .with_initial_tolerance(1e-4) \
            .with_delta_tolerance(1e-5) \
            .with_penalty_weight_update_factor(10.0) \
            .with_max_inner_iterations(1000) \
            .with_max_outer_iterations(50) \
            .with_preconditioning(is_preconditioned)
        og.builder.OpEnOptimizerBuilder(problem,
                                        metadata=meta,
                                        build_configuration=build_config,
                                        solver_configuration=slv_cfg) \
            .build()

    @classmethod
    def setUpPlain(cls):
        u = cs.MX.sym("u", 5)  # decision variable (nu = 5)
        p = cs.MX.sym("p", 2)  # parameter (np = 2)
        phi = og.functions.rosenbrock(u, p)
        bounds = og.constraints.Ball2(None, 1.5)
        tcp_config = og.config.TcpServerConfiguration(bind_port=4598)
        meta = og.config.OptimizerMeta() \
            .with_optimizer_name("plain")
        problem = og.builder.Problem(u, p, phi) \
            .with_constraints(bounds)
        build_config = og.config.BuildConfiguration() \
            .with_open_version(local_path=RustBuildTestCase.get_open_local_absolute_path())  \
            .with_build_directory(RustBuildTestCase.TEST_DIR) \
            .with_build_mode(og.config.BuildConfiguration.DEBUG_MODE) \
            .with_tcp_interface_config(tcp_interface_config=tcp_config) \
            .with_build_c_bindings()
        og.builder.OpEnOptimizerBuilder(problem,
                                        metadata=meta,
                                        build_configuration=build_config,
                                        solver_configuration=cls.solverConfig()) \
            .build()

    @classmethod
    def setUpRosPackageGeneration(cls):
        u = cs.MX.sym("u", 5)  # decision variable (nu = 5)
        p = cs.MX.sym("p", 2)  # parameter (np = 2)
        phi = og.functions.rosenbrock(u, p)
        c = cs.vertcat(1.5 * u[0] - u[1],
                       cs.fmax(0.0, u[2] - u[3] + 0.1))
        bounds = og.constraints.Ball2(None, 1.5)
        meta = og.config.OptimizerMeta() \
            .with_optimizer_name("rosenbrock_ros")
        problem = og.builder.Problem(u, p, phi) \
            .with_constraints(bounds) \
            .with_penalty_constraints(c)
        ros_config = og.config.RosConfiguration() \
            .with_package_name("parametric_optimizer") \
            .with_node_name("open_node") \
            .with_rate(35) \
            .with_description("really cool ROS node")
        build_config = og.config.BuildConfiguration() \
            .with_open_version(local_path=RustBuildTestCase.get_open_local_absolute_path())  \
            .with_build_directory(RustBuildTestCase.TEST_DIR) \
            .with_build_mode(og.config.BuildConfiguration.DEBUG_MODE) \
            .with_build_c_bindings()  \
            .with_ros(ros_config)
        og.builder.OpEnOptimizerBuilder(problem,
                                        metadata=meta,
                                        build_configuration=build_config,
                                        solver_configuration=cls.solverConfig()) \
            .build()

    @classmethod
    def setUpOnlyParametricF2(cls):
        u = cs.MX.sym("u", 5)  # decision variable (nu = 5)
        p = cs.MX.sym("p", 3)  # parameter (np = 3)
        f2 = u[0] - p[2]
        phi = og.functions.rosenbrock(
            u, cs.vertcat(p[0], p[1]))  # cost function
        bounds = og.constraints.Ball2(None, 1.5)  # ball centered at origin
        tcp_config = og.config.TcpServerConfiguration(bind_port=4599)
        meta = og.config.OptimizerMeta() \
            .with_optimizer_name("parametric_f2")
        problem = og.builder.Problem(u, p, phi) \
            .with_penalty_constraints(f2) \
            .with_constraints(bounds)
        build_config = og.config.BuildConfiguration() \
            .with_open_version(local_path=RustBuildTestCase.get_open_local_absolute_path()) \
            .with_build_mode(og.config.BuildConfiguration.DEBUG_MODE) \
            .with_build_directory(RustBuildTestCase.TEST_DIR) \
            .with_tcp_interface_config(tcp_interface_config=tcp_config) \
            .with_build_c_bindings()
        solver_config = og.config.SolverConfiguration() \
            .with_tolerance(1e-6) \
            .with_initial_tolerance(1e-4) \
            .with_delta_tolerance(1e-5) \
            .with_penalty_weight_update_factor(5)
        og.builder.OpEnOptimizerBuilder(
            problem, meta, build_config, solver_config).build()

    @classmethod
    def setUpHalfspace(cls):
        u = cs.SX.sym("u", 5)  # decision variable (nu = 5)
        p = cs.SX.sym("p", 2)  # parameter (np = 2)
        phi = cs.dot(u, u)  # cost function

        bounds = og.constraints.Halfspace([1., 2., 1., 5., 2.], -10.39)

        problem = og.builder.Problem(u, p, phi) \
            .with_constraints(bounds)

        meta = og.config.OptimizerMeta() \
            .with_optimizer_name("halfspace_optimizer")

        tcp_config = og.config.TcpServerConfiguration(bind_port=3305)
        build_config = og.config.BuildConfiguration() \
            .with_build_directory(RustBuildTestCase.TEST_DIR) \
            .with_build_mode(og.config.BuildConfiguration.DEBUG_MODE) \
            .with_open_version(local_path=RustBuildTestCase.get_open_local_absolute_path()) \
            .with_tcp_interface_config(tcp_interface_config=tcp_config)

        builder = og.builder.OpEnOptimizerBuilder(problem,
                                                  meta,
                                                  build_config,
                                                  cls.solverConfig())
        builder.build()

    @classmethod
    def setUpSolverError(cls):
        u = cs.MX.sym("u", 1)
        p = cs.MX.sym("p", 1)
        phi = cs.dot(u, u)
        bounds = og.constraints.Rectangle(xmin=[-1.0], xmax=[1.0])
        tcp_config = og.config.TcpServerConfiguration(bind_port=3310)
        meta = og.config.OptimizerMeta() \
            .with_optimizer_name("solver_error")
        problem = og.builder.Problem(u, p, phi) \
            .with_constraints(bounds)
        build_config = og.config.BuildConfiguration() \
            .with_open_version(local_path=RustBuildTestCase.get_open_local_absolute_path()) \
            .with_build_directory(RustBuildTestCase.TEST_DIR) \
            .with_build_mode(og.config.BuildConfiguration.DEBUG_MODE) \
            .with_tcp_interface_config(tcp_interface_config=tcp_config) \
            .with_build_c_bindings()
        og.builder.OpEnOptimizerBuilder(problem,
                                        metadata=meta,
                                        build_configuration=build_config,
                                        solver_configuration=cls.solverConfig()) \
            .build()

        target_lib = os.path.join(
            RustBuildTestCase.TEST_DIR, "solver_error", "src", "lib.rs")
        with open(target_lib, "r", encoding="utf-8") as fh:
            solver_lib = fh.read()

        # Look for this excerpt inside lib.rs (in the auto-generated solver)...
        anchor = (
            '    assert_eq!(u.len(), SOLVER_ERROR_NUM_DECISION_VARIABLES, '
            '"Wrong number of decision variables (u)");\n'
        )
        # Replace the anchor with this so that if p[0] < 0, the function `solve`
        # will reutrn an error of type SolverError::Cost
        injected_guard = (
            anchor +
            '\n'
            '    if p[0] < 0.0 {\n'
            '        return Err(SolverError::Cost("forced solver error for TCP test"));\n'
            '    }\n'
        )
        if anchor not in solver_lib:
            raise RuntimeError("Could not inject deterministic solver error")

        with open(target_lib, "w", encoding="utf-8") as fh:
            fh.write(solver_lib.replace(anchor, injected_guard, 1))

    @classmethod
    def setUpClass(cls):
        cls.setUpPythonBindings()
        cls.setUpRosPackageGeneration()
        cls.setUpOnlyF1()
        cls.setUpOnlyF2()
        cls.setUpOnlyF2(is_preconditioned=True)
        cls.setUpPlain()
        cls.setUpOnlyParametricF2()
        cls.setUpHalfspace()
        cls.setUpSolverError()

    @staticmethod
    def raw_tcp_request(ip, port, payload, buffer_size=4096):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0) as conn_socket:
            conn_socket.connect((ip, port))
            if isinstance(payload, str):
                payload = payload.encode()
            conn_socket.sendall(payload)
            conn_socket.shutdown(socket.SHUT_WR)

            data = b''
            while True:
                data_chunk = conn_socket.recv(buffer_size)
                if not data_chunk:
                    break
                data += data_chunk

        return json.loads(data.decode())

    @staticmethod
    def send_partial_tcp_payload_and_close(ip, port, payload):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0) as conn_socket:
            conn_socket.connect((ip, port))
            if isinstance(payload, str):
                payload = payload.encode()
            conn_socket.sendall(payload)

    @staticmethod
    def import_generated_module(module_dir, module_name):
        module_path = os.path.join(RustBuildTestCase.TEST_DIR, module_dir)
        if module_path not in sys.path:
            sys.path.insert(1, module_path)
        importlib.invalidate_caches()
        return importlib.import_module(module_name)

    def start_tcp_manager(self, manager):
        manager.start()
        self.addCleanup(manager.kill) # at the end, kill the TCP mngr
        return manager

    def test_python_bindings(self):
        python_bindings = RustBuildTestCase.import_generated_module(
            "python_bindings", "python_bindings")

        solver = python_bindings.solver()
        result = solver.run([1., 2.])
        self.assertTrue(result.is_ok())
        status = result.get()
        self.assertEqual("Converged", status.exit_status)
        self.assertIsNotNone(status.solution)

    def test_python_bindings_error_details(self):
        python_bindings = RustBuildTestCase.import_generated_module(
            "python_bindings", "python_bindings")

        solver = python_bindings.solver()
        result = solver.run([1., 2., 3.])
        self.assertFalse(result.is_ok())
        error = result.get()
        self.assertEqual(3003, error.code)
        self.assertEqual(
            "wrong number of parameters: provided 3, expected 2",
            error.message
        )

    def test_python_bindings_initial_guess_error_details(self):
        python_bindings = RustBuildTestCase.import_generated_module(
            "python_bindings", "python_bindings")

        solver = python_bindings.solver()
        result = solver.run([1., 2.], initial_guess=[0.0])
        self.assertFalse(result.is_ok())
        error = result.get()
        self.assertEqual(1600, error.code)
        self.assertEqual(
            "initial guess has incompatible dimensions: provided 1, expected 5",
            error.message
        )

    def test_python_bindings_initial_lagrange_multipliers_error_details(self):
        python_bindings = RustBuildTestCase.import_generated_module(
            "python_bindings", "python_bindings")

        solver = python_bindings.solver()
        result = solver.run([1., 2.], initial_lagrange_multipliers=[0.1])
        self.assertFalse(result.is_ok())
        error = result.get()
        self.assertEqual(1700, error.code)
        self.assertEqual(
            "wrong dimension of Langrange multipliers: provided 1, expected 0",
            error.message
        )

    def test_python_bindings_solver_error_details(self):
        python_bindings = RustBuildTestCase.import_generated_module(
            "python_bindings", "python_bindings")

        solver = python_bindings.solver()
        result = solver.run([-1.0, 2.0])
        self.assertFalse(result.is_ok())
        error = result.get()
        self.assertEqual(2000, error.code)
        self.assertEqual(
            "problem solution failed: cost or gradient evaluation failed: forced solver error for Python bindings test",
            error.message
        )

    def test_python_bindings_repr(self):
        python_bindings = RustBuildTestCase.import_generated_module(
            "python_bindings", "python_bindings")

        solver = python_bindings.solver()

        ok_response = solver.run([1., 2.])
        self.assertIn("SolverResponse(ok=True", repr(ok_response))
        ok_status = ok_response.get()
        self.assertIn("SolverStatus(", repr(ok_status))
        self.assertIn('exit_status="Converged"', repr(ok_status))

        error_response = solver.run([1., 2., 3.])
        self.assertIn("SolverResponse(ok=False", repr(error_response))
        error = error_response.get()
        self.assertIn("SolverError(", repr(error))
        self.assertIn("code=3003", repr(error))

    def test_tcp_response_repr(self):
        ok_response = og.tcp.SolverResponse({
            "exit_status": "Converged",
            "num_outer_iterations": 2,
            "num_inner_iterations": 7,
            "last_problem_norm_fpr": 1e-6,
            "delta_y_norm_over_c": 0.0,
            "f2_norm": 0.0,
            "solve_time_ms": 1.2,
            "penalty": 10.0,
            "solution": [0.1, 0.2],
            "lagrange_multipliers": [],
            "cost": 0.5,
        })
        self.assertIn("SolverResponse(ok=True", repr(ok_response))
        self.assertIn("exit_status='Converged'", repr(ok_response))

        error_response = og.tcp.SolverResponse({
            "type": "Error",
            "code": 3003,
            "message": "wrong number of parameters",
        })
        self.assertIn("SolverResponse(ok=False", repr(error_response))
        self.assertIn("code=3003", repr(error_response))
        self.assertIn("SolverError(", repr(error_response.get()))

    def test_rectangle_empty(self):
        xmin = [-1, 2]
        xmax = [-2, 4]
        with self.assertRaises(Exception) as __context:
            og.constraints.Rectangle(xmin, xmax)

    def test_rectangle_incompatible_dimensions(self):
        xmin = [-1, -1, 1]
        xmax = [1, 1]
        with self.assertRaises(Exception) as __context:
            og.constraints.Rectangle(xmin, xmax)

    def test_rectangle_both_none(self):
        with self.assertRaises(Exception) as __context:
            og.constraints.Rectangle(None, None)

    def test_ball_negative_radius(self):
        with self.assertRaises(Exception) as __context:
            og.constraints.Ball2(None, -1)

    def test_solver_config_wrong_max_duration(self):
        with self.assertRaises(Exception) as __context:
            og.config.SolverConfiguration().with_max_duration_micros(0)

    def test_solver_config_wrong_update_factor(self):
        with self.assertRaises(Exception) as __context:
            og.config.SolverConfiguration().with_penalty_weight_update_factor(0.5)

    def test_solver_config_wrong_outer_iterations(self):
        with self.assertRaises(Exception) as __context:
            og.config.SolverConfiguration().with_max_outer_iterations(0)

    def test_solver_config_wrong_inner_iterations(self):
        with self.assertRaises(Exception) as __context:
            og.config.SolverConfiguration().with_max_inner_iterations(0)

    def test_solver_config_wrong_constraints_tolerance(self):
        with self.assertRaises(Exception) as __context:
            og.config.SolverConfiguration().with_delta_tolerance(0)

    def test_solver_config_wrong_inner_tolerance(self):
        with self.assertRaises(Exception) as __context:
            og.config.SolverConfiguration().with_tolerance(0)

    def test_solver_config_wrong_lbfgs_memory(self):
        with self.assertRaises(Exception) as __context:
            og.config.SolverConfiguration().with_lbfgs_memory(1)

    def test_solver_config_wrong_max_inner_iterations(self):
        with self.assertRaises(Exception) as __context:
            og.config.SolverConfiguration().with_max_inner_iterations()

    def test_start_multiple_servers(self):
        all_managers = []
        for i in range(10):
            all_managers += [og.tcp.OptimizerTcpManager(
                optimizer_path=RustBuildTestCase.TEST_DIR + '/only_f1',
                ip='0.0.0.0',
                port=15311+i)]

        # Start all servers
        for m in all_managers:
            self.start_tcp_manager(m)

        # Ping all
        for m in all_managers:
            m.ping()

    def test_rust_build_only_f1(self):
        # Start the server using a custom bind IP and port
        mng = self.start_tcp_manager(
            og.tcp.OptimizerTcpManager(RustBuildTestCase.TEST_DIR + '/only_f1',
                                       ip='0.0.0.0',
                                       port=13757))
        pong = mng.ping()  # check if the server is alive
        self.assertEqual(1, pong["Pong"])

        # Regular call
        response = mng.call(p=[2.0, 10.0])
        self.assertEqual("Converged", response["exit_status"])

        # Call with initial params, initial y and initial penalty param
        response = mng.call(p=[2.0, 10.0],
                            initial_guess=response["solution"],
                            initial_y=response["lagrange_multipliers"],
                            initial_penalty=response["penalty"])
        self.assertTrue(response.is_ok())
        status = response.get()
        self.assertEqual(2, status.num_outer_iterations)

        response = mng.call(p=[2.0, 10.0, 50.0])
        status = response.get()
        self.assertFalse(response.is_ok())
        self.assertEqual(True, isinstance(status, og.tcp.SolverError))
        self.assertEqual(3003, status.code)
        self.assertEqual(
            "wrong number of parameters: provided 3, expected 2",
            status.message)

        response = mng.call(p=[2.0, 10.0], initial_guess=[0.1, 0.2])
        self.assertFalse(response.is_ok())
        status = response.get()
        self.assertEqual(True, isinstance(status, og.tcp.SolverError))
        self.assertEqual(1600, status.code)
        self.assertEqual(
            "initial guess has incompatible dimensions: provided 2, expected 5",
            status.message)

        response = mng.call(p=[2.0, 10.0], initial_y=[0.1])
        status = response.get()
        self.assertFalse(response.is_ok())
        self.assertEqual(True, isinstance(status, og.tcp.SolverError))
        self.assertEqual(1700, status.code)
        self.assertEqual(
            "wrong dimension of Langrange multipliers: provided 1, expected 2",
            status.message)

    def test_rust_build_only_f2_preconditioned(self):
        mng1 = self.start_tcp_manager(og.tcp.OptimizerTcpManager(
            RustBuildTestCase.TEST_DIR + '/only_f2'))
        mng2 = self.start_tcp_manager(og.tcp.OptimizerTcpManager(
            RustBuildTestCase.TEST_DIR + '/only_f2_precond'))

        response1 = mng1.call(p=[0.5, 8.5], initial_guess=[
                              1, 2, 3, 4, 0]).get()
        response2 = mng2.call(p=[0.5, 8.5], initial_guess=[
                              1, 2, 3, 4, 0]).get()

        self.assertEqual("Converged", response1.exit_status)
        self.assertEqual("Converged", response2.exit_status)

        # Further testing
        slv_cfg = RustBuildTestCase.solverConfig()
        # check that the solution is (near-) feasible
        self.assertTrue(response1.f2_norm < slv_cfg.constraints_tolerance)
        self.assertTrue(response2.f2_norm < slv_cfg.constraints_tolerance)
        # check the nrom of the FPR
        self.assertTrue(response1.last_problem_norm_fpr <
                        slv_cfg.tolerance)
        self.assertTrue(response2.last_problem_norm_fpr <
                        slv_cfg.tolerance)
        # compare the costs
        self.assertAlmostEqual(response1.cost, response2.cost, 4)

        x1, x2 = response1.solution, response2.solution
        for i in range(len(x1)):
            self.assertAlmostEqual(x1[i], x2[i], delta=5e-4)

        response = mng1.call(p=[2.0, 10.0, 50.0])
        self.assertFalse(response.is_ok())
        status = response.get()
        self.assertEqual(True, isinstance(status, og.tcp.SolverError))
        self.assertEqual(3003, status.code)
        self.assertEqual(
            "wrong number of parameters: provided 3, expected 2",
            status.message)

        response = mng1.call(p=[2.0, 10.0], initial_guess=[0.1, 0.2])
        self.assertFalse(response.is_ok())
        status = response.get()
        self.assertEqual(True, isinstance(status, og.tcp.SolverError))
        self.assertEqual(1600, status.code)
        self.assertEqual(
            "initial guess has incompatible dimensions: provided 2, expected 5",
            status.message)

        response = mng1.call(p=[2.0, 10.0], initial_y=[0.1])
        self.assertFalse(response.is_ok())
        status = response.get()
        self.assertEqual(True, isinstance(status, og.tcp.SolverError))
        self.assertEqual(1700, status.code)
        self.assertEqual(
            "wrong dimension of Langrange multipliers: provided 1, expected 0",
            status.message)

    def test_rust_build_plain(self):
        mng = self.start_tcp_manager(
            og.tcp.OptimizerTcpManager(RustBuildTestCase.TEST_DIR + '/plain'))
        pong = mng.ping()  # check if the server is alive
        self.assertEqual(1, pong["Pong"])

        # Regular call
        response = mng.call(p=[2.0, 10.0])
        self.assertTrue(response.is_ok())
        status = response.get()
        self.assertEqual("Converged", status.exit_status)

    def test_rust_build_plain_invalid_request_details(self):
        self.start_tcp_manager(og.tcp.OptimizerTcpManager(
            RustBuildTestCase.TEST_DIR + '/plain',
            ip='127.0.0.1',
            port=13758))

        malformed_response = og.tcp.SolverResponse(
            RustBuildTestCase.raw_tcp_request('127.0.0.1', 13758, '{"Run":'))
        self.assertFalse(malformed_response.is_ok())
        malformed_status = malformed_response.get()
        self.assertEqual(1000, malformed_status.code)
        self.assertTrue(
            malformed_status.message.startswith("invalid request:"))
        self.assertIn("line 1 column", malformed_status.message)

        utf8_response = og.tcp.SolverResponse(
            RustBuildTestCase.raw_tcp_request('127.0.0.1', 13758, b'\xff\xfe'))
        self.assertFalse(utf8_response.is_ok())
        utf8_status = utf8_response.get()
        self.assertEqual(1000, utf8_status.code)
        self.assertTrue(
            utf8_status.message.startswith(
                "invalid request: request body is not valid UTF-8"))

    def test_rust_build_plain_survives_invalid_request(self):
        mng = self.start_tcp_manager(og.tcp.OptimizerTcpManager(
            RustBuildTestCase.TEST_DIR + '/plain',
            ip='127.0.0.1',
            port=13759))

        malformed_response = og.tcp.SolverResponse(
            RustBuildTestCase.raw_tcp_request('127.0.0.1', 13759, '{"Run":'))
        self.assertFalse(malformed_response.is_ok())
        malformed_status = malformed_response.get()
        self.assertEqual(1000, malformed_status.code)

        pong = mng.ping()
        self.assertEqual(1, pong["Pong"])

        response = mng.call(p=[2.0, 10.0])
        self.assertTrue(response.is_ok())
        self.assertEqual("Converged", response.get().exit_status)

    def test_rust_build_plain_survives_disconnect_mid_request(self):
        mng = self.start_tcp_manager(og.tcp.OptimizerTcpManager(
            RustBuildTestCase.TEST_DIR + '/plain',
            ip='127.0.0.1',
            port=13760))

        RustBuildTestCase.send_partial_tcp_payload_and_close(
            '127.0.0.1',
            13760,
            '{"Run":{"parameter":[2.0'
        )

        pong = mng.ping()
        self.assertEqual(1, pong["Pong"])

        response = mng.call(p=[2.0, 10.0])
        self.assertTrue(response.is_ok())
        self.assertEqual("Converged", response.get().exit_status)

    def test_rust_build_solver_error_details(self):
        mng = self.start_tcp_manager(og.tcp.OptimizerTcpManager(
            RustBuildTestCase.TEST_DIR + '/solver_error'))

        response = mng.call(p=[-1.0])
        self.assertFalse(response.is_ok())
        status = response.get()
        self.assertEqual(2000, status.code)
        self.assertEqual(
            "problem solution failed: cost or gradient evaluation failed: forced solver error for TCP test",
            status.message)

    def test_rust_build_parametric_f2(self):
        # introduced to tackle issue #123
        mng = self.start_tcp_manager(og.tcp.OptimizerTcpManager(
            RustBuildTestCase.TEST_DIR + '/parametric_f2'))
        pong = mng.ping()  # check if the server is alive
        self.assertEqual(1, pong["Pong"])

        # Regular call
        response = mng.call(p=[1.0, 1.0, 0.5])
        self.assertTrue(response.is_ok())
        status = response.get()
        self.assertEqual("Converged", status.exit_status)
        self.assertTrue(status.f2_norm < 1e-4)

    def test_rust_build_parametric_halfspace(self):
        mng = self.start_tcp_manager(og.tcp.OptimizerTcpManager(
            RustBuildTestCase.TEST_DIR + '/halfspace_optimizer'))
        pong = mng.ping()  # check if the server is alive
        self.assertEqual(1, pong["Pong"])

        # Regular call
        response = mng.call(p=[1.0, 1.0])
        self.assertTrue(response.is_ok())
        status = response.get()
        self.assertEqual("Converged", status.exit_status)
        u = status.solution
        c = [1., 2., 1., 5., 2.]
        b = -10.39
        eps = 1e-14
        self.assertTrue(sum([u[i] * c[i] for i in range(5)]) - b <= eps)
        self.assertTrue(-sum([u[i] * c[i] for i in range(5)]) + b <= eps)

    @staticmethod
    def rebuild_generated_staticlib(optimizer_name):
        optimizer_dir = os.path.join(RustBuildTestCase.TEST_DIR, optimizer_name)
        process = subprocess.Popen(
            ["cargo", "build"],
            cwd=optimizer_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        _stdout, stderr = process.communicate()
        return process.returncode, stderr.decode()

    @staticmethod
    def c_bindings_helper(optimizer_name):
        if sys.platform == "win32":
            result = RustBuildTestCase.c_bindings_cmake_helper(
                optimizer_name=optimizer_name,
                build_dir_name="cmake-build-run")
            compile_stdout = result["configure_stdout"] + result["build_stdout"]
            compile_stderr = result["configure_stderr"] + result["build_stderr"]
            compile_returncode = (
                result["configure_returncode"]
                if result["configure_returncode"] != 0
                else result["build_returncode"]
            )

            run_stdout = ""
            run_stderr = ""
            run_returncode = None
            if compile_returncode == 0:
                executable_candidates = [
                    os.path.join(result["build_dir"], "Debug", "optimizer.exe"),
                    os.path.join(result["build_dir"], "optimizer.exe"),
                    os.path.join(result["build_dir"], "Debug", "optimizer"),
                    os.path.join(result["build_dir"], "optimizer"),
                ]
                executable_path = next(
                    (candidate for candidate in executable_candidates if os.path.exists(candidate)),
                    None,
                )
                if executable_path is None:
                    raise RuntimeError("Could not locate built optimizer executable")

                run_process = subprocess.Popen(
                    [executable_path],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                )
                run_stdout_bytes, run_stderr_bytes = run_process.communicate()
                run_stdout = run_stdout_bytes.decode()
                run_stderr = run_stderr_bytes.decode()
                run_returncode = run_process.returncode

            return {
                "compile_returncode": compile_returncode,
                "compile_stdout": compile_stdout,
                "compile_stderr": compile_stderr,
                "run_returncode": run_returncode,
                "run_stdout": run_stdout,
                "run_stderr": run_stderr,
            }

        compile_process = subprocess.Popen(
            ["/usr/bin/gcc",
             RustBuildTestCase.TEST_DIR + "/" + optimizer_name + "/example_optimizer.c",
             "-I" + RustBuildTestCase.TEST_DIR + "/" + optimizer_name,
             "-pthread",
             RustBuildTestCase.TEST_DIR + "/" + optimizer_name +
             "/target/debug/lib" + optimizer_name + ".a",
             "-lm",
             "-ldl",
             "-std=c99",
             "-o",
             RustBuildTestCase.TEST_DIR + "/" + optimizer_name + "/optimizer"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE)

        compile_stdout, compile_stderr = compile_process.communicate()

        run_stdout = b""
        run_stderr = b""
        run_returncode = None
        if compile_process.returncode == 0:
            run_process = subprocess.Popen(
                [RustBuildTestCase.TEST_DIR + "/" + optimizer_name + "/optimizer"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE)
            run_stdout, run_stderr = run_process.communicate()
            run_returncode = run_process.returncode

        return {
            "compile_returncode": compile_process.returncode,
            "compile_stdout": compile_stdout.decode(),
            "compile_stderr": compile_stderr.decode(),
            "run_returncode": run_returncode,
            "run_stdout": run_stdout.decode(),
            "run_stderr": run_stderr.decode(),
        }

    @staticmethod
    def patch_c_bindings_example_parameter_initializer(optimizer_name, replacement_line):
        example_file = os.path.join(
            RustBuildTestCase.TEST_DIR, optimizer_name, "example_optimizer.c")
        with open(example_file, "r", encoding="utf-8") as fh:
            example_source = fh.read()

        original_line = None
        for line in example_source.splitlines():
            if "double p[" in line and "= {" in line:
                original_line = line
                break

        if original_line is None:
            raise RuntimeError("Could not locate parameter initializer in example_optimizer.c")

        with open(example_file, "w", encoding="utf-8") as fh:
            fh.write(example_source.replace(original_line, replacement_line, 1))

        return original_line

    @staticmethod
    def c_bindings_cmake_helper(optimizer_name, build_dir_name="cmake-build-test"):
        cmake_executable = shutil.which("cmake")
        if cmake_executable is None:
            raise unittest.SkipTest("cmake is not available in PATH")

        optimizer_dir = os.path.join(RustBuildTestCase.TEST_DIR, optimizer_name)
        build_dir = os.path.join(optimizer_dir, build_dir_name)
        if os.path.isdir(build_dir):
            shutil.rmtree(build_dir)
        os.makedirs(build_dir)

        configure_process = subprocess.Popen(
            [cmake_executable, ".."],
            cwd=build_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        configure_stdout, configure_stderr = configure_process.communicate()

        build_stdout = b""
        build_stderr = b""
        build_returncode = None
        if configure_process.returncode == 0:
            build_command = [cmake_executable, "--build", "."]
            if sys.platform == "win32":
                build_command.extend(["--config", "Debug"])
            build_process = subprocess.Popen(
                build_command,
                cwd=build_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            build_stdout, build_stderr = build_process.communicate()
            build_returncode = build_process.returncode

        return {
            "configure_returncode": configure_process.returncode,
            "configure_stdout": configure_stdout.decode(),
            "configure_stderr": configure_stderr.decode(),
            "build_returncode": build_returncode,
            "build_stdout": build_stdout.decode(),
            "build_stderr": build_stderr.decode(),
            "build_dir": build_dir,
        }

    def test_c_bindings(self):
        result = RustBuildTestCase.c_bindings_helper(optimizer_name="only_f1")
        self.assertEqual(
            0,
            result["compile_returncode"],
            msg=result["compile_stdout"] + result["compile_stderr"])
        self.assertEqual(0, result["run_returncode"], msg=result["run_stdout"] + result["run_stderr"])
        self.assertIn("Converged", result["run_stdout"])
        self.assertIn("exit status      : 0", result["run_stdout"])
        self.assertIn("error code       : 0", result["run_stdout"])

        result = RustBuildTestCase.c_bindings_helper(optimizer_name="only_f2")
        self.assertEqual(
            0,
            result["compile_returncode"],
            msg=result["compile_stdout"] + result["compile_stderr"])
        self.assertIn("Converged", result["run_stdout"])
        self.assertEqual(0, result["run_returncode"], msg=result["run_stdout"] + result["run_stderr"])
        self.assertIn("exit status      : 0", result["run_stdout"])
        self.assertIn("error code       : 0", result["run_stdout"])

        result = RustBuildTestCase.c_bindings_helper(optimizer_name="plain")
        self.assertIn("Converged", result["run_stdout"])
        self.assertEqual(
            0,
            result["compile_returncode"],
            msg=result["compile_stdout"] + result["compile_stderr"])
        self.assertEqual(0, result["run_returncode"], msg=result["run_stdout"] + result["run_stderr"])
        self.assertIn("exit status      : 0", result["run_stdout"])
        self.assertIn("error code       : 0", result["run_stdout"])

    def test_c_bindings_error_path(self):
        rebuild_rc, rebuild_stderr = RustBuildTestCase.rebuild_generated_staticlib(
            optimizer_name="solver_error")
        self.assertEqual(0, rebuild_rc, msg=rebuild_stderr)

        original_line = RustBuildTestCase.patch_c_bindings_example_parameter_initializer(
            optimizer_name="solver_error",
            replacement_line="    double p[SOLVER_ERROR_NUM_PARAMETERS] = {-1.0};")
        try:
            result = RustBuildTestCase.c_bindings_helper(optimizer_name="solver_error")
        finally:
            RustBuildTestCase.patch_c_bindings_example_parameter_initializer(
                optimizer_name="solver_error",
                replacement_line=original_line)
        self.assertEqual(
            0,
            result["compile_returncode"],
            msg=result["compile_stdout"] + result["compile_stderr"])
        self.assertNotEqual(0, result["run_returncode"])
        self.assertIn("error code       : 2000", result["run_stdout"])
        self.assertIn("forced solver error for TCP test", result["run_stdout"])
        self.assertIn(
            "Solver returned an error; solution vector is not printed.",
            result["run_stderr"])

    def test_c_bindings_cmake_example_builds(self):
        result = RustBuildTestCase.c_bindings_cmake_helper(optimizer_name="plain")
        self.assertEqual(
            0,
            result["configure_returncode"],
            msg=result["configure_stdout"] + result["configure_stderr"])
        self.assertEqual(
            0,
            result["build_returncode"],
            msg=result["build_stdout"] + result["build_stderr"])

    def test_tcp_generated_server_builds(self):
        tcp_iface_dir = os.path.join(
            RustBuildTestCase.TEST_DIR, "plain", "tcp_iface_plain")
        process = subprocess.Popen(
            ["cargo", "build", "--quiet"],
            cwd=tcp_iface_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        _stdout, stderr = process.communicate()

        self.assertEqual(
            0,
            process.returncode,
            msg=stderr.decode()
        )

    def test_tcp_manager_start_fails_cleanly_when_port_is_in_use(self):
        mng1 = self.start_tcp_manager(og.tcp.OptimizerTcpManager(
            RustBuildTestCase.TEST_DIR + '/plain',
            ip='127.0.0.1',
            port=13761))

        mng2 = og.tcp.OptimizerTcpManager(
            RustBuildTestCase.TEST_DIR + '/only_f1',
            ip='127.0.0.1',
            port=13761)
        with self.assertRaises(Exception) as context:
            mng2.start()

        self.assertIn("Port 13761 not available", str(context.exception))

        pong = mng1.ping()
        self.assertEqual(1, pong["Pong"])

    def test_tcp_manager_remote_cannot_start(self):
        remote_tcp_manager = og.tcp.OptimizerTcpManager(
            ip='10.8.0.1', port=3345)
        with self.assertRaises(Exception) as __context:
            remote_tcp_manager.start()

    def test_tcp_manager_remote_ip_no_port(self):
        with self.assertRaises(Exception) as __context:
            _remote_tcp_manager = og.tcp.OptimizerTcpManager(ip='10.8.0.1')

    def test_tcp_manager_remote_port_no_ip(self):
        with self.assertRaises(Exception) as __context:
            _remote_tcp_manager = og.tcp.OptimizerTcpManager(port=8888)

    def test_set_y(self):
        c = og.constraints.Ball2(radius=1)
        y_calc = og.builder.SetYCalculator(c)
        y = y_calc.obtain()

    def test_squared_norm(self):
        u = np.array([3, 4])
        y = og.functions.norm2_squared(u)
        self.assertAlmostEqual(25., y, places=12)

        u = [3, 4]
        y = og.functions.norm2_squared(u)
        self.assertAlmostEqual(25., y, places=12)

        u = cs.SX.sym("u", 2)
        f = og.functions.norm2_squared(u)
        fun = cs.Function('fun', [u], [f])
        y = fun([3, 4])
        self.assertAlmostEqual(25., y, places=12)

    def test_optimizer_meta_valid_version(self):
        meta = og.config.OptimizerMeta().with_version("1.2.3-alpha.1+build.5")
        self.assertEqual("1.2.3-alpha.1+build.5", meta.version)

    def test_optimizer_meta_invalid_version1(self):
        with self.assertRaises(ValueError) as context:
            og.config.OptimizerMeta().with_version("^1.2")

        self.assertIn("Cargo package version", str(context.exception))
    
    def test_optimizer_meta_invalid_version2(self):
        with self.assertRaises(ValueError) as context:
            og.config.OptimizerMeta().with_version("0.1")

        self.assertIn("Cargo package version", str(context.exception))

    def test_with_build_mode_rejects_invalid_values(self):
        """`with_build_mode` should reject unsupported build modes."""
        build_config = og.config.BuildConfiguration()

        with self.assertRaisesRegex(
            ValueError,
            "build mode must be either 'debug' or 'release'",
        ):
            build_config.with_build_mode("profile")


if __name__ == '__main__':
    logging.getLogger('retry').setLevel(logging.ERROR)
    unittest.main()
