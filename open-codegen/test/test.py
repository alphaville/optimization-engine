import os
import unittest
import casadi.casadi as cs
import opengen as og
import subprocess
import logging


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
            .with_delta_tolerance(1e-5)
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
    def setUpClass(cls):
        cls.setUpPythonBindings()
        cls.setUpRosPackageGeneration()
        cls.setUpOnlyF1()
        cls.setUpOnlyF2()
        cls.setUpOnlyF2(is_preconditioned=True)
        cls.setUpPlain()
        cls.setUpOnlyParametricF2()
        cls.setUpHalfspace()

    def test_python_bindings(self):
        import sys
        import os

        # include the target directory into the path...
        sys.path.insert(1, os.path.join(
            RustBuildTestCase.TEST_DIR, "python_bindings"))
        import python_bindings  # import python_bindings.so

        solver = python_bindings.solver()
        # returns object of type OptimizerSolution
        result = solver.run([1., 2.])
        self.assertIsNotNone(result.solution)

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
            m.start()

        # Ping all
        for m in all_managers:
            m.ping()

        # Kill all
        for m in all_managers:
            m.kill()

    def test_rust_build_only_f1(self):
        # Start the server using a custom bind IP and port
        mng = og.tcp.OptimizerTcpManager(RustBuildTestCase.TEST_DIR + '/only_f1',
                                         ip='0.0.0.0',
                                         port=13757)
        mng.start()
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

        response = mng.call(p=[2.0, 10.0], initial_guess=[0.1, 0.2])
        self.assertFalse(response.is_ok())
        status = response.get()
        self.assertEqual(True, isinstance(status, og.tcp.SolverError))
        self.assertEqual(1600, status.code)

        response = mng.call(p=[2.0, 10.0], initial_y=[0.1])
        status = response.get()
        self.assertFalse(response.is_ok())
        self.assertEqual(True, isinstance(status, og.tcp.SolverError))
        self.assertEqual(1700, status.code)

        mng.kill()

    def test_rust_build_only_f2_preconditioned(self):
        mng1 = og.tcp.OptimizerTcpManager(
            RustBuildTestCase.TEST_DIR + '/only_f2')
        mng2 = og.tcp.OptimizerTcpManager(
            RustBuildTestCase.TEST_DIR + '/only_f2_precond')
        mng1.start()
        mng2.start()

        try:
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

            response = mng1.call(p=[2.0, 10.0], initial_guess=[0.1, 0.2])
            self.assertFalse(response.is_ok())
            status = response.get()
            self.assertEqual(True, isinstance(status, og.tcp.SolverError))
            self.assertEqual(1600, status.code)

            response = mng1.call(p=[2.0, 10.0], initial_y=[0.1])
            self.assertFalse(response.is_ok())
            status = response.get()
            self.assertEqual(True, isinstance(status, og.tcp.SolverError))
            self.assertEqual(1700, status.code)
        finally:
            mng1.kill()
            mng2.kill()

    def test_rust_build_plain(self):
        mng = og.tcp.OptimizerTcpManager(RustBuildTestCase.TEST_DIR + '/plain')
        mng.start()
        pong = mng.ping()  # check if the server is alive
        self.assertEqual(1, pong["Pong"])

        # Regular call
        response = mng.call(p=[2.0, 10.0])
        self.assertTrue(response.is_ok())
        status = response.get()
        self.assertEqual("Converged", status.exit_status)

        mng.kill()

    def test_rust_build_parametric_f2(self):
        # introduced to tackle issue #123
        mng = og.tcp.OptimizerTcpManager(
            RustBuildTestCase.TEST_DIR + '/parametric_f2')
        mng.start()
        pong = mng.ping()  # check if the server is alive
        self.assertEqual(1, pong["Pong"])

        # Regular call
        response = mng.call(p=[1.0, 1.0, 0.5])
        self.assertTrue(response.is_ok())
        status = response.get()
        self.assertEqual("Converged", status.exit_status)
        self.assertTrue(status.f2_norm < 1e-4)
        mng.kill()

    def test_rust_build_parametric_halfspace(self):
        mng = og.tcp.OptimizerTcpManager(
            RustBuildTestCase.TEST_DIR + '/halfspace_optimizer')
        mng.start()
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

        mng.kill()

    @staticmethod
    def c_bindings_helper(optimizer_name):
        p = subprocess.Popen(["/usr/bin/gcc",
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

        # Make sure it compiles
        p.communicate()
        rc1 = p.returncode

        # Run the optimizer
        p = subprocess.Popen([RustBuildTestCase.TEST_DIR + "/" + optimizer_name + "/optimizer"],
                             stdout=subprocess.DEVNULL)
        p.communicate()
        rc2 = p.returncode

        return rc1, rc2

    def test_c_bindings(self):
        rc1, rc2 = RustBuildTestCase.c_bindings_helper(
            optimizer_name="only_f1")
        self.assertEqual(0, rc1)
        self.assertEqual(0, rc2)

        rc1, rc2 = RustBuildTestCase.c_bindings_helper(
            optimizer_name="only_f2")
        self.assertEqual(0, rc1)
        self.assertEqual(0, rc2)

        rc1, rc2 = RustBuildTestCase.c_bindings_helper(optimizer_name="plain")
        self.assertEqual(0, rc1)
        self.assertEqual(0, rc2)

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


if __name__ == '__main__':
    logging.getLogger('retry').setLevel(logging.ERROR)
    unittest.main()
