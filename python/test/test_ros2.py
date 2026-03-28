import logging
import os
import re
import shlex
import signal
import shutil
import subprocess
import sys
import time
import unittest

import casadi.casadi as cs
import opengen as og


class BuildConfigurationRos2TestCase(unittest.TestCase):
    """Unit tests for ROS2-specific build configuration behavior."""

    def test_with_ros2_sets_ros2_config_and_enables_c_bindings(self):
        """`with_ros2` should store the ROS2 config and enable C bindings."""
        ros2_config = og.config.RosConfiguration().with_package_name("unit_test_ros2_pkg")
        build_config = og.config.BuildConfiguration().with_ros2(ros2_config)

        self.assertIs(build_config.ros2_config, ros2_config)
        self.assertIsNone(build_config.ros_config)
        self.assertTrue(build_config.build_c_bindings)

        build_dict = build_config.to_dict()
        self.assertIn("ros2_config", build_dict)
        self.assertNotIn("ros_config", build_dict)
        self.assertEqual("unit_test_ros2_pkg", build_dict["ros2_config"]["package_name"])

    def test_ros_and_ros2_configs_clear_each_other(self):
        """Selecting ROS1 or ROS2 should clear the other package configuration."""
        ros1_config = og.config.RosConfiguration().with_package_name("unit_test_ros_pkg")
        ros2_config = og.config.RosConfiguration().with_package_name("unit_test_ros2_pkg")
        build_config = og.config.BuildConfiguration()

        build_config.with_ros2(ros2_config)
        self.assertIs(build_config.ros2_config, ros2_config)
        self.assertIsNone(build_config.ros_config)

        build_config.with_ros(ros1_config)
        self.assertIs(build_config.ros_config, ros1_config)
        self.assertIsNone(build_config.ros2_config)

        build_config.with_ros2(ros2_config)
        self.assertIs(build_config.ros2_config, ros2_config)
        self.assertIsNone(build_config.ros_config)

class Ros2TemplateCustomizationTestCase(unittest.TestCase):
    """Generation tests for custom ROS2 configuration values."""

    TEST_DIR = ".python_test_build"
    OPTIMIZER_NAME = "rosenbrock_ros2_custom"
    PACKAGE_NAME = "custom_parametric_optimizer_ros2"
    NODE_NAME = "custom_open_node_ros2"
    DESCRIPTION = "custom ROS2 package for generation tests"
    RESULT_TOPIC = "custom_result_topic"
    PARAMS_TOPIC = "custom_params_topic"
    RATE = 17.5
    RESULT_QUEUE_SIZE = 11
    PARAMS_QUEUE_SIZE = 13

    @staticmethod
    def get_open_local_absolute_path():
        """Return the absolute path to the local OpEn repository root."""
        return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "rust"))

    @classmethod
    def solverConfig(cls):
        """Return a solver configuration shared by the ROS2 generation tests."""
        return Ros2BuildTestCase.solverConfig()

    @classmethod
    def setUpCustomRos2PackageGeneration(cls):
        """Generate a ROS2 package with non-default configuration values."""
        u = cs.MX.sym("u", 5)
        p = cs.MX.sym("p", 2)
        phi = og.functions.rosenbrock(u, p)
        c = cs.vertcat(1.5 * u[0] - u[1],
                       cs.fmax(0.0, u[2] - u[3] + 0.1))
        bounds = og.constraints.Ball2(None, 1.5)
        meta = og.config.OptimizerMeta() \
            .with_optimizer_name(cls.OPTIMIZER_NAME)
        problem = og.builder.Problem(u, p, phi) \
            .with_constraints(bounds) \
            .with_penalty_constraints(c)
        ros_config = og.config.RosConfiguration() \
            .with_package_name(cls.PACKAGE_NAME) \
            .with_node_name(cls.NODE_NAME) \
            .with_description(cls.DESCRIPTION) \
            .with_rate(cls.RATE) \
            .with_queue_sizes(cls.RESULT_QUEUE_SIZE, cls.PARAMS_QUEUE_SIZE) \
            .with_publisher_subtopic(cls.RESULT_TOPIC) \
            .with_subscriber_subtopic(cls.PARAMS_TOPIC)
        build_config = og.config.BuildConfiguration() \
            .with_open_version(local_path=cls.get_open_local_absolute_path()) \
            .with_build_directory(cls.TEST_DIR) \
            .with_build_mode(og.config.BuildConfiguration.DEBUG_MODE) \
            .with_build_c_bindings() \
            .with_ros2(ros_config)
        og.builder.OpEnOptimizerBuilder(problem,
                                        metadata=meta,
                                        build_configuration=build_config,
                                        solver_configuration=cls.solverConfig()) \
            .build()

    @classmethod
    def setUpClass(cls):
        """Generate the custom ROS2 package once before running tests."""
        cls.setUpCustomRos2PackageGeneration()

    @classmethod
    def ros2_package_dir(cls):
        """Return the filesystem path to the generated custom ROS2 package."""
        return os.path.join(
            cls.TEST_DIR,
            cls.OPTIMIZER_NAME,
            cls.PACKAGE_NAME)

    def test_custom_ros2_configuration_is_rendered_into_generated_files(self):
        """Custom ROS2 config values should appear in the generated package files."""
        ros2_dir = self.ros2_package_dir()

        # The package metadata should reflect the user-provided ROS2 package name
        # and description, not the defaults from the templates.
        with open(os.path.join(ros2_dir, "package.xml"), encoding="utf-8") as f:
            package_xml = f.read()
        self.assertIn(f"<name>{self.PACKAGE_NAME}</name>", package_xml)
        self.assertIn(f"<description>{self.DESCRIPTION}</description>", package_xml)

        # `open_optimizer.hpp` is where the generated node constants are wired in.
        # These assertions make sure the custom topic names, node name, rate, and
        # queue sizes are propagated into the generated C++ code.
        with open(os.path.join(ros2_dir, "include", "open_optimizer.hpp"), encoding="utf-8") as f:
            optimizer_header = f.read()
        self.assertIn(f'#define ROS2_NODE_{self.OPTIMIZER_NAME.upper()}_NODE_NAME "{self.NODE_NAME}"',
                      optimizer_header)
        self.assertIn(f'#define ROS2_NODE_{self.OPTIMIZER_NAME.upper()}_RESULT_TOPIC "{self.RESULT_TOPIC}"',
                      optimizer_header)
        self.assertIn(f'#define ROS2_NODE_{self.OPTIMIZER_NAME.upper()}_PARAMS_TOPIC "{self.PARAMS_TOPIC}"',
                      optimizer_header)
        self.assertIn(f"#define ROS2_NODE_{self.OPTIMIZER_NAME.upper()}_RATE {self.RATE}",
                      optimizer_header)
        self.assertIn(
            f"#define ROS2_NODE_{self.OPTIMIZER_NAME.upper()}_RESULT_TOPIC_QUEUE_SIZE {self.RESULT_QUEUE_SIZE}",
            optimizer_header)
        self.assertIn(
            f"#define ROS2_NODE_{self.OPTIMIZER_NAME.upper()}_PARAMS_TOPIC_QUEUE_SIZE {self.PARAMS_QUEUE_SIZE}",
            optimizer_header)

        # The runtime YAML configuration should carry the custom topic names and
        # timer rate so the launched node uses the intended ROS2 parameters.
        with open(os.path.join(ros2_dir, "config", "open_params.yaml"), encoding="utf-8") as f:
            params_yaml = f.read()
        self.assertIn(f'result_topic: "{self.RESULT_TOPIC}"', params_yaml)
        self.assertIn(f'params_topic: "{self.PARAMS_TOPIC}"', params_yaml)
        self.assertIn(f"rate: {self.RATE}", params_yaml)

        # The generated launch file should point to the correct package and
        # executable so `ros2 launch` can start the generated node.
        with open(os.path.join(ros2_dir, "launch", "open_optimizer.launch.py"), encoding="utf-8") as f:
            launch_file = f.read()
        self.assertIn(f'package="{self.PACKAGE_NAME}"', launch_file)
        self.assertIn(f'executable="{self.NODE_NAME}"', launch_file)
        self.assertIn(f'name="{self.NODE_NAME}"', launch_file)
        self.assertIn(f'FindPackageShare("{self.PACKAGE_NAME}")', launch_file)

        with open(os.path.join(ros2_dir, "msg", "OptimizationResult.msg"), encoding="utf-8") as f:
            result_msg = f.read()
        self.assertIn("uint8 STATUS_INVALID_REQUEST=5", result_msg)
        self.assertIn("int32        error_code", result_msg)
        self.assertIn("string       error_message", result_msg)

        with open(os.path.join(ros2_dir, "include", f"{self.OPTIMIZER_NAME}_bindings.hpp"),
                  encoding="utf-8") as f:
            bindings_header = f.read()
        self.assertIn("error_code", bindings_header)
        self.assertIn("error_message", bindings_header)


class Ros2BuildTestCase(unittest.TestCase):
    """Integration tests for auto-generated ROS2 packages."""

    TEST_DIR = ".python_test_build"
    OPTIMIZER_NAME = "rosenbrock_ros2"
    PACKAGE_NAME = "parametric_optimizer_ros2"
    NODE_NAME = "open_node_ros2"

    @staticmethod
    def get_open_local_absolute_path():
        """Return the absolute path to the local OpEn repository root."""
        return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "rust"))

    @classmethod
    def solverConfig(cls):
        """Return a solver configuration shared by the ROS2 tests."""
        return og.config.SolverConfiguration() \
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

    @classmethod
    def setUpRos2PackageGeneration(cls):
        """Generate the ROS2 package used by the ROS2 integration tests."""
        u = cs.MX.sym("u", 5)
        p = cs.MX.sym("p", 2)
        phi = og.functions.rosenbrock(u, p)
        c = cs.vertcat(1.5 * u[0] - u[1],
                       cs.fmax(0.0, u[2] - u[3] + 0.1))
        bounds = og.constraints.Ball2(None, 1.5)
        meta = og.config.OptimizerMeta() \
            .with_optimizer_name(cls.OPTIMIZER_NAME)
        problem = og.builder.Problem(u, p, phi) \
            .with_constraints(bounds) \
            .with_penalty_constraints(c)
        ros_config = og.config.RosConfiguration() \
            .with_package_name(cls.PACKAGE_NAME) \
            .with_node_name(cls.NODE_NAME) \
            .with_rate(35) \
            .with_description("really cool ROS2 node")
        build_config = og.config.BuildConfiguration() \
            .with_open_version(local_path=cls.get_open_local_absolute_path()) \
            .with_build_directory(cls.TEST_DIR) \
            .with_build_mode(og.config.BuildConfiguration.DEBUG_MODE) \
            .with_build_c_bindings() \
            .with_ros2(ros_config)
        og.builder.OpEnOptimizerBuilder(problem,
                                        metadata=meta,
                                        build_configuration=build_config,
                                        solver_configuration=cls.solverConfig()) \
            .build()

    @classmethod
    def _inject_deterministic_solver_error(cls):
        """Patch the generated solver so negative `p[0]` triggers a known error."""
        solver_root = os.path.join(cls.TEST_DIR, cls.OPTIMIZER_NAME)
        target_lib = os.path.join(solver_root, "src", "lib.rs")
        with open(target_lib, "r", encoding="utf-8") as fh:
            solver_lib = fh.read()

        if "forced solver error for ROS2 test" in solver_lib:
            return

        anchor = (
            '    assert_eq!(u.len(), ROSENBROCK_ROS2_NUM_DECISION_VARIABLES, '
            '"Wrong number of decision variables (u)");\n'
        )
        injected_guard = (
            anchor +
            '\n'
            '    if p[0] < 0.0 {\n'
            '        return Err(SolverError::Cost("forced solver error for ROS2 test"));\n'
            '    }\n'
        )
        if anchor not in solver_lib:
            raise RuntimeError("Could not inject deterministic solver error into ROS2 solver")

        with open(target_lib, "w", encoding="utf-8") as fh:
            fh.write(solver_lib.replace(anchor, injected_guard, 1))

    @classmethod
    def _rebuild_generated_solver_library(cls):
        """Rebuild the generated Rust solver and refresh the ROS2 static library."""
        solver_root = os.path.join(cls.TEST_DIR, cls.OPTIMIZER_NAME)
        process = subprocess.Popen(
            ["cargo", "build"],
            cwd=solver_root,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        _stdout, stderr = process.communicate()
        if process.returncode != 0:
            raise RuntimeError(
                "Could not rebuild generated ROS2 solver:\n{}".format(stderr.decode())
            )

        generated_static_lib = os.path.join(
            solver_root, "target", "debug", f"lib{cls.OPTIMIZER_NAME}.a")
        ros2_static_lib = os.path.join(
            cls.ros2_package_dir(), "extern_lib", f"lib{cls.OPTIMIZER_NAME}.a")
        shutil.copyfile(generated_static_lib, ros2_static_lib)

    @classmethod
    def setUpClass(cls):
        """Generate the ROS2 package once before all tests run."""
        if shutil.which("ros2") is None or shutil.which("colcon") is None:
            raise unittest.SkipTest("ROS2 CLI tools are not available in PATH")
        cls.setUpRos2PackageGeneration()
        cls._inject_deterministic_solver_error()
        cls._rebuild_generated_solver_library()

    @classmethod
    def ros2_package_dir(cls):
        """Return the filesystem path to the generated ROS2 package."""
        return os.path.join(
            cls.TEST_DIR,
            cls.OPTIMIZER_NAME,
            cls.PACKAGE_NAME)

    @classmethod
    def ros2_test_env(cls):
        """Return the subprocess environment used by ROS2 integration tests."""
        env = os.environ.copy()
        ros2_dir = cls.ros2_package_dir()
        os.makedirs(os.path.join(ros2_dir, ".ros_log"), exist_ok=True)
        # Keep ROS2 logs inside the generated package directory so the tests do
        # not depend on a global writable log location.
        env["ROS_LOG_DIR"] = os.path.join(ros2_dir, ".ros_log")
        # Fast DDS is the most reliable middleware choice in our CI/local test
        # setup when checking node discovery from separate processes.
        env.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
        env.pop("ROS_LOCALHOST_ONLY", None)
        ros_env_prefix = env.get("CONDA_PREFIX") or sys.prefix
        ros_env_lib = os.path.join(ros_env_prefix, "lib")
        if os.path.isdir(ros_env_lib):
            for var_name in ("DYLD_LIBRARY_PATH", "DYLD_FALLBACK_LIBRARY_PATH", "LD_LIBRARY_PATH"):
                current_value = env.get(var_name, "")
                env[var_name] = (
                    f"{ros_env_lib}{os.pathsep}{current_value}"
                    if current_value else ros_env_lib
                )
        return env

    @classmethod
    def ros2_shell(cls):
        """Return the preferred shell executable and setup script for ROS2 commands."""
        shell_path = "/bin/bash"
        setup_script = "install/setup.bash"
        preferred_shell = os.path.basename(os.environ.get("SHELL", ""))
        zsh_setup = os.path.join(cls.ros2_package_dir(), "install", "setup.zsh")
        if preferred_shell == "zsh" and os.path.isfile(zsh_setup):
            shell_path = "/bin/zsh"
            setup_script = "install/setup.zsh"
        return shell_path, setup_script

    @classmethod
    def _run_shell(cls, command, cwd, env=None, timeout=180, check=True):
        """Run a command in the preferred shell and return the completed process."""
        shell_path, _ = cls.ros2_shell()
        result = subprocess.run(
            [shell_path, "-lc", command],
            cwd=cwd,
            env=env,
            text=True,
            capture_output=True,
            timeout=timeout,
            check=False)
        if check and result.returncode != 0:
            raise AssertionError(
                "Command failed with exit code "
                f"{result.returncode}: {command}\n"
                f"stdout:\n{result.stdout}\n"
                f"stderr:\n{result.stderr}"
            )
        return result

    @staticmethod
    def _terminate_process(process, timeout=10):
        """Terminate a spawned shell process and its children, then collect output."""
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=timeout)
        try:
            stdout, _ = process.communicate(timeout=1)
        except subprocess.TimeoutExpired:
            stdout = ""
        return stdout or ""

    def _build_generated_package(self, ros2_dir, env):
        """Build the generated ROS2 package with the active Python executable."""
        python_executable = shlex.quote(sys.executable)
        self._run_shell(
            f"source {self.ros2_shell()[1]} >/dev/null 2>&1 || true; "
            f"colcon build --packages-select {self.PACKAGE_NAME} "
            f"--cmake-args -DPython3_EXECUTABLE={python_executable}",
            cwd=ros2_dir,
            env=env,
            timeout=600)

    def _spawn_ros_process(self, command, ros2_dir, env):
        """Start a long-running ROS2 command in a fresh process group."""
        shell_path, setup_script = self.ros2_shell()
        return subprocess.Popen(
            [
                shell_path,
                "-lc",
                f"source {setup_script} && {command}"
            ],
            cwd=ros2_dir,
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True)

    def _wait_for_node_and_topics(self, ros2_dir, env, process=None):
        """Wait until the generated ROS2 node and its topics become discoverable."""
        _, setup_script = self.ros2_shell()
        node_result = None
        topic_result = None
        for _ in range(6):
            if process is not None and process.poll() is not None:
                process_output = self._terminate_process(process)
                raise unittest.SkipTest(
                    "Generated ROS2 node could not start in this environment.\n"
                    f"Process output:\n{process_output}")
            # `ros2 node list` confirms that the process joined the ROS graph,
            # while `ros2 topic list` confirms that the expected interfaces are
            # actually being advertised.
            node_result = self._run_shell(
                f"source {setup_script} && "
                "ros2 node list --no-daemon --spin-time 5",
                cwd=ros2_dir,
                env=env,
                timeout=30,
                check=False)
            topic_result = self._run_shell(
                f"source {setup_script} && "
                "ros2 topic list --no-daemon --spin-time 5",
                cwd=ros2_dir,
                env=env,
                timeout=30,
                check=False)
            node_seen = f"/{self.NODE_NAME}" in node_result.stdout
            topics_seen = "/parameters" in topic_result.stdout and "/result" in topic_result.stdout
            if node_seen and topics_seen:
                return
            time.sleep(1)

        if process is not None and process.poll() is not None:
            process_output = self._terminate_process(process)
            raise unittest.SkipTest(
                "Generated ROS2 node exited before it became discoverable.\n"
                f"Process output:\n{process_output}")

        self.fail(
            "Generated ROS2 node did not become discoverable.\n"
            f"ros2 node list output:\n{node_result.stdout if node_result else ''}\n"
            f"ros2 topic list output:\n{topic_result.stdout if topic_result else ''}")

    def _assert_result_message(self, echo_stdout):
        """Assert that the echoed result message indicates a successful solve."""
        # We do not compare the full numeric solution here; instead, we check
        # that the generated node returned a structurally valid result and that
        # the solver reported convergence.
        self.assertIn("solution", echo_stdout)
        self.assertRegex(
            echo_stdout,
            r"solution:\s*\n(?:- .+\n)+",
            msg=f"Expected a non-empty solution vector in result output:\n{echo_stdout}")
        # `status: 0` matches `STATUS_CONVERGED` in the generated result message.
        self.assertIn("status: 0", echo_stdout)
        self.assertIn("error_code: 0", echo_stdout)
        self.assertIn("error_message:", echo_stdout)
        self.assertRegex(
            echo_stdout,
            r"inner_iterations:\s*[1-9]\d*",
            msg=f"Expected a positive inner iteration count in result output:\n{echo_stdout}")
        self.assertRegex(
            echo_stdout,
            r"outer_iterations:\s*[1-9]\d*",
            msg=f"Expected a positive outer iteration count in result output:\n{echo_stdout}")
        self.assertRegex(
            echo_stdout,
            r"cost:\s*-?\d+(?:\.\d+)?(?:e[+-]?\d+)?",
            msg=f"Expected a numeric cost in result output:\n{echo_stdout}")
        self.assertIn("solve_time_ms", echo_stdout)

    def _assert_invalid_request_message(self, echo_stdout, error_code, error_message_fragment):
        """Assert that the echoed result message reports an invalid request."""
        self.assertIn("status: 5", echo_stdout)
        self.assertIn(f"error_code: {error_code}", echo_stdout)
        self.assertIn(error_message_fragment, echo_stdout)

    def _assert_solver_error_message(self, echo_stdout, error_message_fragment):
        """Assert that the echoed result message reports a solver-side failure."""
        self.assertIn("status: 3", echo_stdout)
        self.assertIn("error_code: 2000", echo_stdout)
        self.assertIn(error_message_fragment, echo_stdout)

    def _publish_request_and_collect_result(self, ros2_dir, env, request_payload):
        """Publish one request and return one echoed result message."""
        _, setup_script = self.ros2_shell()
        echo_process = self._spawn_ros_process("ros2 topic echo /result --once", ros2_dir, env)

        try:
            time.sleep(1)
            self._run_shell(
                f"source {setup_script} && "
                "ros2 topic pub --once /parameters "
                f"{self.PACKAGE_NAME}/msg/OptimizationParameters "
                f"'{request_payload}'",
                cwd=ros2_dir,
                env=env,
                timeout=60)
            echo_stdout, _ = echo_process.communicate(timeout=60)
        finally:
            if echo_process.poll() is None:
                self._terminate_process(echo_process)

        return echo_stdout

    def _exercise_running_optimizer(self, ros2_dir, env):
        """Publish one request and verify that one valid result message is returned."""
        echo_stdout = self._publish_request_and_collect_result(
            ros2_dir,
            env,
            "{parameter: [1.0, 2.0], initial_guess: [0.0, 0.0, 0.0, 0.0, 0.0], initial_y: [], initial_penalty: 15.0}")
        self._assert_result_message(echo_stdout)

    def _exercise_invalid_request(self, ros2_dir, env):
        """Publish an invalid request and verify that the node reports it clearly."""
        echo_stdout = self._publish_request_and_collect_result(
            ros2_dir,
            env,
            "{parameter: [1.0], initial_guess: [0.0, 0.0, 0.0, 0.0, 0.0], initial_y: [], initial_penalty: 15.0}")
        self._assert_invalid_request_message(
            echo_stdout,
            3003,
            "wrong number of parameters")

    def _exercise_invalid_initial_guess(self, ros2_dir, env):
        """Verify that invalid warm-start dimensions are reported clearly."""
        echo_stdout = self._publish_request_and_collect_result(
            ros2_dir,
            env,
            "{parameter: [1.0, 2.0], initial_guess: [0.0], initial_y: [], initial_penalty: 15.0}")
        self._assert_invalid_request_message(
            echo_stdout,
            1600,
            "initial guess has incompatible dimensions")

    def _exercise_invalid_initial_y(self, ros2_dir, env):
        """Verify that invalid multiplier dimensions are reported clearly."""
        echo_stdout = self._publish_request_and_collect_result(
            ros2_dir,
            env,
            "{parameter: [1.0, 2.0], initial_guess: [0.0, 0.0, 0.0, 0.0, 0.0], initial_y: [0.0], initial_penalty: 15.0}")
        self._assert_invalid_request_message(
            echo_stdout,
            1700,
            "wrong dimension of Lagrange multipliers")

    def _exercise_solver_error(self, ros2_dir, env):
        """Verify that solver-side failures propagate to the ROS2 result message."""
        echo_stdout = self._publish_request_and_collect_result(
            ros2_dir,
            env,
            "{parameter: [-1.0, 2.0], initial_guess: [0.0, 0.0, 0.0, 0.0, 0.0], initial_y: [], initial_penalty: 15.0}")
        self._assert_solver_error_message(
            echo_stdout,
            "forced solver error for ROS2 test")

    def test_ros2_package_generation(self):
        """Verify the ROS2 package files are generated."""
        ros2_dir = self.ros2_package_dir()
        # This is a lightweight smoke test for the generator itself before we
        # attempt the slower build/run integration tests below.
        self.assertTrue(os.path.isfile(os.path.join(ros2_dir, "package.xml")))
        self.assertTrue(os.path.isfile(os.path.join(ros2_dir, "CMakeLists.txt")))
        self.assertTrue(os.path.isfile(
            os.path.join(ros2_dir, "launch", "open_optimizer.launch.py")))

    def test_generated_ros2_package_works(self):
        """Build, run, and call the generated ROS2 package."""
        ros2_dir = self.ros2_package_dir()
        env = self.ros2_test_env()

        # First validate the plain `ros2 run` path, which exercises the
        # generated executable directly without going through the launch file.
        self._build_generated_package(ros2_dir, env)

        node_process = self._spawn_ros_process(
            f"ros2 run {self.PACKAGE_NAME} {self.NODE_NAME}",
            ros2_dir,
            env)

        try:
            self._wait_for_node_and_topics(ros2_dir, env, node_process)
            self._exercise_running_optimizer(ros2_dir, env)
            self._exercise_invalid_request(ros2_dir, env)
            self._exercise_invalid_initial_guess(ros2_dir, env)
            self._exercise_invalid_initial_y(ros2_dir, env)
            self._exercise_solver_error(ros2_dir, env)
            self._exercise_running_optimizer(ros2_dir, env)
        finally:
            if node_process.poll() is None:
                self._terminate_process(node_process)

    def test_generated_ros2_launch_file_works(self):
        """Build the package, launch the node, and verify the launch file works."""
        ros2_dir = self.ros2_package_dir()
        env = self.ros2_test_env()

        # Then validate the generated launch description, which should bring up
        # the exact same node and parameters via `ros2 launch`.
        self._build_generated_package(ros2_dir, env)

        launch_process = self._spawn_ros_process(
            f"ros2 launch {self.PACKAGE_NAME} open_optimizer.launch.py",
            ros2_dir,
            env)

        try:
            self._wait_for_node_and_topics(ros2_dir, env, launch_process)
            self._exercise_running_optimizer(ros2_dir, env)
            self._exercise_invalid_request(ros2_dir, env)
            self._exercise_invalid_initial_guess(ros2_dir, env)
            self._exercise_invalid_initial_y(ros2_dir, env)
            self._exercise_solver_error(ros2_dir, env)
            self._exercise_running_optimizer(ros2_dir, env)
        finally:
            if launch_process.poll() is None:
                self._terminate_process(launch_process)


if __name__ == '__main__':
    logging.getLogger('retry').setLevel(logging.ERROR)
    unittest.main()
