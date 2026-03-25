import logging
import os
import shlex
import signal
import shutil
import subprocess
import sys
import time
import unittest

import casadi.casadi as cs
import opengen as og


class Ros2BuildTestCase(unittest.TestCase):
    """Integration tests for auto-generated ROS2 packages."""

    TEST_DIR = ".python_test_build"
    OPTIMIZER_NAME = "rosenbrock_ros2"
    PACKAGE_NAME = "parametric_optimizer_ros2"
    NODE_NAME = "open_node_ros2"

    @staticmethod
    def get_open_local_absolute_path():
        """Return the absolute path to the local OpEn repository root."""
        cwd = os.getcwd()
        return cwd.split('open-codegen')[0]

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
    def setUpClass(cls):
        """Generate the ROS2 package once before all tests run."""
        if shutil.which("ros2") is None or shutil.which("colcon") is None:
            raise unittest.SkipTest("ROS2 CLI tools are not available in PATH")
        cls.setUpRos2PackageGeneration()

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
        env["ROS_LOG_DIR"] = os.path.join(ros2_dir, ".ros_log")
        env.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
        env.pop("ROS_LOCALHOST_ONLY", None)
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

    def test_ros2_package_generation(self):
        """Verify the ROS2 package files are generated."""
        ros2_dir = self.ros2_package_dir()
        self.assertTrue(os.path.isfile(os.path.join(ros2_dir, "package.xml")))
        self.assertTrue(os.path.isfile(os.path.join(ros2_dir, "CMakeLists.txt")))
        self.assertTrue(os.path.isfile(
            os.path.join(ros2_dir, "launch", "open_optimizer.launch.py")))

    def test_generated_ros2_package_works(self):
        """Build, run, and call the generated ROS2 package."""
        ros2_dir = self.ros2_package_dir()
        env = self.ros2_test_env()
        shell_path, setup_script = self.ros2_shell()
        python_executable = shlex.quote(sys.executable)

        self._run_shell(
            f"source {setup_script} >/dev/null 2>&1 || true; "
            f"colcon build --packages-select {self.PACKAGE_NAME} "
            f"--cmake-args -DPython3_EXECUTABLE={python_executable}",
            cwd=ros2_dir,
            env=env,
            timeout=600)

        node_process = subprocess.Popen(
            [
                shell_path,
                "-lc",
                f"source {setup_script} && "
                f"ros2 run {self.PACKAGE_NAME} {self.NODE_NAME}"
            ],
            cwd=ros2_dir,
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True)

        try:
            node_seen = False
            topics_seen = False
            for _ in range(6):
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
                    break
                time.sleep(1)

            if not (node_seen and topics_seen):
                process_output = self._terminate_process(node_process)
                self.fail(
                    "Generated ROS2 node did not become discoverable.\n"
                    f"ros2 node list output:\n{node_result.stdout}\n"
                    f"ros2 topic list output:\n{topic_result.stdout}\n"
                    f"node process output:\n{process_output}")

            echo_process = subprocess.Popen(
                [
                    shell_path,
                    "-lc",
                    f"source {setup_script} && "
                    "ros2 topic echo /result --once"
                ],
                cwd=ros2_dir,
                env=env,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                start_new_session=True)

            try:
                time.sleep(1)
                self._run_shell(
                    f"source {setup_script} && "
                    "ros2 topic pub --once /parameters "
                    f"{self.PACKAGE_NAME}/msg/OptimizationParameters "
                    "'{parameter: [1.0, 2.0], initial_guess: [0.0, 0.0, 0.0, 0.0, 0.0], initial_y: [], initial_penalty: 15.0}'",
                    cwd=ros2_dir,
                    env=env,
                    timeout=60)
                echo_stdout, _ = echo_process.communicate(timeout=60)
            finally:
                if echo_process.poll() is None:
                    self._terminate_process(echo_process)

            self.assertIn("solution", echo_stdout)
            self.assertIn("solve_time_ms", echo_stdout)
        finally:
            if node_process.poll() is None:
                self._terminate_process(node_process)


if __name__ == '__main__':
    logging.getLogger('retry').setLevel(logging.ERROR)
    unittest.main()
