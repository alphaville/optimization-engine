import logging
import os
import shutil
import subprocess
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

    @staticmethod
    def _bash(command, cwd, env=None, timeout=180, check=True):
        """Run a bash command and return the completed process."""
        return subprocess.run(
            ["/bin/bash", "-lc", command],
            cwd=cwd,
            env=env,
            text=True,
            capture_output=True,
            timeout=timeout,
            check=check)

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

        self._bash(
            f"source install/setup.bash >/dev/null 2>&1 || true; "
            f"colcon build --packages-select {self.PACKAGE_NAME}",
            cwd=ros2_dir,
            env=env,
            timeout=600)

        node_process = subprocess.Popen(
            [
                "/bin/bash",
                "-lc",
                f"source install/setup.bash && "
                f"ros2 run {self.PACKAGE_NAME} {self.NODE_NAME}"
            ],
            cwd=ros2_dir,
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT)

        try:
            node_seen = False
            topics_seen = False
            for _ in range(6):
                node_result = self._bash(
                    "source install/setup.bash && "
                    "ros2 node list --no-daemon --spin-time 5",
                    cwd=ros2_dir,
                    env=env,
                    timeout=30,
                    check=False)
                topic_result = self._bash(
                    "source install/setup.bash && "
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
                process_output = ""
                if node_process.poll() is None:
                    node_process.terminate()
                    try:
                        node_process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        node_process.kill()
                        node_process.wait(timeout=10)
                if node_process.stdout is not None:
                    process_output = node_process.stdout.read()
                self.fail(
                    "Generated ROS2 node did not become discoverable.\n"
                    f"ros2 node list output:\n{node_result.stdout}\n"
                    f"ros2 topic list output:\n{topic_result.stdout}\n"
                    f"node process output:\n{process_output}")

            echo_process = subprocess.Popen(
                [
                    "/bin/bash",
                    "-lc",
                    "source install/setup.bash && "
                    "ros2 topic echo /result --once"
                ],
                cwd=ros2_dir,
                env=env,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT)

            try:
                time.sleep(1)
                self._bash(
                    "source install/setup.bash && "
                    "ros2 topic pub --once /parameters "
                    f"{self.PACKAGE_NAME}/msg/OptimizationParameters "
                    "'{parameter: [1.0, 2.0], initial_guess: [0.0, 0.0, 0.0, 0.0, 0.0], initial_y: [], initial_penalty: 15.0}'",
                    cwd=ros2_dir,
                    env=env,
                    timeout=60)
                echo_stdout, _ = echo_process.communicate(timeout=60)
            finally:
                if echo_process.poll() is None:
                    echo_process.terminate()
                    echo_process.wait(timeout=10)

            self.assertIn("solution", echo_stdout)
            self.assertIn("solve_time_ms", echo_stdout)
        finally:
            if node_process.poll() is None:
                node_process.terminate()
                try:
                    node_process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    node_process.kill()
                    node_process.wait(timeout=10)


if __name__ == '__main__':
    logging.getLogger('retry').setLevel(logging.ERROR)
    unittest.main()
