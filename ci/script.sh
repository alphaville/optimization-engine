#!/bin/bash
set -euxo pipefail

SKIP_RPI_TEST="${SKIP_RPI_TEST:-0}"
TASK="${1:-all-python-tests}"

function run_clippy_test() {
    pushd $1
    cargo clippy --all-targets --all-features
    if [ -d "./tcp_iface_$1" ] 
    then
        # Test auto-generated TCP interface
        pushd ./tcp_iface_$1
        cargo clippy --all-targets --all-features 
        popd
    fi
    if [ -d "./icasadi_$1" ] 
    then
        # Test auto-generated CasADi interface
        pushd icasadi_$1
        cargo clippy --all-targets --all-features
        popd
    fi
    popd
}

setup_python_test_env() {
    cd open-codegen
    export PYTHONPATH=.

    python -m venv venv
    source venv/bin/activate
    python -m pip install --upgrade pip
    python -m pip install .
}

generated_clippy_tests() {
    pushd .python_test_build
    run_clippy_test "only_f1"
    run_clippy_test "only_f2"
    run_clippy_test "halfspace_optimizer"
    run_clippy_test "parametric_f2"
    run_clippy_test "plain"
    run_clippy_test "python_bindings"
    run_clippy_test "rosenbrock_ros"
    popd
}

run_python_core_tests() {
    export PYTHONPATH=.

    python -W ignore test/test_constraints.py -v
    python -W ignore test/test.py -v
    if [ "$SKIP_RPI_TEST" -eq 0 ]; then
        python -W ignore test/test_raspberry_pi.py -v
    fi

    # Run Clippy for generated optimizers
    # ------------------------------------
    generated_clippy_tests
}

run_python_ros2_tests() {
    export PYTHONPATH=.
    set +u
    if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        # setup-ros installs the ROS underlay but does not source it for our shell
        source "/opt/ros/${ROS_DISTRO}/setup.bash"
    elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source "/opt/ros/jazzy/setup.bash"
    else
        set -u
        echo "ROS2 environment setup script not found"
        exit 1
    fi
    set -u

    if ! python -c "import em, lark, catkin_pkg" >/dev/null 2>&1; then
        # ROS2 build helpers run under the active Python interpreter. The test venv
        # already has NumPy from `pip install .`, but we also need the ROS-side
        # Python packages used during interface and package metadata generation.
        # Empy 4 has broken older ROS message generators in the past, so keep it
        # on the 3.x API here.
        python -m pip install "empy<4" lark catkin_pkg
    fi

    command -v ros2 >/dev/null
    command -v colcon >/dev/null
    python -W ignore test/test_ros2.py -v
}

run_python_ocp_tests() {
    export PYTHONPATH=.
    python -W ignore test/test_ocp.py -v
}

all_python_tests() {
    setup_python_test_env
    run_python_core_tests
    run_python_ocp_tests
}

test_docker() {
    cd docker
    docker image build -t alphaville/open .
}

main() {
    if [ $DO_DOCKER -eq 0 ]; then
        case "$TASK" in
            python-tests)
                echo "Running Python tests and generated Clippy tests"
                setup_python_test_env
                run_python_core_tests
                ;;
            ros2-tests)
                echo "Running ROS2 Python tests"
                setup_python_test_env
                run_python_ros2_tests
                ;;
            ocp-tests)
                echo "Running OCP Python tests"
                setup_python_test_env
                run_python_ocp_tests
                ;;
            all-python-tests)
                echo "Running Python tests, generated Clippy tests, and OCP tests"
                all_python_tests
                ;;
            *)
                echo "Unknown task: $TASK"
                exit 1
                ;;
        esac
    else
        echo "Building Docker image"
        test_docker
    fi
}

main
