#!/bin/bash
set -euxo pipefail

SKIP_RPI_TEST="${SKIP_RPI_TEST:-0}"

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

generated_clippy_tests() {
    cd .python_test_build
    run_clippy_test "only_f1"
    run_clippy_test "only_f2"
    run_clippy_test "halfspace_optimizer"
    run_clippy_test "parametric_f2"
    run_clippy_test "plain"
    run_clippy_test "python_bindings"
    run_clippy_test "rosenbrock_ros"
}

python_tests() {
    # Run Python tests
    # ------------------------------------

    cd open-codegen
    export PYTHONPATH=.

    # --- create virtualenv
    python -m venv venv

    # --- activate venv
    source venv/bin/activate

    # --- upgrade pip within venv
    python -m pip install --upgrade pip

    # --- install opengen
    python -m pip install .

    # --- run the tests
    export PYTHONPATH=.
    python -W ignore test/test_constraints.py -v
    python -W ignore test/test.py -v
    python -W ignore test/test_ocp.py -v
    if [ "$SKIP_RPI_TEST" -eq 0 ]; then
        python -W ignore test/test_raspberry_pi.py -v
    fi

    # Run Clippy for generated optimizers
    # ------------------------------------
    generated_clippy_tests
}

test_docker() {
    cd docker
    docker image build -t alphaville/open .
}

main() {
    if [ $DO_DOCKER -eq 0 ]; then
        echo "Running Python and generated Clippy tests"
        python_tests
    else
        echo "Building Docker image"
        test_docker
    fi
}

main
