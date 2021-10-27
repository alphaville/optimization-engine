#!/bin/bash
set -euxo pipefail

function run_clippy_test() {
    cd $1
    cargo clippy --all-targets --all-features
    if [ -d "./tcp_iface_$1" ] 
    then
        # Test auto-generated TCP interface
        cd ./tcp_iface_$1
        cargo clippy --all-targets --all-features 
        cd ..
    fi
    if [ -d "./tcp_iface_$1" ] 
    then
        # Test auto-generated CasADi interface
        cd icasadi_$1
        cargo clippy --all-targets --all-features
        cd ..
    fi
    cd ..
}

regular_test() {
    # Run Python tests
    # ------------------------------------

    # --- create virtual environment
    cd open-codegen
    export PYTHONPATH=.

    # --- install virtualenv
    pip install virtualenv

    # --- create virtualenv
    virtualenv -p python3.8 venv

    # --- activate venv
    source venv/bin/activate

    # --- upgrade pip within venv
    pip install --upgrade pip

    # --- install opengen
    pip install .

    # --- uncomment to run main file
    # python main.py

    # --- run the tests
    cd opengen
    export PYTHONPATH=.
    python -W ignore test/test_constraints.py -v
    python -W ignore test/test.py -v


    # Run Clippy for generated optimizers
    # ------------------------------------

    cd .python_test_build
    run_clippy_test "only_f1"
    run_clippy_test "only_f2"
    run_clippy_test "halfspace_optimizer"
    run_clippy_test "parametric_f2"
    run_clippy_test "plain"
    run_clippy_test "python_bindings"
    run_clippy_test "rosenbrock_ros"
    
}

test_docker() {
    cd docker
    docker image build -t alphaville/open .
}

main() {
    if [ $DO_DOCKER -eq 0 ]; then
        echo "Running regular tests"
        regular_test
    else
        echo "Building Docker image"
        test_docker
    fi
}

main
