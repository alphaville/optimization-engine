#!/bin/bash
set -euxo pipefail

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
    cd .python_test_build/only_f1/tcp_iface_only_f1
    cargo clippy --all-targets --all-features
    cd ../../only_f2/tcp_iface_only_f2
    cargo clippy --all-targets --all-features
    cd ../../rosenbrock_ros
    cargo clippy --all-targets --all-features
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
