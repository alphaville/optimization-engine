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

    #TODO: Make a function...

    # CLIPPY @only_f1
    # ...............
    # Test auto-generated code (main)
    cd .python_test_build/only_f1
    cargo clippy --all-targets --all-features
    # Test auto-generated TCP interface
    cd ./tcp_iface_only_f1
    cargo clippy --all-targets --all-features
    # Test auto-generated CasADi interface
    cd ../icasadi_only_f1/
    cargo clippy --all-targets --all-features
    
    # CLIPPY @only_f2
    # ...............
    cd ../../only_f2
    cargo clippy --all-targets --all-features
    cd ./tcp_iface_only_f2
    cargo clippy --all-targets --all-features
    cd ../icasadi_only_f2
    cargo clippy --all-targets --all-features

    # CLIPPY @rosenbrock_ros
    # ...............
    cd ../../rosenbrock_ros
    cargo clippy --all-targets --all-features
    cd ./icasadi_rosenbrock_ros
    cargo clippy --all-targets --all-features


    # CLIPPY @halfspace_optimizer
    # ...............
    cd ../../halfspace_optimizer
    cargo clippy --all-targets --all-features
    cd ./icasadi_halfspace_optimizer
    cargo clippy --all-targets --all-features
    cd ../tcp_iface_halfspace_optimizer/
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
