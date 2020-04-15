#! /bin/bash
set -euxo pipefail

normal_test() {
    # Run Rust tests
    # ------------------------------------
    cargo test


    # Run Python tests
    # ------------------------------------

    # Create virtual environment
    cd open-codegen
    export PYTHONPATH=.
    virtualenv -p python$PYTHON_VERSION venv
    
    # --- activate venv
    source venv/bin/activate
    
    # --- install opengen
    python setup.py install
    
    # --- uncomment to run main file
    # run opengen main.py
    
    # --- run the tests
    cd opengen
    export PYTHONPATH=.
    python -W ignore test/test_constraints.py -v
    python -W ignore test/test.py -v
}

main() {
    if [[ -v DO_DOCKER ]]
    then
        echo 'testing!'
    fi
}

main
