#! /bin/bash
set -euxo pipefail

main() {
    # Run Rust tests
    # ------------------------------------
    cargo test --target $TARGET



    # Run Python tests
    # ------------------------------------

    # NOTE: Temporarily deactivated
    #       TODO: Re-enable later

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
    
    # --- temporarily commented out:
    # python -W ignore test/test.py -v
}

main
