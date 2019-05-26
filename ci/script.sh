set -euxo pipefail

main() {
    # Run Rust tests
    # ------------------------------------
    
    cargo check --target $TARGET
    cargo test --target $TARGET




    # Run Python tests
    # ------------------------------------

    # Create virtual environment
    cd open-codegen
    export PYTHONPATH=.
    virtualenv -p python$(PYTHON_VERSION) venv
    
    # activate venv
    source venv/bin/activate
    
    # install opengen
    python setup.py install
    
    # run opengen main.py
    cd opengen
    export PYTHONPATH=.
    python main.py
}

main
