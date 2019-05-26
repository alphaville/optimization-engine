set -euxo pipefail

main() {
    cargo check --target $TARGET
    cargo test --target $TARGET
    cd open-codegen 
    set -euox pipefail
    virtualenv -p python3.4 venv
    ls
    source venv/bin/activate
    python setup.py install
}

main
