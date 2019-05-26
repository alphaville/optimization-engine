set -euxo pipefail

main() {
    cargo check --target $TARGET
    cargo test --target $TARGET
    cd open-codegen
    virtualenv venv
    ls
    source venv/bin/activate
    python setup.py install
}

main
