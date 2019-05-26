set -euxo pipefail

main() {
    if [ $TARGET != x86_64-unknown-linux-gnu ]; then
        rustup target add $TARGET
    fi
    sudo pip install --upgrade pip
    sudo pip install virtualenv --upgrade
    python --version
    pip --version
    virtualenv --version
    echo $TRAVIS_PYTHON_VERSION
}

main
