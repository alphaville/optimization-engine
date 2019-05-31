set -euxo pipefail

main() {
    if [ $TARGET != x86_64-unknown-linux-gnu ]; then
        rustup target add $TARGET
    fi
    sudo pip install --upgrade pip
    sudo pip install virtualenv --upgrade
    python --version
    pip --version
    which python3
    export P3=`which python3`
    if [ ! -n "$P3" ]
    then
        echo "NO PYTHON3"
    else
        echo "PYTHON3 FOUND!"
        python3 --version
    fi
    virtualenv --version
}

main
