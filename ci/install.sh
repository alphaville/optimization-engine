set -euxo pipefail

main() {
    if [ $TARGET != x86_64-unknown-linux-gnu ]; then
        rustup target add $TARGET
    fi
    sudo pip install --upgrade pip
    sudo pip install virtualenv --upgrade --ignore-installed six
}

main
