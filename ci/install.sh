set -euxo pipefail

main() {
    if [ $DO_DOCKER -eq 0 ]; then
        rustup toolchain remove stable && rustup toolchain install stable
        sudo pip install --upgrade pip
        sudo pip install virtualenv --upgrade --ignore-installed six
    fi
}

main
