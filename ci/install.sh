set -euxo pipefail

main() {
    # If DO_DOCKER is unset, install necessary stuff for testing
    if [ -z $DO_DOCKER ]; then
        rustup toolchain remove stable && rustup toolchain install stable
        sudo pip install --upgrade pip
        sudo pip install virtualenv --upgrade --ignore-installed six
    fi
}

main
