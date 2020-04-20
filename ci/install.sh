#!/bin/bash

set -euxo pipefail

main() {
    rustup toolchain remove stable && rustup toolchain install stable
    sudo pip install --upgrade pip
    sudo pip install virtualenv --upgrade --ignore-installed six
}

main
