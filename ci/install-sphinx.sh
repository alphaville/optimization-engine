#!/bin/bash

# Install sphinx

cd "$( dirname "${BASH_SOURCE[0]}" )"/..

set -ex
pushd /tmp

[ -d sphinx ] || \
    git clone https://github.com/tttapa/sphinx --branch 4.x --depth 1
pushd sphinx
python setup.py install
popd

popd