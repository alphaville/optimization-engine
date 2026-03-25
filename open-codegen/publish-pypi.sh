#!/bin/sh
set -eu

# This script facilitates releasing a new version of opengen to PyPI.
# It expects a local virtual environment at ./venv with publishing tools.

echo "[OpEnGen] Checking out master"
git checkout master
git pull origin master

echo "[OpEnGen] Cleaning previous build artifacts"
rm -rf ./build ./dist ./opengen.egg-info

echo "[OpEnGen] Activating virtual environment"
. venv/bin/activate

echo "[OpEnGen] Installing packaging tools"
python -m pip install --upgrade pip build twine

echo "[OpEnGen] Building source and wheel distributions"
python -m build

echo "[OpEnGen] Checking distributions with twine"
python -m twine check dist/*

echo "[OpEnGen] Uploading to PyPI..."
printf "Are you sure? [y/N] "
read -r response
case "$response" in
    [yY][eE][sS]|[yY])
        echo "[OpEnGen] Uploading to PyPI now"
        python -m twine upload dist/*
        ;;
    *)
        echo "[OpEnGen] Upload cancelled"
        ;;
esac

echo "[OpEnGen] Don't forget to create a tag; run:"
version=$(cat VERSION)
echo "\$ git tag -a opengen-$version -m 'opengen-$version'"
echo "\$ git push --tags"
