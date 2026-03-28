#!/bin/sh
set -eu

# This script facilitates releasing a new version of opengen to PyPI.
# It expects a local virtual environment at ./venv with publishing tools.

version=$(cat VERSION)
current_branch=$(git rev-parse --abbrev-ref HEAD)

is_alpha_version=false
case "$version" in
    *a[0-9]*)
        is_alpha_version=true
        ;;
esac

if [ "$current_branch" != "master" ] && [ "$is_alpha_version" = false ]; then
    echo "[OpEnGen] Warning: version $version is not an alpha release and the current branch is '$current_branch' (not 'master')."
    printf "Proceed anyway? [y/N] "
    read -r response
    case "$response" in
        [yY][eE][sS]|[yY])
            echo "[OpEnGen] Proceeding from branch '$current_branch'"
            ;;
        *)
            echo "[OpEnGen] Publish cancelled"
            exit 0
            ;;
    esac
fi

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

echo "[OpEnGen] You are about to publish version $version from branch '$current_branch'."
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
echo "\$ git tag -a opengen-$version -m 'opengen-$version'"
echo "\$ git push --tags"
