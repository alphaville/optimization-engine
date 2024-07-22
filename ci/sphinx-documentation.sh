#!/bin/bash
set -euxo pipefail

# This script generates and publishes the sphinx API documentation
# of opengen

# Firstly, get the current branch and the current commit message
current_branch=$(git rev-parse --abbrev-ref HEAD)
commit_message=$(git log -1 --pretty=format:"%s")
commit_hash=$(git rev-parse --short HEAD)

echo "CURRENT BRANCH:" $current_branch
echo "COMMIT MESSAGE:" $commit_message
echo "COMMIT HASH   :" $commit_hash

# If the current branch is not master and the current commit
# message does not contain [docit], then stop
magic_docs_keyword="[docit]"
if [[ "$commit_message" != *"$magic_docs_keyword"* ]] && [ "$current_branch" != "master" ]; then 
    echo "The commit message does not contain [docit] and this is not the master branch /exiting! bye :)";
    exit 0;
fi

# Install sphinx and the RTD theme
cd $GITHUB_WORKSPACE/
pip install sphinx
pip install sphinx-rtd-theme

# Install opengen
pushd open-codegen
pip install .
popd # back to $GITHUB_WORKSPACE

# Set git username and email
git config --global user.name "github-actions"
git config --global user.email "actions@github.com"

# Checkout gh-pages and delete the folder api-dox (don't push yet)
# At the end, return to the current branch
git fetch origin gh-pages:gh-pages || :
git checkout gh-pages
if [ -d "api-dox/" ]; then
    git rm -r api-dox/
    git commit -m "remove old api-dox files"
fi
git checkout $current_branch

# Build the docs
rm -rf sphinx
mkdir -p sphinx
pushd sphinx-dox
sphinx-apidoc -o ./source/ ../open-codegen/opengen
echo Last updated: $(date -u)  >> source/index.rst; sed '$d' source/index.rst; # update date at the end of file
make html || :
cp -r build/html/ ../sphinx
git checkout source/index.rst # no need to commit this
popd # back to $GITHUB_WORKSPACE

# Push to gh-pages
rm -rf api-dox/
mv sphinx/ api-dox/
git checkout gh-pages
git add api-dox/
touch .nojekyll
git add .nojekyll
git commit -m "documentation for $commit_hash"
git push origin gh-pages || :
