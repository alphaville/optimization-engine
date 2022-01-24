#!/bin/sh
# This is a script to faciliate the release of new versions
# Make sure you have created a virtual environment, `venv`
# and that you have installed twine
 
echo "[OpEnGen] Checking out master"
git checkout master
git pull origin master

echo "[OpEnGen] Clean build"
rm -rf ./build ./dist opengen.egg-info 

echo "[OpEnGen] Build (within a virtual environment)"
source venv/bin/activate
pip install .

echo "[OpEnGen] Build dist"
python setup.py sdist bdist_wheel

echo "[OpEnGen] Check..."
ok=`twine check dist/** | grep PASSED | wc -l`
echo $ok
if [ $ok -eq 2 ]; then
    echo "[OpEnGen] twine check: all passed"
else
    echo "[OpEnGen] twine: some checks did not pass"
    exit 2
fi

echo "[OpEnGen] Uploading to pypi..."
read -r -p "Are you sure? [y/N] " response
case "$response" in
    [yY][eE][sS]|[yY]) 
        echo "[OpEnGen] Thanks, uploading to PyPi now"
        twine upload dist/*
        ;;
    *)
        echo "---"
        ;;
esac

echo "[OpEnGen] Don't forget to create a tag; run:"
a=`cat VERSION`
echo "\$ git tag -a opengen-$a -m 'opengen-$a'"
echo "\$ git push --tags"