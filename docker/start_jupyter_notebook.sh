#!/bin/bash
# Shell script that starts the Jupyter Python Notebook
# in the docker container

if [ -z "$JUPYTER_NOTEBOOK_PASSWORD" ]
then
      echo "No Jupyter Notebook password provided - starting in unsafe mode"
      echo "Set password using -e JUPYTER_NOTEBOOK_PASSWORD={sha of password}"
      jupyter notebook  \
          --port=8888 --no-browser  \
          --ip=0.0.0.0 --allow-root  \
          --NotebookApp.password='' --NotebookApp.token=''
else
      echo "Jupyter Notebook password provided by user"
      jupyter notebook  \
          --port=8888 --no-browser  \
          --ip=0.0.0.0 --allow-root  \
          --NotebookApp.password=$JUPYTER_NOTEBOOK_PASSWORD
fi

