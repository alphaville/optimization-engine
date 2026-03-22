#!/bin/bash
# Shell script that starts JupyterLab in the docker container

set -euo pipefail

JUPYTER_ARGS=(
    lab
    --ServerApp.root_dir=/open
    --port=8888
    --no-browser
    --ip=0.0.0.0
    --allow-root
)

if [ -n "${JUPYTER_NOTEBOOK_PASSWORD:-}" ]; then
    echo "Starting JupyterLab with password authentication"
    exec /venv/bin/jupyter "${JUPYTER_ARGS[@]}" \
        --ServerApp.password="${JUPYTER_NOTEBOOK_PASSWORD}" \
        --ServerApp.token=''
fi

if [ -n "${JUPYTER_NOTEBOOK_TOKEN:-}" ]; then
    echo "Starting JupyterLab with token authentication"
    exec /venv/bin/jupyter "${JUPYTER_ARGS[@]}" \
        --ServerApp.token="${JUPYTER_NOTEBOOK_TOKEN}"
fi

echo "No password provided - starting JupyterLab with default token authentication"
echo "Set JUPYTER_NOTEBOOK_PASSWORD to use password-only authentication"
exec /venv/bin/jupyter "${JUPYTER_ARGS[@]}"
