# OpEn JupyterLab Docker Image

This directory contains the Docker image used to try OpEn quickly from a browser-based Python environment.

The current Dockerfile and Docker image version documented here is `0.6.0`.
This image bundles the Python package `opengen==0.10.0`.

## What is in the image?

The image is built from `python:3.12-slim-bookworm` and contains:

- Python 3.12 in a virtual environment at `/venv`
- `opengen==0.10.0`
- JupyterLab
- Rust installed through `rustup`
- Example notebooks copied into `/open/notebooks`

## Pull and run the image

```console
docker pull alphaville/open:0.6.0
docker run --name open-jupyter -p 127.0.0.1:8888:8888 -it alphaville/open:0.6.0
```

Open JupyterLab at:

- `http://127.0.0.1:8888/lab`

By default, the container starts with Jupyter's token authentication enabled. The token is printed in the container logs.

To see the token:

```console
docker logs open-jupyter
```

If you stop the container, restart it with:

```console
docker start -ai open-jupyter
```

## Password-based access

To run JupyterLab with a password instead of the default token, provide a hashed password through `JUPYTER_NOTEBOOK_PASSWORD`.

```console
docker run \
  --name open-jupyter \
  -e JUPYTER_NOTEBOOK_PASSWORD=... \
  -p 127.0.0.1:8888:8888 \
  -it alphaville/open:0.6.0
```

For password hashing instructions, see the Jupyter documentation:

- https://jupyter-server.readthedocs.io/en/latest/operators/public-server.html

## Working with notebooks

The bundled example notebook is stored in this repository under:

- `docker/notebooks/example.ipynb`

Inside the container it is available under:

- `/open/notebooks/example.ipynb`

To keep your own notebooks between container runs, mount a volume onto `/open`:

```console
docker volume create OpEnVolume
docker run --name open-jupyter \
  --mount source=OpEnVolume,destination=/open \
  -p 127.0.0.1:8888:8888 \
  -it alphaville/open:0.6.0
```

## Open a shell in the container

```console
docker exec -it open-jupyter bash
```

The virtual environment is available at `/venv`.

## Maintainer Notes

Build the image from within the `docker/` directory:

```console
docker image build -t alphaville/open:0.6.0 .
```

You can also build it from the repository root:

```console
docker image build -t alphaville/open:0.6.0 -f docker/Dockerfile docker
```

Before publishing, do a quick smoke test:

```console
docker run --rm -p 127.0.0.1:8888:8888 alphaville/open:0.6.0
```

Then push the tag you built:

```console
docker push alphaville/open:0.6.0
```

If you want to check the bundled Python package version inside the image, run:

```console
docker run --rm --entrypoint /venv/bin/python alphaville/open:0.6.0 -c "from importlib.metadata import version; print(version('opengen'))"
```
