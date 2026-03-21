---
id: docker
title: Docker
sidebar_label: Docker
description: Docker image of a JupyterLab environment to run OpEn
---

<img src="/optimization-engine/img/docker.gif" alt="OpEn in Docker container" width="200"/>

## What is Docker?

[Docker](https://www.docker.com/) is a tool for packaging applications and their dependencies into containers. A container can run on different machines without requiring you to recreate the same environment manually.

## What is JupyterLab?

According to [jupyter.org](https://jupyter.org/), JupyterLab is a web-based development environment for notebooks, code, and data.

## Requirements

You need to have Docker installed. See the [official installation instructions](https://docs.docker.com/get-docker/).

## Pull and Run the Docker Image

You can download the current Docker image using:

```console
docker pull alphaville/open:0.6.0
```

and then run it with:

```console
docker run --name open-jupyter -p 127.0.0.1:8888:8888 -it alphaville/open:0.6.0
```

This starts JupyterLab and makes it available at:

- [http://127.0.0.1:8888/lab](http://127.0.0.1:8888/lab)

The image currently includes:

- Python 3.12
- `opengen==0.10.0`
- JupyterLab
- Rust installed through `rustup`
- Example notebooks under `/open/notebooks`

By default, JupyterLab starts with token authentication enabled. To view the token:

```bash
docker logs open-jupyter
```

It is always a good idea to give your container a name using `--name`.

<div class="alert alert-info">
<b>Info:</b> Use <code>docker run</code> only the first time you create the container. Use <code>docker start -ai open-jupyter</code> to start it again later.</div>

<div class="alert alert-success">
<b>Tip:</b> To stop a running container, do <code>docker stop open-jupyter</code>.</div>

## Configure the Docker Image

### Configure password-based access

To run JupyterLab with a password instead of the default token, provide a hashed password through `JUPYTER_NOTEBOOK_PASSWORD`:

```bash
docker run \
  --name open-jupyter \
  -e JUPYTER_NOTEBOOK_PASSWORD='your hashed password' \
  -p 127.0.0.1:8888:8888 \
  -it alphaville/open:0.6.0
```

For password hashing instructions, see the [Jupyter Server documentation](https://jupyter-server.readthedocs.io/en/latest/operators/public-server.html).

<details>
  <summary>How to set up a password</summary>

  You can read more about how to set up a password for your 
  Python notebook [here](https://jupyter-server.readthedocs.io/en/latest/operators/public-server.html).
  TL;DR: run the following command:

  ```bash
  docker run --rm -it --entrypoint /venv/bin/python \
    alphaville/open:0.6.0 \
    -c "from jupyter_server.auth import passwd; print(passwd())"
  ```

  You will be asked to provide your password twice. Then a string 
  will be printed; this is your hashed password.
</details>

### Configure port

You can access JupyterLab on a different host port by changing Docker's port forwarding. For example, to use port `80` on your machine:

```bash
docker run -p 80:8888 alphaville/open:0.6.0
```

Then JupyterLab will be available at `http://localhost/lab`.

### Work with notebooks

The bundled example notebook is available inside the container at:

```text
/open/notebooks/example.ipynb
```

To persist your own notebooks across container restarts, mount a Docker volume onto `/open`:

```bash
docker volume create OpEnVolume
docker run --name open-jupyter \
  --mount source=OpEnVolume,destination=/open \
  -p 127.0.0.1:8888:8888 \
  -it alphaville/open:0.6.0
```

### Load additional Python packages

You can install additional packages from inside JupyterLab. For example:

```python
!pip install matplotlib
```

Packages installed into a persistent container or volume-backed environment will still be there the next time you access it.

### Open a terminal in the container

Suppose you have a running Docker container with name `open-jupyter`. To open a shell in it:

```bash
docker exec -it open-jupyter /bin/bash
```

The Python virtual environment is available at `/venv`.

### Download your optimizer

To download a generated optimizer from JupyterLab, first create an archive:

```python
!tar -cf rosenbrock.tar.gz optimizers/rosenbrock
```
