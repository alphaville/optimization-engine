---
id: docker
title: Docker
sidebar_label: Docker
description: Docker image of a Jupyter Python Notebook to run OpEn
---

<img src="/optimization-engine/img/docker.gif" alt="OpEn in Docker container" width="200"/>

## What is Docker?

[Docker](https://www.docker.com/) is a tool that facilitates the execution and deployment of applications (often web applications) using containers. Containers package the applications along with the entirety of its context such as libraries and configuration files. You may think of containers as independent, isolated and self-sufficient processes. A container is then guaranteed to run on any other Linux-based machine regardless of the exact operating system, local configurations. 

Docker has nowadays become extremely popular. Developers package their software in bundles known as *Docker images* which can be stored in Docker Hub: an online repository. Then the users can obtain such images and run them locally.

Docker is open-source and free and can be easily installed on any Linux, Mac OSX or Windows machine. 


## What is a Jupyter Notebook?
According to [jupyter.org](https://jupyter.org/), "JupyterLab is a web-based interactive development environment for Jupyter notebooks, code, and data."


## Requirements

You need to have Docker installed ([instructions](https://docs.docker.com/get-docker/)).

## Pull and Use Docker Image

Hmm.. that [OpEn](https://alphaville.github.io/optimization-engine/) framework looks really cool. I wonder whether it is possible to try it out very quickly without installing much...

TaDaaa...

You can now download this docker image using

```console
docker pull alphaville/open
```

and then run it with

```console
docker run -p 8888:8888 -it --name=open-notebook alphaville/open
```
It will start a Jupyter Notebook at [localhost:8888](http://localhost:8888) without a password.

It is always a good idea to give your docker container a name using `--name`. For example

<div class="alert alert-info">
<b>Info:</b> Use <code>docker run alphaville/open</code> only the first time you run the container. Use <code>docker start {container_name}</code> all subsequent times. Always give a name to your containers.</div>

<div class="alert alert-success">
<b>Tip:</b> To stop a running container do <code>docker stop {container_name}</code>.</div>


## Configure Docker Image

### Configure password

To set up a password for your Python Notebook:

- <a href="https://jupyter-notebook.readthedocs.io/en/stable/public_server.html#hashed-pw" target="_blank">Hash your password</a>
- Run your docker image with:

```
docker run  \
  -e JUPYTER_NOTEBOOK_PASSWORD={your hashed password}  \
  -p 8888:8888  \
  --name=open-notebook  \
  alphaville/open
```

For example, let's say you want to set the password **open**. Then do

```console
docker run   \
  -e JUPYTER_NOTEBOOK_PASSWORD=sha1:898ca689bf37:2ee883bfd6ffe82a749a86e37964700bd06a2ff9  \
  -p 8888:8888  \
  --name=open-notebook  \
  alphaville/open
```

### Configure port

You can access the Jupyter Notebook at a different port by configuring the port forwarding of Docker. For example, to run on port 80 run:

```console
docker run -p 80:8888 alphaville/open
```

Then, Jupyter Notebook will be accessible at http://localhost.
The argument `-p 80:8888` means that the internal port `8888` is mapped to the host's port `80`.


### Load additional python packages

You can load any packages you like in your Python Notebook using `pip3` and prepend a `!` to your command. For example, if you want to load `matplotlib`, do 

```python
!pip3 install matplotlib
```

If you install a Python package in a certain docker container, it will be there the next time you access it.

### Open terminal into docker container

Suppose you have a running docker container with name `open-notebook` (you may specify a name for your docker container when you start it using `--name`). To open a terminal into it you can run

```console
docker exec -it open-notebook /bin/bash
```

### Download your optimizer
To download your optimizer from Jupyter you first need to create an archive ot it. You can do that using `tar` as follows

```python
!tar -cf rosenbrock.tar.gz optimizers/rosenbrock
```
