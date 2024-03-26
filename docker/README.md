# OpEn Jupyter Notebook

Hmm.. that [OpEn](https://alphaville.github.io/optimization-engine/) framework looks really cool, but I'm too lazy to spend two minutes to [install](https://alphaville.github.io/optimization-engine/docs/installation) it!

## TaDa...

You can now download this docker image using

```console
docker pull alphaville/open
```

and then run it with

```console
docker run --name open-jupyter -p 8888:8888 -it alphaville/open
```

Note that this will create a docker container with name `open-jupyter`. If you stop it, you can restart it with (don't do `docker run` again)

```console
docker start -ai open-jupyter
```

## What it does

It starts a Jupyter Notebook at [localhost:8888](http://localhost:8888) without a password.

## What's in the docker image?

This docker image is build from `debian:stable` and contains:

- A virtual environment (with Python 3)
- Opengen [v0.8.0](https://github.com/alphaville/optimization-engine/releases/tag/opengen-0.8.0)
- The [latest version](https://crates.io/crates/optimization_engine) of the OpEn rust solver is installed automatically
- Jupyter notebook (runs automatically when the image runs)


## How to open a terminal into the docker image

Just 

```console
docker exec -it open-jupyter bash
```

This will open a bash shell to the docker image with name `open-jupyter`; this is the name we specified above when we ran the image (using `--name open-jupyter`). In this bash shell, the virtual environment on which the Jupyter notebook is running is enabled by default. 


### How to run with specified volume

Firstly, you need to create a volume. You only need to do this once (unless you want to create different volumes). As an example, let us create a docker volume with name `OpEnVolume`:

```console
docker volume create OpEnVolume
```

Next, let us run the image `alphaville/open:0.5.0` with the above volume:

```console
docker run --name open-jupyter \
           --mount source=OpEnVolume,destination=/open \
           -p 8888:8888 \
           -it alphaville/open:0.5.0
```

## Set a password

To set up a password for your Python Notebook:

- [Hash your password](https://jupyter-notebook.readthedocs.io/en/stable/public_server.html#hashed-pw)
- Run your docker image with:

```
docker run -e JUPYTER_NOTEBOOK_PASSWORD=... -p 8888:8888 -it alphaville/open
```

For example, let's say you want to set the password **open**. Then do

```console
docker run   \
  -e JUPYTER_NOTEBOOK_PASSWORD=sha1:898ca689bf37:2ee883bfd6ffe82a749a86e37964700bd06a2ff9  \
  -p 8888:8888 -it alphaville/open
```


