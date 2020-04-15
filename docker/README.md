# OpEn Jupyter Notebook

Hmm.. that [OpEn](https://alphaville.github.io/optimization-engine/) framework looks really cool, but I'm too lazy to spend two minutes to [install](https://alphaville.github.io/optimization-engine/docs/installation) it!

## TaDa...

You can now download this docker image using

```console
docker pull alphaville/open
```

and then run it with

```console
docker run -p 8888:8888 -it alphaville/open
```

## What it does

It starts a Jupyter Notebook at [localhost:8888](http://localhost:8888) without a password.

## Set a password

To set up a password for your Python Notebook:

- [Hash your password](https://jupyter-notebook.readthedocs.io/en/stable/public_server.html#hashed-pw)
- Run your docker image with:

```
docker run -e JUPYTER_NOTEBOOK_PASSWORD=... -p 8888:8888 -it alphaville/open
```

For example, let's say you want to set the password **open**. Then do

```
docker run   \
  -e JUPYTER_NOTEBOOK_PASSWORD=sha1:898ca689bf37:2ee883bfd6ffe82a749a86e37964700bd06a2ff9  \
  -p 8888:8888 -it alphaville/open
```


