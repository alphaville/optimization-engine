#
# Dockerfile for OpEn/opengen
# Dockerfile version: 0.6.0
# Bundled Python package: opengen v0.10.0
#
# Copyright (c) 2017 Pantelis Sopasakis and Emil Fresk
#
#
#
# -------- Run Instructions ----------------------------------------------------
# Run the docker image and start a terminal to access it by running:
#
# $ docker run -p 127.0.0.1:8888:8888 -it alphaville/open
#
# You will be able to access JupyterLab at http://localhost:8888/lab
# using Jupyter's default token authentication. To set a password, read the
# documentation.
#
#

FROM python:3.12-slim-bookworm

ARG OPENGEN_VERSION=0.10.0

# Labels for the docker image
LABEL 	maintainer="Pantelis Sopasakis <p.sopasakis@gmail.com>" \
    license="MIT license" \
    description="JupyterLab for Optimization Engine (OpEn)"

WORKDIR /open

# Example notebook content bundled with the image
COPY notebooks/ /open/notebooks/

ENV VIRTUAL_ENV=/venv
ENV PATH="${VIRTUAL_ENV}/bin:/root/.cargo/bin:${PATH}"

# These commands are groupped into a separate RUN to faciliate
# caching; it is unlikely that any of the following will change
# in future versions of this docker image
RUN apt-get update -y \
    && apt-get -y --no-install-recommends install \
    build-essential \
    ca-certificates \
    curl \
    && curl https://sh.rustup.rs -sSf | bash -s -- -y --profile minimal \
    && python -m pip install --no-cache-dir --upgrade "pip>=26.0.1" \
    && python -m venv "${VIRTUAL_ENV}" \
    && "${VIRTUAL_ENV}/bin/python" -m pip install --no-cache-dir --upgrade "pip>=26.0.1" wheel \
    && "${VIRTUAL_ENV}/bin/python" -m pip install --no-cache-dir "opengen==${OPENGEN_VERSION}" "jupyterlab<5" \
    && "${VIRTUAL_ENV}/bin/python" -m pip uninstall -y py \
    && apt-get clean  \
    &&  rm -rf /var/lib/apt/lists/* \
    && echo "source ${VIRTUAL_ENV}/bin/activate" >> /root/.bashrc

VOLUME /open

EXPOSE 8888

# Run the following command every time this docker image
# is executed
COPY start_jupyter_notebook.sh /usr/local/bin/start_jupyter_notebook.sh
RUN ["chmod", "+x", "/usr/local/bin/start_jupyter_notebook.sh"]
CMD ["/usr/local/bin/start_jupyter_notebook.sh"]



# -------- Build Instructions --------------------------------------------------
# Build the docker image by running (for DEVELOPERS only): 
#
# $ docker image build -t alphaville/open:0.6.0 .
#
# from within the docker directory of this project.
#
#
# ------------------------------------------------------------------------------
