# Interesting: https://gist.github.com/brson/3762a4d581ef867d505f1e83d2d23bc1


# -------- Licence -------------------------------------------------------------
# Dual license: MIT and Apache v2
#
# Copyright (c) 2017 Pantelis Sopasakis and Emil Fresk
# ------------------------------------------------------------------------------


# -------- Run Instructions ----------------------------------------------------
# Run the docker image and start a terminal to access it by running:
#
# $ docker run -p 8888:8888 alphaville/open:0.1
# ------------------------------------------------------------------------------

FROM debian

# Labels for the SuperSCS docker image
LABEL 	maintainer="Pantelis Sopasakis <p.sopasakis@gmail.com>" \
	version="v0.1" \
        license="MIT license" \
        description="Jupyter notebook for Optimization Engine (OpEn)"

WORKDIR /open

RUN "sh" "-c" "echo nameserver 8.8.8.8 >> /etc/resolv.conf"
RUN apt-get update -y \
    && apt-get -y --no-install-recommends install \
        build-essential \
        curl \
        jupyter-notebook  \
        python3 \
        python3-pip \
        python3-setuptools

# Get Rust

RUN curl https://sh.rustup.rs -sSf | bash -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

RUN apt-get -y --no-install-recommends install libpython3-dev && pip3 install wheel
RUN pip3 install opengen

CMD ["jupyter", "notebook", "--port=8888", "--no-browser", "--ip=0.0.0.0", "--allow-root"]




# -------- Build Instructions --------------------------------------------------
# Build the docker image by running (for DEVELOPERS only): 
#
# $ docker image build -t alphaville/open:{version-name} .
#
# from within the base directory of this project.
#
# ------------------------------------------------------------------------------
