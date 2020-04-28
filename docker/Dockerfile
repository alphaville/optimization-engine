# -------- Licence -------------------------------------------------------------
# Dual license: MIT and Apache v2
#
# Copyright (c) 2017 Pantelis Sopasakis and Emil Fresk
# ------------------------------------------------------------------------------


# -------- Run Instructions ----------------------------------------------------
# Run the docker image and start a terminal to access it by running:
#
# $ docker run -p 8888:8888 -it alphaville/open
#
# You will be able to access your Jupyter Notebook at http://localhost:8888/
# without a password. To set a password, read the documentation.
#
# ------------------------------------------------------------------------------

FROM debian:stable

# Labels for the docker image
LABEL 	maintainer="Pantelis Sopasakis <p.sopasakis@gmail.com>" \
        license="MIT license" \
        description="Jupyter notebook for Optimization Engine (OpEn)"

WORKDIR /open

# Example Python notebook
COPY example.ipynb /open/

ENV PATH="/root/.cargo/bin:${PATH}"

# These commands are groupped into a separate RUN to faciliate
# caching; it is unlikely that any of the following will change
# in future versions of this docker image
RUN "sh" "-c" "echo nameserver 8.8.8.8 >> /etc/resolv.conf"  \
    && apt-get update -y \
    && apt-get -y --no-install-recommends install \
        build-essential \
        curl \
        jupyter-notebook  \
        python3 \
        python3-pip \
        python3-setuptools  \
    &&  curl https://sh.rustup.rs -sSf | bash -s -- -y  \
    &&  pip3 install wheel  \ 
    &&  pip3 install opengen  \
    &&  apt-get clean  \
    &&  rm -rf /var/lib/apt/lists/*


# Run the following command every time this docker image
# is executed
COPY start_jupyter_notebook.sh /
RUN ["chmod", "+x", "/start_jupyter_notebook.sh"]
ENTRYPOINT ["/start_jupyter_notebook.sh"]
CMD ["/start_jupyter_notebook.sh"]



# -------- Build Instructions --------------------------------------------------
# Build the docker image by running (for DEVELOPERS only): 
#
# $ docker image build -t alphaville/open .
#
# from within the base directory of this project.
#
# ------------------------------------------------------------------------------
