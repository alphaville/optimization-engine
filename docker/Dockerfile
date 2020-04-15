# -------- Licence -------------------------------------------------------------
# Dual license: MIT and Apache v2
#
# Copyright (c) 2017 Pantelis Sopasakis and Emil Fresk
# ------------------------------------------------------------------------------


# -------- Run Instructions ----------------------------------------------------
# Run the docker image and start a terminal to access it by running:
#
# $ docker run -p 8888:8888 alphaville/open:0.1
#
# You will then be able to access your Jupyter Notebook at
#  http://localhost:8888/
#
# ***IMPORTANT***
# The password is: open
# ------------------------------------------------------------------------------

FROM debian

# Labels for the docker image
LABEL 	maintainer="Pantelis Sopasakis <p.sopasakis@gmail.com>" \
        license="MIT license" \
        description="Jupyter notebook for Optimization Engine (OpEn)"

WORKDIR /open

COPY example.ipynb /open/

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

# Install necessary Python libraries
RUN pip3 install wheel    # If we don't install wheel first, pyyaml
                          # complains
RUN pip3 install opengen  # our library

# Configure Jupyter Notebook: set password (it is: open)
RUN jupyter notebook --generate-config  \
    && echo "c.NotebookApp.password='sha1:b983859a4bae:fb295c0c9b656bc2bdeffa359d8585f5cc3c2ac5'">>/root/.jupyter/jupyter_notebook_config.py

# Run the following command every time this docker image 
# is executed
CMD ["jupyter", "notebook", "--port=8888", "--no-browser", "--ip=0.0.0.0", "--allow-root"]



# -------- Build Instructions --------------------------------------------------
# Build the docker image by running (for DEVELOPERS only): 
#
# $ docker image build -t alphaville/open .
#
# from within the base directory of this project.
#
# ------------------------------------------------------------------------------
