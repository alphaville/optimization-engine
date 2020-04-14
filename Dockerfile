# Interesting: https://gist.github.com/brson/3762a4d581ef867d505f1e83d2d23bc1

FROM debian

RUN "sh" "-c" "echo nameserver 8.8.8.8 >> /etc/resolv.conf"
RUN apt-get update -y \
    && apt-get -y --no-install-recommends install \
        build-essential \
        curl \
        jupyter-notebook  \
        python3 \
        python3-virtualenv

# Get Rust
RUN curl https://sh.rustup.rs -sSf | bash -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

RUN python3 -m virtualenv --python=/usr/bin/python3 /opt/venv \
    && . /opt/venv/bin/activate  \
    && pip install opengen

# What happens when you run this image...
CMD [". /opt/venv/bin/activate"]
CMD ["jupyter", "notebook", "--port=8888", "--no-browser", "--ip=0.0.0.0", "--allow-root"]




