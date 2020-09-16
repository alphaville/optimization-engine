---
id: installation
title: OpEn Installation
sidebar_label: Installation
description: How to install OpEn
---

<script>
  ((window.gitter = {}).chat = {}).options = {
    room: 'alphaville/optimization-engine'
  };
</script>
<script src="https://sidecar.gitter.im/dist/sidecar.v1.js" async defer></script>

## OpEn Requirements: Rust and clang

Before you start, you need to install

* **Rust**, following the official <a href="https://www.rust-lang.org/tools/install" rel="nofollow" target="blank">installation guide</a>,
  - _Why?_ The Rust compiler is an essential component of OpEn; you will most likely
    not need to write (or compile yourself) any Rust code, but OpEn's Python/MATLAB
    interface will need the compiler to build your optimizer
  - _How?_ Follow the <a href="https://www.rust-lang.org/tools/install" rel="nofollow" target="blank">instructions</a>;
    in a nutshell, on Linux and MacOSX run:
```sh
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```
   and add `/.cargo/bin` to your path - e.g., on Linux, put the following line in your `~/.profile` file:
```sh
# Add this to your ~/.profile file
export PATH="$HOME/.cargo/bin:$PATH"
```
then **logout and login again** (or restart) for this to take effect.
* **clang**, following this <a href="https://github.com/rust-lang/rust-bindgen/blob/master/book/src/requirements.md" rel="nofollow" target="blank">guide</a>
  - Why? OpEn uses CasADi to build certain functions in C, which then need to be
    called from OpEn's core solver in Rust. For that purpose we need **bindgen**,
    which requires **clang**


## Python Interface
As simple as

```console
pip install opengen
```

You might need to prepend `sudo` on some Linux systems.


Note that OpEn requires Python **version 3.5 or newer**. You might, therefore,
need to install it using

```console
pip3 install opengen
```

OpEn may run on earlier versions of Python (as old as 2.7), but we cannot promise
you that (the main difficulty being the installation of dependencies).

We strongly recommend that you use `virtualenv`.

### Python installation with virtualenv

To install OpEn in a virtual environment, using `virtualenv`, you first
need to create such an environment, then activate it, and lastly, install
`opengen` as above using `pip`. That is, you need to run:

```console
virtualenv -p python3.6 venv36
source venv36/bin/activate
pip install opengen
```

## MATLAB Interface
You first need to download [Optimization Engine](https://github.com/alphaville/optimization-engine/archive/master.zip), `cd` to `./matlab/` and run

```matlab
setup_open
```

This will include to your MATLAB path all necessary folders.

You also need to <a href="https://web.casadi.org/" target="_blank"><b>download and install CasADi</b></a>.

## OpEn in Rust
To use **OpEn** in your Rust project, add the following in your project's `Cargo.toml` file:

```
[dependencies]
optimization_engine = "*"
```

You may replace the asterisk with some particular version (e.g., `optimization_engine = "0.6.0"`).
**OpEn** is available on <a href="https://crates.io/crates/optimization_engine">crates.io</a>.



## Install from source

This section is intended mainly for developers and contributors.

### Get the source code: clone and/or fork

The first step is to get the latest version from OpEn's git repository:

```console
git clone \
https://github.com/alphaville/optimization-engine.git \
optimization-engine
```

To clone a particular branch do

```console
git clone -b <branch_name> \
https://github.com/alphaville/optimization-engine.git \
optimization-engine
```

If you want to contribute to OpEn, you should rather *fork* OpEn on [github](https://github.com/alphaville/optimization-engine). In that case, you should also read the [contributing guide](contributing) as well.


### Install opengen

Go intro `optimization-engine/open-codegen` and create a virtual environment:

```sh
cd optimization-engine/open-codegen
virtualenv -p python3.6 venvopen
source venvopen/bin/activate
python setup.py install
```

You're ready to go!

It's a good idea to use an IDE, such as 
<a href="https://www.jetbrains.com/pycharm/" target="_blank">PyCharm</a>. 
Use the above virtual environment (`venvopen`) in PyCharm:

- go to Run > Edit Configurations > Add new configuration
- Script path: specify `main.py`
- Working dir: `optimization-engine/open-codegen/opengen`
- Python interpreter: `venvopen`

### Install OpEn in Rust


As easy as:

```
cd optimization-engine
cargo build
```

If you need to use `opengen` - the Python interface of OpEn - with a local
version of the Rust library, use `with_open_version(local_path=...)` in 
your code. Read the [advanced options](python-advanced#build-options)
for details.
