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

* **Rust**, following the official [installation guide](https://www.rust-lang.org/tools/install),
  - Why? The Rust compiler is an essential component of OpEn; you will most likely
    not need to write (or compile yourself) any Rust code, but OpEn's Python/MATLAB
    interface will need the compiler to build your optimizer
* **clang**, following this [guide](https://github.com/rust-lang/rust-bindgen/blob/master/book/src/requirements.md)
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

You also need to [**download and install CasADi**](https://web.casadi.org/).

## OpEn in Rust 
To use **OpEn** in your Rust project, add the following in your project's `Cargo.toml` file:

```
[dependencies]
optimization_engine = "*"
```

You may replace the asterisk with some particular version (e.g., `optimization_engine = "0.6.0"`).
**OpEn** is available on [crates.io](https://crates.io/crates/optimization_engine).


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

It's a good idea to use an IDE, such as [PyCharm](https://www.jetbrains.com/pycharm/). Use the above virtual environment (`venvopen`) in PyCharm:

- go to Run > Edit Configurations > Add new configuration
- Script path: specify `main.py`
- Working dir: `optimization-engine/open-codegen/opengen`
- Python interpreter: `openve`

### Install OpEn in Rust


As easy as:

```
cd optimization-engine
cargo build
```
