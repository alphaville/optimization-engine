---
id: installation
title: OpEn Installation
sidebar_label: Installation
---

## Install Rust

Install Rust following the official [installation guide](https://www.rust-lang.org/tools/install).


## Python Interface
As simple as

```
pip install opengen
```

You might need to prepend `sudo` on some Linux 
systems.

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

You may replace the asterisk with some particular version (e.g., `optimization_engine = "0.2.0"`).
**OpEn** is available on [crates.io](https://crates.io/crates/optimization_engine).
