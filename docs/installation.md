---
id: installation
title: OpEn Installation
sidebar_label: Installation
---

## Install Rust

Install Rust following the official [installation guide](https://www.rust-lang.org/tools/install).

## OpEn in Rust 
To use **OpEn** in your Rust project, add the following in your project's `Cargo.toml` file:

```
[dependencies]
optimization_engine = "*"
```

You may replace the asterisk with some particular version (e.g., `optimization_engine = "0.2.0"`).

**OpEn** is available on [crates.io](https://crates.io/crates/optimization_engine).

Then, in order to use **OpEn** in your Rust project, include the following line:

```rust
extern crate optimization_engine;
```

That's all! You don't need to download or build the crate `optimization_engine` manually; `cargo` will take care of that automatically.

In your Rust program, you will also have to include the following dependencies:


```rust
use optimization_engine::constraints::*;
use optimization_engine::core::panoc::*;
use optimization_engine::core::*;
```


## MATLAB Interface
You first need to download [Optimization Engine](https://github.com/alphaville/optimization-engine/archive/master.zip), `cd` to `./matlab/` and run 

```matlab
setup_open
```

This will include to your MATLAB path all necessary folders.

You also need to [**download and install CasADi**](https://web.casadi.org/).


## Python Interface
Coming soon