---
title: "OpEn: a pure Rust optimizer"
author: Pantelis Sopasakis
authorURL: https://github.com/alphaville
authorImageURL: https://avatars.githubusercontent.com/u/125415?v=4
---

The majority of optimization packages in Rust, such as [IPOPT](https://crates.io/crates/ipopt), [OSQP](https://crates.io/crates/osqp), [NLOPT](https://crates.io/crates/nlopt), are essentially bindings (interfaces) to other software. There are a few pure-Rust packages, such as [rustimization](https://crates.io/crates/rustimization), [argmin](https://crates.io/crates/argmin), they implement algorithms which are not suitable for embedded nonconvex optimization.

![Rust language](/optimization-engine/img/rust1.jpeg)

**OpEn** is the first **pure-Rust** package 
