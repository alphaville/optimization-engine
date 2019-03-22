---
title: OpEn: a pure Rust optimizer
author: Pantelis Sopasakis
authorURL: http://twitter.com/isToxic
authorImageURL: https://pbs.twimg.com/profile_images/1062281000171003904/KkolV9Eg_400x400.jpg
---

The majority of optimization packages in Rust, such as [IPOPT](https://crates.io/crates/ipopt), [OSQP](https://crates.io/crates/osqp), [NLOPT](https://crates.io/crates/nlopt), are essentially bindings (interfaces) to other software. There are a few pure-Rust packages, such as [rustimization](https://crates.io/crates/rustimization), [argmin](https://crates.io/crates/argmin), they implement algorithms which are not suitable for embedded nonconvex optimization.

![Rust language](/optimization-engine/img/rust1.jpeg)

**OpEn** is the first **pure-Rust** package 