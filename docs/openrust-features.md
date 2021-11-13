---
id: openrust-features
title: Features
description: Rust Features
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.1/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>


## Features

The following [features] are supported in OpEn:

### Memory allocators

You can use the feature `rp` to use the memory allocator `rpmalloc` and `jem` for `jemalloc`.
For example, you can include OpEn as a dependency in your Rust project by adding the following line to you `Cargo.toml` file:

```.toml
[dependencies]
optimization-engine = { version = "0.7", features = ["rp"] }
```

You cannot use both `rp` and `jemalloc`.


### WebAssembly Support

If you intend to use OpEn in WebAssembly you need to use the feature `wasm`.

```.toml
[dependencies]
optimization-engine = { version = "0.7", features = ["wasm"] }
```

<!-- Links -->

[features]: https://doc.rust-lang.org/cargo/reference/features.html
