---
id: python-ocp-3
title: Building OCPs
description: Optimal Control with OpEn/opengen
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" async src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.1/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>
<style>
.but{
  border: none;
  color: #348c4f;
  padding: 15px 20px;
  text-align: center;
  text-decoration: none;
  display: inline-block;
  font-size: 16px;
  margin: 0px 0px;
  cursor: pointer;
  width: 250px;
  border-radius: 8px;
}
</style>
<style>
.but1 {
    background-color: #e9e642;
}
</style><style>
.but2 {
    background-color: #008CBA;
}
</style>

:::note Info
The functionality presented here was introduced in `opengen` version [`0.10.0a1`](https://pypi.org/project/opengen/#history). The API is still young and is likely to change in version `0.11`.
:::

Once an OCP has been defined, it can be turned into a generated optimizer
using `og.ocp.OCPBuilder`. This builder lowers the high-level OCP to the
standard OpEn problem format and then invokes the usual code-generation
pipeline.

At a minimum, you need:

- the OCP definition,
- optimizer metadata,
- a build configuration, and
- a solver configuration.

For example:

```python
ocp_optimizer = og.ocp.OCPBuilder(
    ocp,
    metadata=og.config.OptimizerMeta().with_optimizer_name("my_ocp"),
    build_configuration=og.config.BuildConfiguration()
        .with_build_python_bindings(),
    solver_configuration=og.config.SolverConfiguration()
        .with_tolerance(1e-5)
        .with_delta_tolerance(1e-5),
).build()
```

The call to `.build()` generates the solver and returns a
`GeneratedOptimizer`, which can then be used directly:

```python
result = ocp_optimizer.solve(x0=[0.4, 0.2], xref=[0.0, 0.0])
```

If you prefer to access the generated solver over TCP instead of direct Python
bindings, use `with_tcp_interface_config(...)` in the build configuration:

```python
ocp_optimizer = og.ocp.OCPBuilder(
    ocp,
    metadata=og.config.OptimizerMeta().with_optimizer_name("my_ocp_tcp"),
    build_configuration=og.config.BuildConfiguration()
        .with_tcp_interface_config(),
    solver_configuration=og.config.SolverConfiguration(),
).build()
```

In both cases, the high-level interface remains the same:

```python
result = ocp_optimizer.solve(x0=[0.4, 0.2], xref=[0.0, 0.0])
```

For more details on build configurations, TCP servers, Python bindings, and
the low-level code-generation pipeline, see the main
[Python interface documentation](https://alphaville.github.io/optimization-engine/docs/python-interface).
