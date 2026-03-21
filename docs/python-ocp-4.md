---
id: python-ocp-4
title: Solving OCPs
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

After an OCP optimizer has been built, it can be called using the method
`solve(...)`. The main difference compared to the low-level interface is that
the OCP optimizer accepts **named parameters** rather than a manually packed
parameter vector.

For example, if the OCP declares the parameters `x0`, `xref`, `q`, and `r`,
the optimizer can be called as

```python
result = ocp_optimizer.solve(
    x0=[0.4, 0.2],
    xref=[0.0, 0.0],
    q=30.0,
    r=1.0,
)
```

Any parameter with a default value may be omitted. For example, if `xref`,
`q`, and `r` have defaults, then the following is also valid:

```python
result = ocp_optimizer.solve(x0=[0.4, 0.2])
```

Optional warm-start information may also be provided:

```python
result = ocp_optimizer.solve(
    x0=[0.4, 0.2],
    initial_guess=[0.0] * 20,
    initial_lagrange_multipliers=[0.0] * 5,
    initial_penalty=10.0,
)
```

## Solution object

The method `solve(...)` returns an object of type `OcpSolution`. This object
contains both the raw low-level solver status and OCP-oriented views such as
the stage-wise inputs and the state trajectory.

The most commonly used fields are:

- `result.inputs`: sequence of optimal inputs
- `result.states`: sequence of states
- `result.cost`: value of the objective at the solution
- `result.exit_status`: solver exit status
- `result.solve_time_ms`: total solver time in milliseconds

Additional solver diagnostics are also available:

- `result.penalty`
- `result.num_outer_iterations`
- `result.num_inner_iterations`
- `result.last_problem_norm_fpr`
- `result.f1_infeasibility`
- `result.f2_norm`
- `result.lagrange_multipliers`

For convenience, the solution object can be printed directly:

```python
print(result)
```

This displays a compact summary of the solution, including inputs, states, and
solver diagnostics.

## Direct bindings and TCP

The same high-level `solve(...)` interface is used regardless of whether the
optimizer was built with direct Python bindings or with the TCP interface.
When TCP is used, the underlying server is managed automatically by the
generated optimizer wrapper.

When the optimizer is accessed over TCP, the server is started automatically
the first time `solve(...)` is called and remains alive for subsequent calls.
When you are done with the optimizer, you should stop the TCP server
explicitly by calling:

```python
ocp_optimizer.kill()
```

This has no effect for optimizers using direct Python bindings.

## Reloading a generated optimizer

Generated OCP optimizers save a manifest automatically when they are built.
They can later be reloaded without rebuilding:

```python
optimizer = og.ocp.GeneratedOptimizer.load(
    "path/to/optimizer_manifest.json"
)

result = optimizer.solve(x0=[0.4, 0.2], xref=[0.0, 0.0])
```

This is useful when you want to reuse a previously generated solver across
Python sessions.
