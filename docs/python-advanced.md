---
id: python-advanced
title: Advanced options
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" async src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>



## Solver options
The user may wish to modify some additional solver parameters.

When using the penalty method to account for general constraints,
the most important parameters which determine the speed of 
convergence are the **initial values** of the penalty weights and the 
**update factor**. These are set using

```python
solver_config.with_penalty_weight_update_factor(8.0)       \
             .with_initial_penalty_weights([20.0, 5.0])
``` 

The number of the initial penalty weights must be equal to 
$n_c$; the number of constraints. If you need to set all 
weights to the same value, use

```python
solver_config.with_initial_penalty_weights(100.0)
```

In embedded applications it is often important that the solver
is forced to terminate within a given time period. The user may
set the **maximum runtime** (in microseconds) using

```python
# Maximum duration: 50ms
solver_config.with_max_duration_micros(50000)
```

Lastly, the maximum number of outer iterations can be set using

```python
solver_config.with_max_outer_iterations(num_out_iters)
```

The number of outer iterations should be kept reasonably small
to avoid too high values of the penalty parameters (which increase
exponentially).


## Build options

During the design phase, one needs to experiment with the problem
formulation and solver parameters. This is way the default build
mode is the "debug" mode, which compiles fast, but it suboptimal.
Building in "release" mode takes slightly longer to compile, but
can lead to a significant speed-up. To do so, use the option

```python
build_config.with_build_mode("debug")
```

*Note.* Coming soon: cross-compilation for different targets
(e.g., a Raspberry Pi).

 
## TCP/IP interface 

In order to change the IP and port at which the server listens
for requests (e.g., for remote connections), you may crate an 
`TcpServerConfiguration` object as follows

```python
tcp_config = og.config.TcpServerConfiguration('10.8.0.12', 9555)
```

and then provide it to the builder configuration using 

```python
builder_config.with_tcp_interface_config(tcp_config)
```
