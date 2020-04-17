---
id: python-tcp-ip
title: TCP Sockets
description: How to use the TCP/IP API for OpEn
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" async src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.1/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>

As discussed [previously], one possible way of invoking an auto-generated parametric
optimizer is over a TCP/IP socket. 

[previously]: /optimization-engine/docs/python-interface#calling-the-optimizer

<img src="/optimization-engine/img/edge_intelligence.png" alt="Edge Intelligence Logo"/>

## The server

Provided that option `.with_tcp_interface_config()` has been activated, the code
generator creates a

```text
build_directory/
        |-- optimizer/
                |-- tcp_iface_{optimizer_name}/
```

We can start the server from Python using the [`OptimizerTcpManager`], but we can also start it by running `cargo run` from within `tcp_iface_{optimizer_name}` (where `{optimizer_name}` is the name of the optimizer). In order to see the log messages of the server, start it with

```shell
$ RUST_LOG=tcp_iface=info cargo run
```
We can then call the server from any programming language. Next, we will give examples
using a Linux terminal and the command `netcat` (or `nc`).

[`OptimizerTcpManager`]: /optimization-engine/docs/python-interface#calling-the-optimizer


## TCP/IP Socket interface

### Ping

In order to tell whether the server is alive and listening, we can "ping" it
by sending a simple JSON of the form `{ "Ping" : N }`, where `N` is any integer.

The server is expected to return `{ "Pong" : N }` with the same number.

Here is an example:

```
$  echo '{ "Ping" : 1 }' | nc localhost 4598
```

that will return 

```json
{
	"Pong" : 1
}
```


### Run

To call the optimizer for a given parameter `p` we run

```
echo '{"Run": {"parameter": [1.0, 10.0]} }' | nc localhost 4598
```

which will return the following JSON document

```json
{
    "exit_status": "Converged",
    "num_outer_iterations": 9,
    "num_inner_iterations": 85,
    "last_problem_norm_fpr": 8.879341428457282e-06,
    "delta_y_norm_over_c": 7.147511762156759e-06,
    "f2_norm": 0.0,
    "solve_time_ms": 13.569209,
    "penalty": 78125.0,
    "solution": [
        0.018786377508686856,
        0.028186552233630396,
        -0.10471801035932687,
        0.02921323766336347,
        0.0007963509453450717
    ],
    "lagrange_multipliers": [
        0.7699528316368849,
        14.491152879893193
    ]
}
```

On success, the response from the TCP server is a JSON document with the 
following fields:

| Response JSON Field       | Explanation                                 |
|---------------------------|---------------------------------------------|
| `exit_status`             | Exit status; can be (i) `Converged` or (ii) `NotConvergedIterations`, if the maximum number of iterations was reached, therefore, the algorithm did not converge up to the specified tolerances, or (iii) `NotConvergedOutOfTime`, if the solver did not have enough time to converge |
| `num_outer_iterations`    | Number of outer iterations   |
| `num_inner_iterations`    | Total number of inner iterations (for all inner problems)    |
| `last_problem_norm_fpr`   | Norm of the fixed-point residual of the last inner problem; this is a measure of the solution quality of the inner problem      |
| `delta_y_norm_over_c`     | Euclidean norm of $c^{-1}(y^+-y)$, which is equal to the distance between $F_1(u, p)$ and $C$ at the solution   |
| `f2_norm`                 | Euclidean norm of $F_2(u, p)$ at the solution|
| `solve_time_ms`           | Total execution time in milliseconds |
| `penalty`                 | Last value of the penalty parameter |
| `solution`                | Solution | 
| `lagrange_multipliers`    | Vector of Lagrange multipliers (if $n_1 > 0$) or an empty vector, otherwise | 

If we call the solver again, it will use the previous solution as an initial 
guess. The client may override this behaviour and provide a different initial
guess:

```
$ echo '{ "Run" : {"parameter" : [1.0,10.0], \
                   "initial_guess" : [0.0, 5.0, ...]}}' \
| nc localhost 4598
```

### Kill

To kill the server, just send the following request

```
$  echo '{ "Kill" : 1 }' | nc localhost 4598
```


### Error reporting

In case a request cannot be processed, e.g., because the provided JSON is malformed, the provided vectors have incompatible dimensions, the TCP server will return to the client an error report. This is a JSON with three attributes: (i) a key-value pair `"type": "Error"`, to allow the client to tell that an error has occurred, (ii) a `code`, which can be used to uniquely identify the type of error and (iii) a `message`, which offers some human-readable details.

For example, if the client provides an incompatible number of parameters, that is, if vector `parameter` is of the wrong length, then the server will return the following error:

```json
{
	"type": "Error", 
	"code": 3003, 
	"message": "wrong number of parameters"
}
```

The following errors may be returned to the client

| Code      | Explanation                                 |
|-----------|---------------------------------------------|
| 1000      | Invalid request: Malformed or invalid JSON  |
| 1600      | Initial guess has incompatible dimensions   |
| 1700      | Wrong dimension of Langrange multipliers    |
| 2000      | Problem solution failed (solver error)      |
| 3003      | Vector `parameter` has wrong length         |

