---
id: python-tcp-ip
title: TCP/IP Socket Interface
---

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
                |-- tcp_iface/
```

We can start the server from Python using the [`OptimizerTcpManager`], but we can 
also start it by running `cargo run` from within `tcp_iface`. In order to see the 
log messages of the server, start it with

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
  "num_outer_iterations": 1,
  "num_inner_iterations": 15,
  "last_problem_norm_fpr": 0.000053258819596141074,
  "max_constraint_violation": 0.0,
  "solve_time_ms": 1.016792,
  "solution": [
    0.9790942942294071,
    0.955734452549382,
    0.9113348829947056,
    0.8292339534219697,
    0.6804702314295598
  ]
}
```

If we call the solver again, it will use the previous solution as an initial 
guess. The client may override this behaviour and provide a different initial
guess:

```
echo '{"Run":{"parameter":[1.0,10.0],"initial_guess":[0.0,5.0,...]}}'\
| nc localhost 4598
```

### Kill

To kill the server, just send the following request

```
$  echo '{ "Kill" : 1 }' | nc localhost 4598
```


### Error reporting

In case a request cannot be processed, e.g., because the provided JSON is malformed, the provided vectors have incompatible dimensions, the TCP server will return to the client an error report. This is a JSON with three attributes: (i) a key-value pair `"type": "Error"`, to allow the client to tell that an error has occured, (ii) a `code`, which can be used to uniquely identify the type of error and (iii) a `message`, which offers some human-readable details.

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
| 2000      | Problem solution failed (solver error)      |
| 3003      | Vector `parameter` has wrong length         |

