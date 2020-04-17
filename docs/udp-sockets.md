---
id: udp-sockets
title: UDP Sockets
sidebar_label: UDP sockets
description: How to use the UDP interface of OpEn
---

## Consuming the module 

![UDP sockets](/optimization-engine/img/udp_socket.png)

### Communication Protocol
The client sends to the server a JSON file with the value of parameter `p` in the following format:

```json
{
	"parameter" : [1.0, 2.3]
}
```

The server solves the optimization problem and returns a JSON file in the following format:

```json
{
	"p" : [1.0, 2.3],
	"u" : [1.1479546337, 1.2921200844, 1.6371973229, 2.6535245176],
	"n" : 6,
	"f" : -7.032997779803717,
	"dt" : "422.958µs"
}
```

Here `p` is the parameter sent by the client, `u` is the solution, `n` is the number of iterations, `f` is the logarithm (base 10) of the fixed-point residual and `dt` is the elapsed time.

**NOTE:** Every time the solver is called, it is warm started with the previous solution.

### Errors
If the server receives a request it cannot process, it will return an error message. For example:

```json
{
	"parameter": [1.0, 22.0, 33.0]
}
```
Will return:

```json
{
	"error":"wrong param size (np=2, len(p)=3)"
}
```

### Killing the solver
To **kill** the process gracefully send the message `x`. The server will return to you the message:

```json
{
	"msg":"Received quit command"
}
```

and will exit immediately.


### Linux command line

```bash
netcat -u 127.0.0.1 3248
```
