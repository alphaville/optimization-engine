---
id: python-advanced
title: Advanced options
description: Advanced options of opengen, OpEn's Python interface
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" async src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.1/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>



## Solver options
The user may wish to modify some additional solver parameters.

When using the penalty method to account for general constraints,
the most important parameters which determine the speed of 
convergence are the **initial value** of the penalty weight and the 
**update factor**. These are set using

```python
solver_config.with_penalty_weight_update_factor(8.0)       \
             .with_initial_penalty(20.0)
``` 

The number of the initial penalty weights must be equal to 
$n_c$; the number of constraints. If you need to set all 
weights to the same value, use

```python
solver_config.with_initial_penalty(100.0)
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

<div class="alert alert-info">
<b>Preconditioning:</b> Opengen since version <b>0.7.0</b> supports a preconditioning mechanism. If activated,  
the initial penalty is computed internally. Moreover, appropriate scaling is applied on the cost function, 
the PM-type constraints, $F_2(u; p) = 0$, and the ALM-type constraints, $F_1(u; p) \in C$, but only if $C=\{0\}$, or 
if $C$ is a quadrant, e.g., $C = \{x: x\leq 0\}$.<br/><br/>

Preconditioning seems to improve the convergence speed and robustness of the solver based 
on some initial benchmarks, however, for the time being it is not active by default.
To activate it, use `with_preconditioning(True)`.
</div>


A complete list of solver options is given in the following table

| Method                                 | Explanation                                 |
|----------------------------------------|---------------------------------------------|
| `with_tolerance`                       | Target $\epsilon$-tolerance                 |
| `with_delta_tolerance`                 | Tolerance $\delta$                          |
| `with_initial_penalty`                 | Initial penalty value, $c_0$                |
| `with_max_inner_iterations`            | Maximum number of outer iterations          |
| `with_max_outer_iterations`            | Maximum number of inner iterations          |
| `with_penalty_weight_update_factor`    | Update factor of penalty weight, $\rho$     |
| `with_initial_tolerance`               | Initial tolerance, $\epsilon_0$             |
| `with_sufficient_decrease_coefficient` | Sufficient decrease coeff. (for skipping penalty update) |
| `with_max_duration_micros`             | Maximum duration of execution in micros     |
| `with_cbfgs_parameters`                | CBFGS parameters                            |
| `with_lbfgs_memory`                    | LBFGS memory                                | 
| `with_inner_tolerance_update_factor`   | Update factor for the inner tolerance       | 
| `with_preconditioning`                 | Whether preconditioning should be applied   |

## Build options

### Build mode

During the design phase, one needs to experiment with the problem
formulation and solver parameters. This is way the default build
mode is the "debug" mode, which compiles fast, but it suboptimal.
Building in "release" mode takes slightly longer to compile, but
can lead to a significant speed-up. To do so, use the option

```python
build_config.with_build_mode(
    og.config.BuildConfiguration.DEBUG_MODE)
```

οr 

```python
build_config.with_build_mode(
    og.config.BuildConfiguration.RELEASE_MODE)
```

### Cross-compilation 

You can either compile for your own system, or cross-compile for a 
different target system. For example, to cross-compile for a **Raspberry Pi**,
set the following option

```python
build_config.with_target_system("arm-unknown-linux-gnueabihf")
```

or 

```python
build_config.with_target_system("rpi")  # Raspberry Pi
```

Note that you need to install the necessary target first. 

<details>
<summary><b>See setup details</b></summary>
To cross-compile for a Raspberry Pi you need to run the following in your terminal

```bash
rustup target add arm-unknown-linux-gnueabihf
```

You also need to install the following dependencies

<!--DOCUSAURUS_CODE_TABS-->

<!--Linux-->
```bash
sudo apt-get update
sudo apt-get install -y gcc-arm-linux-gnueabihf libc6-dev-armhf-cross
```

<!--MacOS-->
```bash
# Tap the repository that provides the cross-compiler
brew tap messense/macos-cross-toolchains
# Update brew to ensure the tap is recognized (can sometimes be needed)
brew update 
# Install the full toolchain (includes gcc, binutils, sysroot)
# This specific formula provides the entire toolchain.
brew install arm-unknown-linux-gnueabihf 

# Verify the compiler is found
which arm-linux-gnueabihf-gcc || (echo "arm-linux-gnueabihf-gcc not found in PATH" && exit 1)
```
<!--END_DOCUSAURUS_CODE_TABS-->
</details>

<br>
If you need to compile for a target other than `arm-linux-gnueabihf-gcc` (`rpi`)
some manual configuration may be needed (you may need to install the target 
and/or a compiler/linker) and you may need to edit the auto-generated 
`.cargo/config.toml` files you will find in your auto-generated solvers. 

<details>
<summary><b>Non-supported targets</b></summary>
The auto-generated `.cargo/config.toml` files contain entries like

```toml
[target.arm-unknown-linux-gnueabihf]
linker="arm-linux-gnueabihf-gcc"
```

Here you may have to insert manually your own target. 
Feel free to open an [issue](https://github.com/alphaville/optimization-engine/issues) 
on GitHub if you would like us to add support for a particular target (create a feature
request); see the [contributing guidelines](https://alphaville.github.io/optimization-engine/docs/contributing).
</details>


When cross-compiling for a Raspberry Pi you may want to configure a TCP server
so you can call the optimizer remotely. You can find more information about this 
[below](#tcpip-interface). 
Once you have cross-compiled, locate the file
```text
{your_optimizer}/tcp_iface_{your_optimizer}/target/arm-unknown-linux-gnueabihf/release/tcp_iface_{your_optimizer}
```
—where `{your_optimizer}` is the name of your optimizer—and copy it to your Raspberry Pi.
On your Raspberry, change the permissions so you can execute this file
```bash
chmod u+x ./tcp_iface_{your_optimizer}
```
and [run it](https://alphaville.github.io/optimization-engine/docs/python-tcp-ip). Your OpEn server is live. 
Read also the [documentation](https://alphaville.github.io/optimization-engine/docs/python-tcp-ip) 
on the TCP sockets protocol of OpEn servers.

### Other build options

All build options are shown below

| Method                        | Explanation                                 |
|-------------------------------|---------------------------------------------|
| `with_build_directory`        | Target build directory; the default is `.`  |
| `with_build_mode`             | `release` or `debug`; the default option is `release`, which requires more time to compile, but leads to high performance executables; use `debug` for faster compilation, at the cost of lower performance (this is useful when experimenting with OpEn)  |
| `with_tcp_interface_config`   | Enable TCP server; provide configuration    |
| `with_target_system`          | Target system (to be used when you need to cross-compile) |
| `with_build_c_bindings`       | Enalbe generation of C/C++ bindings         |
| `with_rebuild`                | Whether to do a clean build                 |
| `with_open_version`           | Use a certain version of OpEn (see [all versions]), e.g., `with_open_version("0.6.0")`, or a local version of OpEn (this is useful when you want to download the latest version of OpEn from github). You can do so using `with_open_version(local_path="/path/to/open/")`. |
|`with_allocator`               | Available in `opengen >= 0.6.6`. Compile with a different memory allocator. The available allocators are the entries of `RustAllocator`. OpEn currently supports [Jemalloc](https://github.com/gnzlbg/jemallocator) and [Rpmalloc](https://github.com/EmbarkStudios/rpmalloc-rs).|

[all versions]: https://crates.io/crates/optimization_engine/versions

## TCP/IP interface 

### Generation of TCP server

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

### Communicate with TCP server

There are two ways to connect to a generated TCP server and call the
auto-generated optimizer:

<div class="alert alert-info">
<b>Connect to a local optimizer</b> by providing the path of the optimizer
directory. For that purpose, we need to create an instance of 
<code>OptimizerTcpManager</code> and specify the path to the auto-generated optimizer.
</div>

For example,

```python
mng = og.tcp.OptimizerTcpManager('python_build/the_optimizer')
```

we can then `start` the optimizer. The TCP manager known what IP and port 
to link to, so we can `call` it directly.

<div class="alert alert-info">
<b>Connect to a local optimizer</b> and <b>customize</b> its IP and port. 
This is particularly useful if you need to start multiple instances of a TCP 
server (with different ports).
</div>

For example,

```python
ip = '0.0.0.0'
port = 5678
mng = og.tcp.OptimizerTcpManager('python_build/the_optimizer', ip, port)
```

<div class="alert alert-info">
<b>Connect to a remote optimizer</b> by providing its IP and port. In that 
case we assume that an optimizer is up an running at some remote address
and listens for connections at a certain port. In that case, we cannot 
<code>start</code> the optimizer remotely using the TCP manager. 
</div>



For example to connect to a *remote* TCP server at `10.8.0.7:5678`, we can
create a TCP manager as follows:

```python
mng = og.tcp.OptimizerTcpManager(ip="10.8.0.7", port=5678)
```

A list of the methods of `OptimizerTcpManager` is given below:

| Method          | Explanation                                 |
|-----------------|---------------------------------------------|
| `start`         | Starts the (local) server; throws an exception if the IP is not local. If the server is local, it will start in a separate thread and will remain alive unit `kill` is invoked. |
| `ping`          | Pings the server to check if it is alive    |
| `call`          | Calls the TCP server; provides a parameter and the solver responds with either the solution or an error report |
| `kill`                | Kills the server associated with the TCP manager; works both on local and remote servers. |


## Metadata

The solver metadata offer important information about the auto-generated 
solver such as its name


| Method                        | Explanation                                 |
|-------------------------------|---------------------------------------------|
| `with_optimizer_name`         | Optimizer name  (which is also the name of the folder in which the auto-generated solver will be stored). The default optimizer name is `open_optimizer` |
| `with_authors`                | Provide list of author names                |
| `with_licence`                | Name or URL of license of generated solver; the default licence is the MIT licence  |
| `with_version`                | Version of auto-generated solver; the default version is `0.0.0` |
