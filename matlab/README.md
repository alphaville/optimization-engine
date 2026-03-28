# OpEn MATLAB API

This directory contains the MATLAB interface of **Optimization Engine (OpEn)**.

The current MATLAB API lives in [`matlab/api`](./api). It communicates with
optimizers generated in Python that expose a TCP server interface.

The legacy MATLAB code is preserved in [`matlab/legacy`](./legacy).

## Capabilities

The current MATLAB toolbox supports:

- Connecting to TCP-enabled optimizers generated in Python
- Calling standard parametric optimizers using a flat parameter vector
- Calling OCP-generated optimizers using named parameter blocks from
  `optimizer_manifest.json`
- Loading OCP manifests and, when available, automatically reading the TCP
  endpoint from the sibling `optimizer.yml`
- Sending `ping` and `kill` requests to the optimizer server
- Providing optional warm-start data through:
  - `InitialGuess`
  - `InitialLagrangeMultipliers`
  - `InitialPenalty`
- Returning normalized solver responses with an `ok` flag and solver
  diagnostics
- Returning stage-wise `inputs` for OCP optimizers and `states` for
  multiple-shooting OCP optimizers

The main entry points are:

- [`matlab/api/OpEnTcpOptimizer.m`](./api/OpEnTcpOptimizer.m)
- [`matlab/api/createOpEnTcpOptimizer.m`](./api/createOpEnTcpOptimizer.m)

## Getting Started

Add the MATLAB API folder to your path:

```matlab
addpath(fullfile(pwd, 'matlab', 'api'));
```

Make sure the target optimizer TCP server is already running.

## Simple Optimizers

### Connect to a parametric optimizer

Use a TCP port directly. The IP defaults to `127.0.0.1`.

```matlab
client = OpEnTcpOptimizer(3301);
pong = client.ping();
disp(pong.Pong);
```

You can also specify the endpoint explicitly:

```matlab
client = OpEnTcpOptimizer('127.0.0.1', 3301);
```

### Solve a parametric optimizer

For a standard parametric optimizer, pass the flat parameter vector:

```matlab
response = client.solve([2.0, 10.0]);

if response.ok
    disp(response.solution);
    disp(response.cost);
else
    error('OpEn:SolverError', '%s', response.message);
end
```

### Solve with warm-start information

```matlab
response1 = client.solve([2.0, 10.0]);

response2 = client.solve( ...
    [2.0, 10.0], ...
    'InitialGuess', response1.solution, ...
    'InitialLagrangeMultipliers', response1.lagrange_multipliers, ...
    'InitialPenalty', response1.penalty);
```

### Stop the server

```matlab
client.kill();
```

## OCP Optimizers

For OCP-generated optimizers, MATLAB uses **name-value pairs** to provide the
parameter blocks listed in `optimizer_manifest.json`.

### Load an OCP optimizer from its manifest

If `optimizer_manifest.json` and `optimizer.yml` are in the same generated
optimizer directory, the client can infer the TCP endpoint automatically:

```matlab
manifestPath = fullfile( ...
    pwd, ...
    'python', ...
    '.python_test_build_ocp', ...
    'ocp_single_tcp', ...
    'optimizer_manifest.json');

client = OpEnTcpOptimizer('ManifestPath', manifestPath);
disp(client.parameterNames());
```

You can also override the endpoint explicitly:

```matlab
client = OpEnTcpOptimizer(3391, 'ManifestPath', manifestPath);
```

### Solve a single-shooting OCP optimizer

The following example matches the OCP manifest in
`python/.python_test_build_ocp/ocp_single_tcp`:

```matlab
response = client.solve( ...
    'x0', [1.0, -1.0], ...
    'xref', [0.0, 0.0]);

if response.ok
    disp(response.solution);
    disp(response.inputs);
    disp(response.exit_status);
else
    error('OpEn:SolverError', '%s', response.message);
end
```

If the manifest defines default values for some parameters, you only need to
provide the required ones:

```matlab
manifestPath = fullfile( ...
    pwd, ...
    'python', ...
    '.python_test_build_ocp', ...
    'ocp_manifest_bindings', ...
    'optimizer_manifest.json');

client = OpEnTcpOptimizer('ManifestPath', manifestPath);
response = client.solve('x0', [1.0, 0.0]);
```

### Solve a multiple-shooting OCP optimizer

For multiple-shooting OCPs, the MATLAB client also returns the state
trajectory reconstructed from the manifest slices:

```matlab
manifestPath = fullfile( ...
    pwd, ...
    'python', ...
    '.python_test_build_ocp', ...
    'ocp_multiple_tcp', ...
    'optimizer_manifest.json');

client = OpEnTcpOptimizer('ManifestPath', manifestPath);

response = client.solve( ...
    'x0', [1.0, -1.0], ...
    'xref', [0.0, 0.0]);

disp(response.inputs);
disp(response.states);
```

### OCP warm-start example

Warm-start options can be combined with named OCP parameters:

```matlab
response1 = client.solve( ...
    'x0', [1.0, -1.0], ...
    'xref', [0.0, 0.0]);

response2 = client.solve( ...
    'x0', [1.0, -1.0], ...
    'xref', [0.0, 0.0], ...
    'InitialGuess', response1.solution, ...
    'InitialLagrangeMultipliers', response1.lagrange_multipliers, ...
    'InitialPenalty', response1.penalty);
```

## Notes

- The MATLAB API does not start the optimizer server; it connects to a server
  that is already running.
- For plain parametric optimizers, use `client.solve(p)`.
- For OCP optimizers, use `client.solve('name1', value1, 'name2', value2, ...)`.
- The helper function `createOpEnTcpOptimizer(...)` is a thin wrapper around
  `OpEnTcpOptimizer(...)`.
