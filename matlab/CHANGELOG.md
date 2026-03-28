# Change Log 

All notable changes to the MATLAB interface will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

Note: This is the Changelog file for the MATLAB interface of OpEn.


## 0.1.0 - 31 March 2026

### Added

- New MATLAB TCP client in `matlab/api/OpEnTcpOptimizer.m` for TCP-enabled optimizers generated in Python.
- Convenience constructor helper `matlab/api/createOpEnTcpOptimizer.m`.
- Support for parametric optimizers over TCP using calls of the form `response = client.solve(p)`.
- Support for OCP optimizers over TCP by loading `optimizer_manifest.json` and allowing named-parameter calls such as `response = client.solve('x0', x0, 'xref', xref)`.
- Automatic packing of named OCP parameter blocks according to the manifest order, including support for manifest defaults.
- MATLAB-side helpers for `ping`, `kill`, warm-start options, and normalized solver responses.

### Changed

- Added a dedicated MATLAB API area under `matlab/api` for the current interface, separate from the legacy MATLAB code.
- Code generation in MATLAB is now moved to `matlab/legacy`
