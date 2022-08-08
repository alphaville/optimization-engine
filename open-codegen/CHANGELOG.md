# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

Note: This is the Changelog file of `opengen` - the Python interface of OpEn

## [0.6.12] - 2022-8-8

### Fixed 

* Got rid of Rust warnings for generated code


## [0.6.11] - 2022-3-10

### Fixed

* Changed f-strings (`f"{variable}"`) to `.format` for python3.5 compatibility
* Fixed typo in `tcp_server.rs`

## [0.6.9] - 2022-1-24

### Fixed

* Forced use of clap v2 because migration required for v3

## [0.6.8] - 2021-12-3

### Fixed

* Fixed bug in python code file generation for Windows users

## [0.6.7] - 2021-11-1

### Added

* Support for simplices and ell1 balls via code generation


## [0.6.6] - 2021-10-27

### Added

* Support for [`rpmalloc`](https://github.com/EmbarkStudios/rpmalloc-rs) and [`jemalloc`](https://github.com/gnzlbg/jemallocator) using `BuildConfiguration.with_allocator`
* Implementation/testing of projection on simplices (`simplex.py`) (addresses #234)
* Implementation/testing of projection on Ball-1 (`ball1.py`) (addresses #238)
* `# Safety` section in auto-generated unsafe auto-generated Rust code


### Removed 

* Unnecessary `#[no_mangle]`s in auto-generated Rust code

### Changed

* Took care of most clippy warnings in Rust and auto-generated Rust code


## [0.6.5] - 2021-04-16

### Changed

* Include `VERSION` file in `MANIFEST.in` (included in Python package)
* Bump versions: `cbindgen`: `0.8 --> 0.20.*` and `libc`: `0.2.0 -> 0.2.*` 


## [0.6.4] - 2020-10-21

### Added

* Accessing Rust from Python directly using PyO3 

### Fixed

* List of authors in `Cargo.toml` is generated properly
* Fixed bug when curvature is zero

### Changed

* `enable_tcp_interface` previously gave a `DeprecationWarning`; now it raises it. In a future version, it will be removed. 


## [0.6.3] - 2020-10-06

### Fixed

* Fixed bug #210: Cartesian products in code generation
* Fixed bug #211: `OptimizerTcpManager` and remote connections


## [0.6.2] - 2020-09-27

### Fixed

* Fixed issue with TCP connections in Release mode (PR #206)


## [0.6.1] - 2020-09-10

### Changed

* `OptimizerTcpManager`: ip and port can be set dynamically


## [0.6.0] - 2020-09-03

### Added 

* Support for half-spaces in problem constraints
* Added checks for `segments` in `CartesianProduct`

### Changed

* Dropping first argument in `Cartesian` (`dimension`) as it is unnecessary
* Documented class `CartesianProduct`
* Dropped `dimension` from constructor of `CartesianProduct` (breaking change)

### Fixed

* Issue #185: ROS config parameters are ignored


## [0.5.0] - 2020-05-12

### Fixed

* Prevent multiple log messages from printing
* Check if TCP port is available before starting server 

### Added

* Auto-generation of ROS packages
* Cost value at solution is returned

### Changed

* Reorganised template files into folders

### Removed


## [0.4.1] - 2020-04-13

### Fixed

* Project-specific `icasadi` dependency
* Project-specific `tcp_iface` TCP interface
* Fixed `lbfgs` typo

[0.6.12]: https://github.com/alphaville/optimization-engine/compare/opengen-0.6.11...opengen-0.6.12
[0.6.11]: https://github.com/alphaville/optimization-engine/compare/opengen-0.6.9...opengen-0.6.11
[0.6.9]: https://github.com/alphaville/optimization-engine/compare/opengen-0.6.8...opengen-0.6.9
[0.6.8]: https://github.com/alphaville/optimization-engine/compare/opengen-0.6.7...opengen-0.6.8
[0.6.7]: https://github.com/alphaville/optimization-engine/compare/opengen-0.6.6...opengen-0.6.7
[0.6.6]: https://github.com/alphaville/optimization-engine/compare/opengen-0.6.5...opengen-0.6.6
[0.6.5]: https://github.com/alphaville/optimization-engine/compare/opengen-0.6.4...opengen-0.6.5
[0.6.4]: https://github.com/alphaville/optimization-engine/compare/opengen-0.6.3...opengen-0.6.4
[0.6.3]: https://github.com/alphaville/optimization-engine/compare/opengen-0.6.2...opengen-0.6.3
[0.6.2]: https://github.com/alphaville/optimization-engine/compare/opengen-0.6.1...opengen-0.6.2
[0.6.1]: https://github.com/alphaville/optimization-engine/compare/opengen-v0.6.0...opengen-0.6.1
[0.6.0]: https://github.com/alphaville/optimization-engine/compare/opengen-v0.5.0...opengen-0.6.0
[0.5.0]: https://github.com/alphaville/optimization-engine/compare/opengen-0.4.1...opengen-v0.5.0
[0.4.1]: https://github.com/alphaville/optimization-engine/compare/opengen-0.4.1...master
