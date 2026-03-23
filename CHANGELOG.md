# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

Note: This is the main Changelog file for the Rust solver. The Changelog file for the Python interface (`opengen`) can be found in [/open-codegen/CHANGELOG.md](open-codegen/CHANGELOG.md)


<!-- ---------------------
      v0.12.0
     --------------------- -->
## [v0.12.0] - Unreleased


### Changed

- Rust solver supports generic float types


<!-- ---------------------
      v0.11.1
     --------------------- -->
## [v0.11.1] - 2026-03-23 


### Fixed

- Return best PANOC half-step on early exit (issue #325)


<!-- ---------------------
      v0.11.0
     --------------------- -->
## [v0.11.0] - 2026-03-14

### Added 

- Implementation of `BallP` in Rust: projection on lp-ball

### Changed

- Algorithmic improvements in `EpigraphSquaredNorm` (numerically stable Newton refinement) and more detailed docs
- Assertion in `Ball1`, `Ball2`, and `BallInf` to check that that dimensions of `x` and `xc` are compatible (with unit test)
- Added validation in `Rectangle` and `Hyperplane` for invalid constructor inputs and strengthened dimension checks in hyperplane projection
- Added validation in `Sphere2` for empty inputs and incompatible center dimensions
- Added validation in `FiniteSet` for incompatible projection dimensions and corrected convexity detection for singleton sets
- Added unit tests for invalid `Rectangle`, `Simplex`, `Hyperplane`, `Sphere2`, and `FiniteSet` edge cases

### Fixed 

- Typos and doctest annotations in docs of `CartesianProduct` (in Rust), `Rectangle`, and `Hyperplane`, with more detailed documentation


<!-- ---------------------
      v0.10.0
     --------------------- -->
## [v0.10.0] - 2026-03-10

### Added

- Custom implementation of Cholesky factorisation (and solver); this is used in `AffineSpace` now.
- New function in `matrix_operations` to compute AA' given a matrix A

### Changed

- Update version of `ndarray`, in `Cargo.toml`
- Removed `modcholesky` because it was causing a bug (see issue #378)

<!-- ---------------------
      v0.9.0
     --------------------- -->
## [v0.9.1] - 2024-08-14


### Fixed

- Order of dependencies in `Cargo.toml` fixes crate-not-found issue on Windows



<!-- ---------------------
      v0.9.0
     --------------------- -->
## [v0.9.0] - 2024-03-20

### Added

- Rust implementation of epigraph of squared Euclidean norm (constraint) 
- Implementation of `AffineSpace`

### Fixed

- Clippy fixes

<!-- ---------------------
      v0.8.1
     --------------------- -->
## [v0.8.1] - 2023-10-27

### Fixed

- Fix bug in implementation of `ball2.rs` (radius was being ignored for balls centered not at the origin)



<!-- ---------------------
      v0.8.0
     --------------------- -->
## [v0.8.1] - 2023-10-27

### Added

- New constraint: sphere of Euclidean norm

<!-- ---------------------
      v0.7.7
     --------------------- -->
## [v0.7.7] - 2023-01-17

### Fixed

- Change `time::Instant` to `instant::Instant` to support WASM



<!-- ---------------------
      v0.7.6
     --------------------- -->
## [v0.7.6] - 2022-10-11

### Added

- Update functions in `AlmOptimizerStatus`



<!-- ---------------------
      v0.7.5
     --------------------- -->

## [v0.7.5] - 2022-06-22

### Fixed

- Fixed estimation of initial Lipschitz constant, `L`, when it is close to or equal to zero (e.g., Huber loss function)
- Fixed issue in `AlmFactory` related to (F2) penalty constraints


<!-- ---------------------
      v0.7.4
     --------------------- -->   
## [v0.7.4] - 2021-11-15

### Added 

- Optional feature `wasm` in `Cargo.toml` (WebAssembly support); see https://alphaville.github.io/optimization-engine/docs/openrust-features for details
- Using `instant::Instant` instead of `std::Instant` (Wasm-compatible)
- Fixed Rust documentation of `Ball1` 

<!-- ---------------------
      v0.7.3
     --------------------- -->     
## [v0.7.3] - 2021-11-1

### Added 

* Implementation of Simplex and Ball1 constraints in Rust
* Fix issue with simultaneous use of features `jem` and `rp`


<!-- ---------------------
      v0.7.2
     --------------------- -->
## [v0.7.2] - 2021-10-27

### Changed

* Removed unnecessary `#[no_mangle]` annotations
* Took care of additional clippy warnings
* Bump versions: `cbindgen`: `0.8 --> 0.20` and `libc`: `0.2.0 -> 0.2.*`

### Added

* Support for [`rpmalloc`](https://github.com/EmbarkStudios/rpmalloc-rs) and [`jemalloc`](https://github.com/gnzlbg/jemallocator) using the features `jem` and `rp`



<!-- ---------------------
      v0.7.1
     --------------------- -->

## [v0.7.1] - 2020-09-04

### Added

* Introduced `Halfspace` (implemented and tested)
* Introduced `Hyperplane` (implemented and tested)
* New types: `FunctionCallResult`, `MappingType` and `JacobianMappingType`
* Various clippy-related code improvements



## [v0.7.0] - 2020-05-04


### Added

* ALM: compute cost value at solution




## [v0.6.2] - 2019-10-29

### Fixed

* Bug in codegen for Cartesian products (PR #147)
* Removed the use of `Default` in Rust (does not work for large slices)
* Python: fixed typo in method `with_lfbgs_memory`

### Added

* New support for C-to-Rust interface via bindgen
* Generation of example C code for C-to-Rust interface
* CMakeLists for auto-generated example in C
* Additional Python examples on web page
* Chat button in web page (for gitter)
* Added option `local_path` in `with_open_version`

### Changed

* Homotopy module in Rust is annotated as deprecated
* TCP server response is cast into Python objects (PR #144)
* Auto-generated code links to most recent crate, unless overriden
* Changed `jacobian` to `gradient` in Python

## [v0.6.1-alpha.2] - 2019-09-7

### Fixed

* TCP server: Malformed error JSON is now fixed
* Algorithm now returns `u_bar`, which is feasible (not `u`)

### Added

* Introduced C interface to CasADi-generated C functions
* Rust and Python implementations of joint ALM/PM algorithms
* Rust docs for augmented Lagrangian method (ALM)
* Release of crate version `0.6.1-alpha.1` and `0.6.1-alpha.2`
* Introduced `#![allow(dead_code)]` in ALM implementation
* New AKKT-compliant termination criterion
* Tolerance relaxation in penalty method
* Finite sets supported in Rust
* Rust/Python: setting CBFGS parameters
* Second-order cones supported in Rust
* Rust docs: support for equations with KaTeX

### Changed

* Updated README


### Removed

* Support for Python <3.6 (deprecated)
* Module `continuation` is going to become obsolete

<!-- ---------------------
      v0.5.0
     --------------------- -->
## [v0.5.0] - 2019-06-22

### Fixed

* Fixed `with_max_duration` in `PANOC` not following the builder pattern
* Fixed misplaced `.unwrap()` in the `HomotopyOptimizer`
* Fixed so the Python builder uses the current directory as default

### Added

* Generation of C/C++ bindings added in the Python interface and included in the test suite
* Support in Rust for Cartesian product of constraints

### Removed

* Deprecated: `enable_tcp_interface` and `enable_c_bindings_generation`


<!-- ---------------------
      v0.4.0
     --------------------- -->
## [v0.4.0] - 2019-06-03

### Fixed

* Windows interoperability of `matlab_open_root()` [closes #24]
* Issues with file separator on Windows [#26 and #27]
* Handling corner cases such as wrong input parameters
* Rust: checking for `NaN` and `Inf` values in solution

### Added

* New Python interface for code generation (works with Python 2.7, 3.4 and 3.6)
* Homotopy method implemented in Rust
* TCP interface in Rust is generated automatically on request
* Support for OSX and linux distros on [travis] [closes #25]
* Continuous integration on [Appveyor]
* Experimental C bindings library
* Documentation for new Rust code and Python code
* Unit tests in Python using `unittest`

### Changed

* Rust API: Using `Option<>` and `Result<>` to handle errors
* Updated L-BFGS dependency; now using version `0.2` (no NonZeroUsize)

<!-- ---------------------
      v0.3.1
     --------------------- -->
## [v0.3.1] - 2019-05-21

### Fixed

* An error in the Matlab codegen which made it inoperable

### Added

* Support for compiling for different targets


<!-- ---------------------
      v0.3.0
     --------------------- -->
## [v0.3.0] - 2019-05-16

This is a breaking API change.

### Fixed

* A lot of internal fixes and clean up
* `PANOCEngine` and `FBSEngine` is no longer explicitly needed
* Simplified import system
* Cost functions now need to return a `Result<(), Error>` to indicate if the evaluation was successful

### Added

* Started an `examples` folder

<!-- ---------------------
      LINKS...
     --------------------- -->

<!-- Releases -->
[v0.11.1]: https://github.com/alphaville/optimization-engine/compare/v0.11.0...v0.11.1
[v0.11.0]: https://github.com/alphaville/optimization-engine/compare/v0.10.0...v0.11.0
[v0.10.0]: https://github.com/alphaville/optimization-engine/compare/v0.9.1...v0.10.0
[v0.9.1]: https://github.com/alphaville/optimization-engine/compare/v0.9.0...v0.9.1
[v0.9.0]: https://github.com/alphaville/optimization-engine/compare/v0.8.1...v0.9.0
[v0.8.1]: https://github.com/alphaville/optimization-engine/compare/v0.8.0...v0.8.1
[v0.8.0]: https://github.com/alphaville/optimization-engine/compare/v0.7.7...v0.8.0
[v0.7.7]: https://github.com/alphaville/optimization-engine/compare/v0.7.6...v0.7.7 
[v0.7.6]: https://github.com/alphaville/optimization-engine/compare/v0.7.5...v0.7.6 
[v0.7.5]: https://github.com/alphaville/optimization-engine/compare/v0.7.4...v0.7.5 
[v0.7.4]: https://github.com/alphaville/optimization-engine/compare/v0.7.3...v0.7.4
[v0.7.3]: https://github.com/alphaville/optimization-engine/compare/v0.7.2...v0.7.3
[v0.7.2]: https://github.com/alphaville/optimization-engine/compare/v0.7.1...v0.7.2
[v0.7.1]: https://github.com/alphaville/optimization-engine/compare/v0.7.0...v0.7.1
[v0.7.0]: https://github.com/alphaville/optimization-engine/compare/v0.6.2...v0.7.0
[v0.6.2]: https://github.com/alphaville/optimization-engine/compare/v0.6.1-alpha.2...v0.6.2
[v0.6.1-alpha.2]: https://github.com/alphaville/optimization-engine/compare/v0.5.0...v0.6.1-alpha.2
[v0.5.0]: https://github.com/alphaville/optimization-engine/compare/v0.4.0...v0.5.0
[v0.4.0]: https://github.com/alphaville/optimization-engine/compare/v0.3.1...v0.4.0
[v0.3.1]: https://github.com/alphaville/optimization-engine/compare/v0.3.0...v0.3.1
[v0.3.0]: https://github.com/alphaville/optimization-engine/compare/v0.2.2...v0.3.0

<!-- Issues -->
[closes #24]: https://github.com/alphaville/optimization-engine/issues/24
[closes #25]: https://github.com/alphaville/optimization-engine/issues/25

<!-- Other -->
[travis]: https://travis-ci.org/alphaville/optimization-engine/builds/537155440
[Appveyor]: https://ci.appveyor.com/project/alphaville/optimization-engine
