# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).


<!-- ---------------------
      Unreleased
     --------------------- -->

## [Unreleased]

### Fixed

* Fixed `with_max_duration` in `PANOC` not following the builder pattern
* Fixed misplaced `.unwrap()` in the `HomotopyOptimizer`

### Added

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
[Unreleased]: https://github.com/alphaville/optimization-engine/compare/v0.4.0...master
[v0.4.0]: https://github.com/alphaville/optimization-engine/compare/v0.3.1...v0.4.0
[v0.3.1]: https://github.com/alphaville/optimization-engine/compare/v0.3.0...v0.3.1
[v0.3.0]: https://github.com/alphaville/optimization-engine/compare/v0.2.2...v0.3.0

<!-- Issues -->
[closes #24]: https://github.com/alphaville/optimization-engine/issues/24
[closes #25]: https://github.com/alphaville/optimization-engine/issues/25

<!-- Other -->
[travis]: https://travis-ci.org/alphaville/optimization-engine/builds/537155440
[Appveyor]: https://ci.appveyor.com/project/alphaville/optimization-engine
