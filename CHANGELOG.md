# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]

### Fixed

* Windows interoperability of `matlab_open_root()` [issue #24]

## [v0.3.0] - 2019-05-16

This is a breaking API change.

### Fixed

* A lot of internal fixes and clean up
* `PANOCEngine` and `FBSEngine` is no longer explicitly needed
* Simplified import system
* Cost functions now need to return a `Result<(), Error>` to indicate if the evaluation was successful

### Added

* Started an `examples` folder

[Unreleased]: https://github.com/alphaville/optimization-engine/compare/v0.3.0...HEAD
[v0.3.0]: https://github.com/alphaville/optimization-engine/compare/v0.3.0...v0.2.2
