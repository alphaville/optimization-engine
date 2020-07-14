# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

Note: This is the Changelog file of `opengen` - the Python interface of OpEn

## [0.6.0]

### Changed

* Dropping first argument in `Cartesian` (`dimension`) as it is unnecessary

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


[0.6.0]: https://github.com/alphaville/optimization-engine/compare/opengen-0.6.0...opengen-0.5.0
[0.5.0]: https://github.com/alphaville/optimization-engine/compare/opengen-0.5.0...opengen-0.4.1
[0.4.1]: https://github.com/alphaville/optimization-engine/compare/opengen-0.4.1...master
