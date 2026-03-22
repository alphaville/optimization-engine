# Changelog

All notable changes to the Docker image in this directory will be documented in this file.

The format is loosely based on Keep a Changelog and tracks changes to the Dockerfile,
bundled notebooks, and image-facing documentation.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [0.7.0] - 2026-03-22

### Added

- Added a Rust Jupyter notebook at `docker/notebooks/openrust_basic.ipynb` based on the
  OpEn basic Rust example.
- Added a Python Jupyter notebook at `docker/notebooks/python_ocp_1.ipynb` based on the
  getting-started optimal control example.
- Added Matplotlib to the image so bundled Python notebooks can plot trajectories.
- Added Rust kernel support in JupyterLab through `evcxr_jupyter`.

### Changed

- Updated the image to bundle Python 3.12 from `python:3.12-slim-bookworm`.
- Switched the interactive environment from the classic Jupyter notebook UI to JupyterLab.
- Moved the example notebook into `docker/notebooks/`.
- Updated `docker/README.md` and `docs/docker.md` to describe the current image layout,
  authentication model, kernels, notebook locations, and build flow.
- Documented the Dockerfile/image version separately from the bundled Python package
  version.

### Security

- Enabled Jupyter's default token authentication by default instead of disabling auth.
- Upgraded `pip` in both the system Python and virtual environment during the image build.
- Removed the transient `py` package from the virtual environment after installing
  `opengen`.

## [0.6.0] - 2026-03-21

### Changed

- Updated the Docker image version from `0.5.0` to `0.6.0`
- Updated Python notebook 

## [0.5.0] - 2026-03-21

### Added

- Added maintainer-facing build and publish instructions in `docker/README.md`.

### Changed

- Defined Dockerfile/image version `0.5.0`.
- Bundled `opengen==0.10.0` in the image while keeping the Docker image version distinct.
- Reworked notebook packaging so bundled notebooks are copied from `docker/notebooks/`.

