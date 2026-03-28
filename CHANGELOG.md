# Changelog

This file is the project-wide changelog for **Optimization Engine (OpEn)**.

It serves as the entrypoint for releases across the repository and points to the
component-specific changelogs, where the detailed changes for each module are
recorded.

## Component Changelogs

- [Rust library changelog](rust/CHANGELOG.md)
- [Python interface (`opengen`) changelog](python/CHANGELOG.md)
- [MATLAB interface changelog](matlab/CHANGELOG.md)
- [Docker image changelog](docker/CHANGELOG.md)

## How To Read This Changelog

- Use this file for a high-level overview of changes across the whole project.
- Use the component changelogs above for detailed release notes, migration
  notes, and module-specific fixes.
- Not every repository change appears here. Day-to-day implementation details
  usually live only in the relevant component changelog files.

## Versioning Notes

OpEn is a multi-component project, and not every part evolves on the same
version number or schedule.

- The Rust solver has its own release history in [`rust/CHANGELOG.md`](rust/CHANGELOG.md).
- The Python package `opengen` has its own release history in
  [`python/CHANGELOG.md`](python/CHANGELOG.md).
- The MATLAB interface and Docker image also track their changes separately.

When a release affects multiple parts of the repository, this root changelog can
be used to summarize the release and point readers to the detailed component
entries.

## Current Development Snapshot

At the time of writing, the main ongoing release tracks include:

- Rust library: `0.12.0`
- Python interface (`opengen`): `0.11.0`
- MATLAB interface: `0.1.0`
- Docker image: `0.7.0`

Please consult the linked component changelogs for the exact release state of
each module.

## Repository Layout

The main components live in:

- [rust/](rust/)
- [python/](python/)
- [matlab/](matlab/)
- [docker/](docker/)
- [docs/](docs/)

This changelog is intentionally lightweight, so it can remain a stable landing
page even as individual components evolve independently.
