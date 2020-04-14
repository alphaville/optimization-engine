---
id: contributing
sidebar_label: Contributing
title: Contributing to OpEn
description: How do I contribute to OpEn
---

## How can I contribute to OpEn?
Thank you for considering contributing to Optimization Engine (OpEn)!

OpEn is an open source project and welcomes contributions from the community.

You can contribute in several ways:

- Submit an [**issue**](https://github.com/alphaville/optimization-engine/issues): 
  Often bugs will go unnoticed by the core development team and certain 
  use cases and user needs will have evaded our attention. 
  Consider submitting an issue if:
  - You would like to report a [bug]; please, use the provided template for reporting 
    bugs. It is essential to give information about your system (OS, OpEn version)
    and outline a sequence of steps to reproduce the error. When possible, please
    provide a [minimum working example]
  - You would like to request a [new feature]; please use the provided template
  - You would like to propose modifications in OpEn's documentation, such as
    for some concepts to be better elucidated or a request for an additional example
- Share with us a **success story** on [**Discord**](https://discord.gg/mfYpn4V)
- Create a **pull request** (see below)

or, show us your love:

- Give us a [**star on gitub**](https://github.com/alphaville/optimization-engine)
- Spread the word on [**Twitter**]

![Star](https://media.giphy.com/media/ZxblqUVrPVmcqATkC4/giphy.gif)

## I just have a question!
The easiest and quickest way to ask a question is to reach us on [**Discord**](https://discord.gg/mfYpn4V) or [**Gitter**](https://gitter.im/alphaville/optimization-engine).

You may also consult the [**frequently asked questions**](/optimization-engine/docs/faq).


## Submitting issues
You may submit an issue regarding anything related to **OpEn**, such as:

- a bug
- insufficient/vague documentation
- request for a feature
- request for an example

You should, however, make sure that the same - or a very similar - issue is not already open. In that case, you may write a comment in an existing issue.


## Contributing code or docs

In order to contribute code or documentation, you need to [fork] our github repository, make you modifications and submit a pull request. You should follow these rules:

- create one or more [issues on github] that will be associated with your changes
- take it from `master`: fork OpEn and create a branch on `master`

```console
git checkout -b fix/xyz master
```

- read the [style guide](#coding-style-guide) below (and write unit/integration tests)
- create a pull request in which you need to explain the key changes

## Coding style guide

Things to keep in mind:

- **Code**: intuitive structure and variable names, short atomic functions, 
- **Comments**: help others better understand your code
- **Docs**: document all functions (even private ones)
- **Tests**: write comprehnsive, exhaustive tests

### Rust

*General guidelines:* Read the Rust [API guidelines] and this [API checklist]

*Naming convention:* We follow the [standard naming convention](https://rust-lang-nursery.github.io/api-guidelines/naming.html) of Rust.

*Documentation:* We follow [these guidelines](https://rust-lang-nursery.github.io/api-guidelines/documentation.html). Everything should be documented.

### Python

We follow [this style guide](https://www.python.org/dev/peps/pep-0008) and its [naming convention](https://www.python.org/dev/peps/pep-0008/#naming-conventions)


### Website
This documentation is generated with Docusaurus - read a detailed guide [here](https://github.com/alphaville/optimization-engine/blob/master/website/README.md).

- All docs are in `docs/`
- Blog entries are in `website/blog/`

To start the website locally (at [http://localhost:3000/optimization-engine](http://localhost:3000/optimization-engine)) change directory to `website` and run `yarn start`. To update the website, execute `./publish.sh` (you need to be a collaborator on github).

## Using Git
When using Git, keep in mind the following guidelines:

- Create simple, atomic, commits
- Write comprehensive commit messages
- Work on a forked repository
- When you're done, submit a pull request to 
[`alphaville/optimization-engine`](https://github.com/alphaville/optimization-engine/); 
it will be promptly delegated to a reviewer and we will contact you 
as soon as possible.

Branch `master` is protected and all pull requests need to be reviewed by a person 
other than their proposer before they can be merged into `master`.

## Versioning
This project consists of independent modules: 
(i) the core Rust library, 
(ii) the MATLAB interface, 
(iii) the Python interface. 
Each module has a different version number (`X.Y.Z`). 

We use the **SemVer** standard - we quote from [semver.org](https://semver.org/):

Given a version number `MAJOR.MINOR.PATCH`, increment the:

- `MAJOR` version when you make incompatible API changes,
- `MINOR` version when you add functionality in a backwards-compatible manner, and
- `PATCH` version when you make backwards-compatible bug fixes.

Additional labels for pre-release and build metadata are available as extensions to the `MAJOR.MINOR.PATCH` format.

We also keep a [log of changes](https://github.com/alphaville/optimization-engine/blob/master/CHANGELOG.md) where we summarize the main changes since last version.

## Releasing

Each time the major or minor number of the Rust library is updated, a new crate should be published on [crates.io](https://crates.io/crates/optimization_engine).

In order to release a new version make sure that 
you have done the following:

- Updated [CHANGELOG]
- Updated the version in (SemVer):
    - [CHANGELOG]
    - [Cargo.toml]
    - [setup.py]
- Resolved all associated issues on github (and you have created tests for these)
- Updated the documentation (Rust/Python API docs + website)
- Merged into master (your pull request has been approved)
- All tests pass on Travis CI and Appveyor
- Set `publish=true` in `Cargo.toml` (set it back to `false` for safety)
- Publish `opengen` on PyPI (if necessary)
    - before doing so, make sure that the cargo.toml template 
      points to the correct version of OpEn
- Changed "Unreleased" into the right version in [CHANGELOG] and created
  a release on github (example [release v0.4.0]) 
    
[CHANGELOG]: https://github.com/alphaville/optimization-engine/blob/master/CHANGELOG.md
[Cargo.toml]: https://github.com/alphaville/optimization-engine/blob/master/Cargo.toml    
[setup.py]: https://github.com/alphaville/optimization-engine/blob/master/open-codegen/setup.py
[release v0.4.0]: https://github.com/alphaville/optimization-engine/releases/tag/v0.4.0
[bug]: https://github.com/alphaville/optimization-engine/issues/new?template=bug_report.md
[issues on github]: https://github.com/alphaville/optimization-engine/issues
[**Twitter**]: https://twitter.com/intent/tweet?original_referer=https%3A%2F%2Falphaville.github.io%2Foptimization-engine&ref_src=twsrc%5Etfw&text=Fast%20and%20accurate%20embedded%20nonconvex%20optimization%20with%20%23OptimizationEngine&tw_p=tweetbutton&url=https%3A%2F%2Falphaville.github.io%2Foptimization-engine&via=isToxic
[minimum working example]: https://en.wikipedia.org/wiki/Minimal_working_example
[new feature]: https://github.com/alphaville/optimization-engine/issues/new?template=feature_request.md
[fork]: https://github.com/alphaville/optimization-engine
[API guidelines]: https://rust-lang-nursery.github.io/api-guidelines/about.html
[API checklist]: https://rust-lang-nursery.github.io/api-guidelines/checklist.html
