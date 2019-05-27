---
id: contributing
sidebar_label: Contributing
title: Contributing to OpEn
---

## Contributing 
Thank you for considering contributing to Optimization Engine (OpEn)!

OpEn is an open source project and welcomes contributions from the community.

You can contribute in several ways:

- Submit an [**issue**](https://github.com/alphaville/optimization-engine/issues) (feature request or bug report)
- Share with us a **success story** on [**Discord**](https://discord.gg/mfYpn4V)
- Create a **pull request** (see below)

or, show us your love:

- Give us a [**star on gitub**](https://github.com/alphaville/optimization-engine)
- Spread the word on [**Twitter**](https://twitter.com/intent/tweet?original_referer=http%3A%2F%2Flocalhost%3A3000%2Foptimization-engine%2Fblog%2F2019%2F03%2F06%2Ftalk-to-us&ref_src=twsrc%5Etfw&text=Fast%20and%20accurate%20embedded%20nonconvex%20optimization%20with%20%23OptimizationEngine&tw_p=tweetbutton&url=http%3A%2F%2Flocalhost%3A3000%2Foptimization-engine%2Fblog%2F2019%2F03%2F06%2Ftalk-to-us&via=isToxic)

![Star](https://media.giphy.com/media/ZxblqUVrPVmcqATkC4/giphy.gif)

## I just have a question!
The easiest and quickest way to ask a question is to reach us on [**Discord**](https://discord.gg/mfYpn4V) or [**Gitter**](https://gitter.im/alphaville/optimization-engine).

You may also consult the [**frequently asked questions**](http://localhost:3000/optimization-engine/docs/faq).


## Submitting issues
You may submit an issue regarding anything related to **OpEn**, such as:

- a bug
- insufficient/vague documentation
- request for a feature
- request for an example

You should, however, make sure that the same - or a very similar - issue is not already open. In that case, you may write a comment in an existing issue.


## Contributing code or docs

In order to contribute code or documentation, you need to [fork]() our github repository, make you modifications and submit a pull request. You should follow these rules:

- create one or more [issues on github](https://github.com/alphaville/optimization-engine/issues) that will be associated with your changes
- take it from `master`: fork OpEn and create a branch on `master`
- read the [style guide](#coding-style-guide) below (and write tests)
- create a pull request

## Coding style guide

Things to keep in mind:

- **Code**: intuitive structure and variable names, short atomic functions, 
- **Comments**: help others better understand your code
- **Docs**: document all functions (even private ones)
- **Tests**: write comprehnsive, exhaustive tests

### Rust

*General guidelines:* Read [this](https://rust-lang-nursery.github.io/api-guidelines/about.html)

*Naming convention:* We follow the [standard naming convention](https://rust-lang-nursery.github.io/api-guidelines/naming.html) of Rust.

*Documentation:* We follow [these guidelines](https://rust-lang-nursery.github.io/api-guidelines/documentation.html). Everything should be documented.

### Python

We follow [this style guide](https://www.python.org/dev/peps/pep-0008) and its [naming convention](https://www.python.org/dev/peps/pep-0008/#naming-conventions)

### MATLAB



### Website
This documentation is generated with Docusaurus - read a detailed guide [here](https://github.com/alphaville/optimization-engine/blob/master/website/README.md).

- All docs are in `docs/`
- Blog entries are in `website/blog/`

To start the website locally (at [http://localhost:3000](http://localhost:3000)) change directory to `website` and run `yarn start`. To update the website, execute `./publish.sh` (you need to be a collaborator on github).

## Using Git
When using Git, keep in mind the following guidelines:

- Create simple, atomic, commits
- Write comprehensive commit messages
- Work on a forked repository
- When you're done, submit a pull request to `alphaville/optimization-engine`; we'll review it asap

## Versioning
This project consists of independent modules: (i) the core Rust library, (ii) the MATLAB interface, (iii) the Python interface. Each module has a different version number (`vX.Y.Z`). There's only one rule that connects them: the first and second version numbers of all modules must be equal at all times. 

We use the **SemVer** standard - we quote from [semver.org](https://semver.org/):

Given a version number `MAJOR.MINOR.PATCH`, increment the:

- `MAJOR` version when you make incompatible API changes,
- `MINOR` version when you add functionality in a backwards-compatible manner, and
- `PATCH` version when you make backwards-compatible bug fixes.

Additional labels for pre-release and build metadata are available as extensions to the `MAJOR.MINOR.PATCH` format.

We also keep a [log of changes](https://github.com/alphaville/optimization-engine/blob/master/CHANGELOG.md) where we summarize the main changes since last version.

## Crates.io

Each time the major or minor number of the Rust library is updated, a new crate should be published on [crates.io](https://crates.io/crates/optimization_engine).

In order to publish a new version on `crates.io` make sure that:

- You can updated CHANGELOG
- You have updated the version (SemVer)
- You have resolved all associated issues on github (and you have created tests for these)
- You have merged into master (your pull request has been approved)
- You have updated the documentation
- All tests pass
- You have set `publish=true` in `Cargo.toml` (set it back to `false` for safety)