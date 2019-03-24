# Optimization Engine

[![Build Status](https://travis-ci.org/alphaville/optimization-engine.svg?branch=master)](https://travis-ci.org/alphaville/optimization-engine) [![MIT license](https://img.shields.io/badge/License-MIT-blue.svg)](https://lbesson.mit-license.org/) [![Apache v2 license](https://img.shields.io/badge/License-Apache%20v2-blue.svg)](https://github.com/alphaville/optimization-engine/blob/master/LICENSE-APACHE) [![Gitter](https://badges.gitter.im/alphaville/optimization-engine.svg)](https://gitter.im/alphaville/optimization-engine?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge) [![Chat on Discord](https://img.shields.io/badge/chat-on%20discord-gold.svg)](https://discord.gg/mfYpn4V) [![Chat on IRC](https://img.shields.io/badge/chat-on%20irc-pink.svg)](https://webchat.freenode.net/?channels=%23optimization_engine)

![OpEn logo](https://pbs.twimg.com/media/D1d8iOuXQAUFKJT.png:large)


Optimization Engine (OpEn) is a solver for embedded nonconvex optimization. 

**Documentation available at** [**alphaville.github.io/optimization-engine**](https://alphaville.github.io/optimization-engine/)

## Features

- Fast nonconvex optimization
- Numerical algorithm written in Rust
- Provably safe memory management
- Ideal for nonlinear MPC applications (e.g., autonomous navigation)


## Demos

Code generation? Piece of cake!

**OpEn** generates parametric optimizer modules in Rust - it's blazingly fast - it's safe - it can run on embedded devices

![Code generation](website/static/img/115ba54c2ad0.gif "demo1")

OpEn can run on embedded devices; here we see it running on an intel Atom for the autonomous navigation of a lab-scale micro aerial vehicle - the controller runs at **20Hz**!

![Autonomous Aerial Vehicle](website/static/img/e8f236af8d38.gif "demo-mav")

and here we see it in action for the autonomous navigation and obstacle avoidance on an Odroid XU4 platform

![Autonomous Ground Vehicle](website/static/img/6f6ea4f8d194.gif "demo-agv")


## License

This project is licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.


## Authors
- Pantelis Sopasakis
- Emil Fresk

## Contributions

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the
work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.
