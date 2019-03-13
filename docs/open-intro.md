---
id: open-intro
title: Optimization Engine
sidebar_label: Introduction
---



## About Optimization Engine (OpEn)

Embedded optimization is of great importance in a wide range of engineering applications. For example, **model predictive control** is becoming all the more popular in highly dynamical systems with sampling times of a few milliseconds. 

Relevant application domains include

- **fast robotic systems** which are often controlled by optimization-based methodologies,
- **autonomous vehicles** where the collision avoidance problem is stated as a nonconvex optimization problem,
- **autonomous UAVs** which have very tight runtime requirements

to name a few.

**Optimization Engine** (OpEn) is a framework that allows engineers to **design** and **embed** optimization-based control and monitoring modules on such autonomous highly dynamical systems.

The aim of **Optimization Engine** is to become a widely used software solution, which stands up to the high performance requirements of modern cyber-physical systems, runs **fast**, makes good use of the available computation **resources** (CPU, RAM), is **reliable** and has **fully predictable behavior**.

**Optimization Engine** is easy to use and fully documented. Users can either work with the engine directly (in Rust), or interface it via other languages such as Python and MATLAB.

Users can, for example, do their design entirely in Python and then use the **Optimization Engine Interface** to automatically generate Rust code (which can be used on an embedded device).



## What can OpEn do?
**Optimization Engine** can be embedded on simple hardware devices and provide autonomy and reliability while it boosts the system's performance. 

Here is a lab demonstration of Optimization Engine in action. 

<a href="http://www.youtube.com/watch?v=E4vCSJw97FQ" target="_blank">![Aerial Navigation Video](http://img.youtube.com/vi/E4vCSJw97FQ/0.jpg)</a>

In this video we demonstrate that **Optimization Engine** can solve an obstacle avoidance problem as fast as 20 times per second.


## The power of Rust

Rust is exactly the right language for making efficient and reliable software. It combines high-level abstractions with low-level authority on the system without the need for a garbage collector or complex memory management. 

Most importantly, Rust is memory-safe and thread-safe by design! If something can go wrong during runtime, the software won't compile!

Rust compiles into llvm instructions and can run on any device, including embedded devices (such as ARM-based platforms, e.g., Raspberry Pi).


## The power of PANOC
What makes OpEn so fast?

The typical approach for solving nonconvex optimization problems in real time is the use of *Sequential Quadratic Programming* (SQP). At every iteration, SQP approximates the given nonconvex problem by a Quadratic Program. This is its main drawback: it necessitates inner iterative procedures, which will perform poorly especially when the problem at hand is ill-conditioned. The same holds for *interior point methods* - they require heavyweight inner iteration procedures.

**OpEn** uses the proximal averaged Newton-type method (PANOC) which uses the same oracle as the projected gradient method, therefore, it involves only simple iterations. PANOC is a line-search method that combines forward-backward iterations with fast Newton-type steps over the *forward-backward envelope* - a real-valued continuous and exact merit function. 

This way, **OpEn** enables very fast convergence (up to *superlinear convergence*, under mild assumptions), while it features very simple iterations which involve access to first-order information of the cost function and low-cost linear algebra (only vector-vector operations).

The result is a simple, yet rapidly convergent algorithm, which is perfectly suitable for embedded applications.

Find out more about PANOC in the [original publication](https://arxiv.org/pdf/1709.06487.pdf). See PANOC in action in obstacle avoidance scenarios in [this paper](https://core.ac.uk/download/pdf/153430972.pdf) and [this paper](https://arxiv.org/pdf/1812.04755.pdf).

## The structure of OpEn
**Optimization Engine** is a framework which comprises several components and layers.

All numerical routines are written in Rust, but users can design their optimizers in Python or MATLAB.

A code generation tool will create Rust code which can be used for maximum efficiency and can be uploaded on embedded devices.

**OpEn** comprises the following components:

- In Rust:
    - The `optimization-engine` crate
    - Crate `icasadi` which can be used to interface C code
- In MATLAB:
    - A code generation toolbox
    - An additional toolbox for MPC (and optimal control) applications
- In Python:
    - A code generation library
    - An additional library for MPC (and optimal control) applications


