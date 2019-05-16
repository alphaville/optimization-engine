//! **Optimization Engine** is a framework for **fast** and **accurate** embedded nonconvex optimization.
//!
//! # About Optimization Engine
//!
//!
//! Its core functionality (including all numerical routines) is written in [Rust](https://www.rust-lang.org/).
//!
//! **Optimization Engine** can be used on PCs (all OSs are supported) and on embedded devices
//! (e.g., Raspberry Pi, Atom, Odroid, etc).
//!
//! Note that this is the **API documentation** of **Optimization Engine**; to get started,
//! you would rather check out the [documentation]().
//!

extern crate num;

pub mod constraints;
pub mod core;
pub mod lipschitz_estimator;
pub mod matrix_operations;

pub use crate::core::fbs;
pub use crate::core::panoc;
pub use crate::core::{AlgorithmEngine, Optimizer, Problem, SolverStatus};

/* ---------------------------------------------------------------------------- */
/*          TESTS                                                               */
/* ---------------------------------------------------------------------------- */
#[cfg(test)]
mod mocks;

#[cfg(test)]
mod tests;
