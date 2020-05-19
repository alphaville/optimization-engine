#![deny(missing_docs)]
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
//! # Optimization Problems
//!
//! Optimization Engine solves optimization problems of the general form
//!
//! $$\begin{aligned}
//! \mathrm{Minimize}\  f(u)
//! \\\\
//! u \in U
//! \\\\
//! F_1(u) \in C
//! \\\\
//! F_2(u) = 0
//! \end{aligned}$$
//!
//! where
//!
//! - $u\in\mathbb{R}^{n_u}$ is the decision variable,
//! - $f:\mathbb{R}^n\to\mathbb{R}$ is a $C^{1,1}$-smooth cost function,
//! - $U$ is a (not necessarily convex) closed subset of $\mathbb{R}^{n_u}$
//!   on which we can easily compute projections (e.g., a rectangle, a ball,
//!   a second-order cone, a finite set, etc),
//! - $F_1:\mathbb{R}^{n_u}\to\mathbb{R}^{n_1}$ and $F_2:\mathbb{R}^{n_u} \to\mathbb{R}^{n_2}$
//!   are mappings with smooth partial derivatives, and
//! - $C\subseteq\mathbb{R}^{n_1}$ is a convex closed set on which we can easily compute projections.
//!

extern crate num;

/// Exceptions/Errors that may arise while solving a problem
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolverError {
    /// If the gradient or cost function cannot be evaluated
    Cost,
    /// Computation failed and NaN/Infinite value was obtained
    NotFiniteComputation,
}

/// Result of a function call (status)
pub type FunctionCallResult = Result<(), SolverError>;

pub mod alm;
pub mod constraints;
pub mod core;
pub mod lipschitz_estimator;
pub mod matrix_operations;

pub use crate::core::fbs;
pub use crate::core::panoc;
pub use crate::core::{AlgorithmEngine, Optimizer, Problem};

/* ---------------------------------------------------------------------------- */
/*          TESTS                                                               */
/* ---------------------------------------------------------------------------- */
#[cfg(test)]
mod mocks;

#[cfg(test)]
mod tests;
