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
//! you should rather check out the [documentation](https://alphaville.github.io/optimization-engine/).
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

use std::fmt;

/// Exceptions/Errors that may arise while solving a problem
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolverError {
    /// If the gradient or cost function cannot be evaluated
    Cost(&'static str),
    /// Computation failed and NaN/Infinite value was obtained
    NotFiniteComputation(&'static str),
    /// A projection could not be computed numerically
    ProjectionFailed(&'static str),
    /// A linear algebra operation failed
    LinearAlgebraFailure(&'static str),
    /// The solver reached an unexpected internal state
    InvalidProblemState(&'static str),
}

impl fmt::Display for SolverError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SolverError::Cost(reason) => {
                write!(f, "cost or gradient evaluation failed: {}", reason)
            }
            SolverError::NotFiniteComputation(reason) => {
                write!(f, "non-finite computation: {}", reason)
            }
            SolverError::ProjectionFailed(reason) => write!(f, "projection failed: {}", reason),
            SolverError::LinearAlgebraFailure(reason) => {
                write!(f, "linear algebra failure: {}", reason)
            }
            SolverError::InvalidProblemState(reason) => {
                write!(f, "invalid internal problem state: {}", reason)
            }
        }
    }
}

impl std::error::Error for SolverError {}

impl From<crate::matrix_operations::MatrixError> for SolverError {
    fn from(_: crate::matrix_operations::MatrixError) -> Self {
        SolverError::LinearAlgebraFailure("matrix operation failed")
    }
}

impl From<crate::cholesky_factorizer::CholeskyError> for SolverError {
    fn from(_: crate::cholesky_factorizer::CholeskyError) -> Self {
        SolverError::LinearAlgebraFailure("Cholesky factorization or solve failed")
    }
}

/// Result of a function call (status)
pub type FunctionCallResult = Result<(), SolverError>;

pub mod alm;
pub mod cholesky_factorizer;
pub mod constraints;
pub mod core;
pub mod lipschitz_estimator;
pub mod matrix_operations;
mod numeric;

pub use crate::cholesky_factorizer::{CholeskyError, CholeskyFactorizer};
pub use crate::core::fbs;
pub use crate::core::panoc;
pub use crate::core::{AlgorithmEngine, Optimizer, Problem};

/* Use Jemalloc if the feature `jem` is activated */
#[cfg(not(target_env = "msvc"))]
#[cfg(feature = "jem")]
use jemallocator::Jemalloc;

#[cfg(not(target_env = "msvc"))]
#[cfg(feature = "jem")]
#[global_allocator]
static JEMALLOC_GLOBAL: Jemalloc = Jemalloc;

#[cfg(all(feature = "rp", not(feature = "jem")))]
#[global_allocator]
static RPMALLOC_GLOBAL: rpmalloc::RpMalloc = rpmalloc::RpMalloc;

/* ---------------------------------------------------------------------------- */
/*          TESTS                                                               */
/* ---------------------------------------------------------------------------- */
#[cfg(test)]
mod mocks;

#[cfg(test)]
mod tests;
