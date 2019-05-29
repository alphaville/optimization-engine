//! Optimisation algorithms
//!
//!

pub mod exit_status;
pub mod fbs;
pub mod panoc;
pub mod problem;
pub mod solver_status;

pub use crate::{constraints, Error};
pub use exit_status::ExitStatus;
pub use problem::Problem;
pub use solver_status::SolverStatus;

/// A general optimizer
pub trait Optimizer {
    /// solves a given problem and updates the initial estimate `u` with the solution
    ///
    /// Returns the solver status
    ///
    fn solve(&mut self, u: &mut [f64]) -> SolverStatus;
}

/// Engine supporting an algorithm
///
/// An engine is responsible for the allocation of memory for an algorithm,
/// especially memory that is reuasble is multiple instances of the same
/// algorithm (as in model predictive control).
///
/// It defines what the algorithm does at every step (see `step`) and whether
/// the specified termination criterion is satisfied
///
pub trait AlgorithmEngine {
    /// Take a step of the algorithm and return `Ok(true)` only if the iterations should continue
    fn step(&mut self, u: &mut [f64]) -> Result<bool, Error>;

    /// Initializes the algorithm
    fn init(&mut self, u: &mut [f64]) -> Result<(), Error>;
}
