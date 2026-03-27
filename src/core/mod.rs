#![deny(missing_docs)]
//! Optimisation algorithms
//!
//!

use num::Float;

pub mod fbs;
pub mod panoc;
pub mod problem;
pub mod solver_status;

pub use crate::{constraints, FunctionCallResult, SolverError};
pub use problem::Problem;
pub use solver_status::SolverStatus;

/// Exit status of an algorithm (not algorithm specific)
///
///
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExitStatus {
    /// The algorithm has converged
    ///
    /// All termination criteria are satisfied and the algorithm
    /// converged within the available time and number of iterations
    Converged,
    /// Failed to converge because the maximum number of iterations was reached
    NotConvergedIterations,
    /// Failed to converge because the maximum execution time was reached
    NotConvergedOutOfTime,
}

/// A general optimizer
pub trait Optimizer<T = f64>
where
    T: Float,
{
    /// Solves a given problem and updates the initial estimate `u` with the solution.
    ///
    /// Returns the solver status on success.
    ///
    /// If the algorithm cannot proceed because a user callback fails, a
    /// projection fails, non-finite values are encountered, or an internal
    /// numerical/kernel inconsistency is detected, this method returns
    /// `Err(SolverError)`.
    ///
    fn solve(&mut self, u: &mut [T]) -> Result<SolverStatus<T>, SolverError>;
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
pub trait AlgorithmEngine<T = f64> {
    /// Take a step of the algorithm and return `Ok(true)` only if the iterations should continue.
    ///
    /// Returns `Err(SolverError)` if a callback or projection fails, if a
    /// non-finite value is produced, or if the engine detects an invalid
    /// numerical state.
    fn step(&mut self, u: &mut [T]) -> Result<bool, SolverError>;

    /// Initializes the algorithm.
    ///
    /// Returns `Err(SolverError)` if initialization requires evaluating a
    /// callback/projection and that operation fails.
    fn init(&mut self, u: &mut [T]) -> FunctionCallResult;
}
