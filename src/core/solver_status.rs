//! Status of the result of a solver (number of iterations, etc)
//!
//!
use crate::core::ExitStatus;
use std::time;

/// Solver status
///
/// This structure contais information about the solver status. Instances of
/// `SolverStatus` are returned by optimizers.
///
#[derive(Debug, PartialEq, Copy, Clone)]
pub struct SolverStatus {
    /// exit status of the algorithm
    exit_status: ExitStatus,
    /// number of iterations for convergence
    num_iter: usize,
    /// time it took to solve
    solve_time: time::Duration,
    /// norm of the fixed-point residual (FPR)
    fpr_norm: f64,
    /// cost value at the candidate solution
    cost_value: f64,
}

impl SolverStatus {
    /// Constructs a new instance of SolverStatus
    ///
    /// ## Arguments
    ///
    /// - `converged` whether the algorithm has converged to a solution up to
    ///   the specified tolerance
    /// - `num_iter` number of iterations
    /// - `fpr_norm` norm of the fixed-point residual; a gauge of the solution
    ///    quality
    /// - `cost_value` the value of the cost function at the solution
    ///
    pub fn new(
        exit_status: ExitStatus,
        num_iter: usize,
        solve_time: time::Duration,
        fpr_norm: f64,
        cost_value: f64,
    ) -> SolverStatus {
        SolverStatus {
            exit_status,
            num_iter,
            solve_time,
            fpr_norm,
            cost_value,
        }
    }

    /// whether the algorithm has converged
    pub fn has_converged(&self) -> bool {
        self.exit_status == ExitStatus::Converged
    }

    /// number of iterations taken by the algorithm
    pub fn iterations(&self) -> usize {
        self.num_iter
    }

    /// total execution time
    pub fn solve_time(&self) -> time::Duration {
        self.solve_time
    }

    /// norm of the fixed point residual
    pub fn norm_fpr(&self) -> f64 {
        self.fpr_norm
    }

    /// value of the cost at the solution
    pub fn cost_value(&self) -> f64 {
        self.cost_value
    }

    /// exit status of solver
    pub fn exit_status(&self) -> ExitStatus {
        self.exit_status
    }
}
