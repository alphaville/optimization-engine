//! Status of the result of a solver (number of iterations, etc)
//!
//!
use std::time;

/// Solver status
///
/// This structure contais information about the solver status. Instances of
/// `SolverStatus` are returned by optimizers.
///
#[derive(Debug)]
pub struct SolverStatus {
    /// whether the algorithm has converged
    converged: bool,
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
        converged: bool,
        num_iter: usize,
        solve_time: time::Duration,
        fpr_norm: f64,
        cost_value: f64,
    ) -> SolverStatus {
        SolverStatus {
            converged,
            num_iter,
            solve_time,
            fpr_norm,
            cost_value,
        }
    }

    /// whether the algorithm has converged
    pub fn has_converged(&self) -> bool {
        self.converged
    }

    /// number of iterations taken by the algorithm
    pub fn iterations(&self) -> usize {
        self.num_iter
    }

    /// number of iterations taken by the algorithm
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
}
