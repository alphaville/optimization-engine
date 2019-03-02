//! Status of the result of a solver (number of iterations, etc)
//!
//!
pub use crate::core::SolverStatus;

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
    pub fn new(converged: bool, num_iter: usize, fpr_norm: f64, cost_value: f64) -> SolverStatus {
        SolverStatus {
            converged: converged,
            num_iter: num_iter,
            fpr_norm: fpr_norm,
            cost_value: cost_value,
        }
    }

    /// whether the algorithm has converged
    pub fn has_converged(&self) -> bool {
        self.converged
    }

    /// number of iterations taken by the algorithm
    pub fn get_number_iterations(&self) -> usize {
        self.num_iter
    }

    /// norm of the fixed point residual
    pub fn get_norm_fpr(&self) -> f64 {
        self.fpr_norm
    }

    /// value of the cost at the solution
    pub fn get_cost_value(&self) -> f64 {
        self.cost_value
    }
}
