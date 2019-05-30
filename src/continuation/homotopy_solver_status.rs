use crate::core::ExitStatus;
/// Solver status of the homotopy method
///
#[derive(Debug)]
pub struct HomotopySolverStatus {
    /// Exit status
    exit_status: ExitStatus,
    /// Number of outer iterations
    num_outer_iterations: usize,
    /// Total number of inner iterations
    ///
    /// This is the sum of the numbers of iterations of
    /// inner solvers
    num_inner_iterations: usize,
    /// Norm of the fixed-point residual of the the problem
    last_problem_norm_fpr: f64,
    /// Maximum constraint violation
    max_constraint_violation: f64,
    /// Total solve time
    solve_time: std::time::Duration,
}

impl HomotopySolverStatus {
    /// Construct a new instance of `HomotopySolverStatus`
    ///
    /// ## Arguments
    /// - exit_status: exit status
    /// - num_outer_iterations: number of outer iterations
    /// - num_inner_iterations: number of inner iterations
    /// - last_problem_norm_fpr: norm of the FPR of the last solved inner problem
    /// - max_constraint_violation: maximum constraint violation
    /// - solve_time: total solve time
    ///
    /// ## Returns
    ///
    /// New instance of `HomotopySolverStatus`
    pub fn new(
        exit_status: ExitStatus,
        num_outer_iterations: usize,
        num_inner_iterations: usize,
        last_problem_norm_fpr: f64,
        max_constraint_violation: f64,
        solve_time: std::time::Duration,
    ) -> HomotopySolverStatus {
        HomotopySolverStatus {
            exit_status: exit_status,
            num_outer_iterations: num_outer_iterations,
            num_inner_iterations: num_inner_iterations,
            last_problem_norm_fpr: last_problem_norm_fpr,
            max_constraint_violation: max_constraint_violation,
            solve_time: solve_time,
        }
    }

    /// exit status of solver
    pub fn exit_status(&self) -> ExitStatus {
        self.exit_status
    }
}
