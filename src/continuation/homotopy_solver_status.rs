/// Solver status of the homotopy method
///
#[derive(Debug)]
pub struct HomotopySolverStatus {
    has_converged: bool,
    num_outer_iterations: usize,
    num_inner_iterations: usize,
    last_problem_norm_fpr: f64,
}

// TODO: add: time
impl HomotopySolverStatus {
    pub fn new(
        has_converged: bool,
        num_outer_iterations: usize,
        num_inner_iterations: usize,
        last_problem_norm_fpr: f64,
    ) -> HomotopySolverStatus {
        HomotopySolverStatus {
            has_converged: has_converged,
            num_outer_iterations: num_outer_iterations,
            num_inner_iterations: num_inner_iterations,
            last_problem_norm_fpr: last_problem_norm_fpr,
        }
    }
}
