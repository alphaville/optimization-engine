use crate::core::ExitStatus;
#[derive(Debug)]
pub struct AlmOptimizerStatus {
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
    lagrange_multipliers: Option<Vec<f64>>,
    /// Total solve time
    solve_time: std::time::Duration,
}

impl AlmOptimizerStatus {
    pub fn new(exit_status: ExitStatus) -> Self {
        AlmOptimizerStatus {
            exit_status,
            num_outer_iterations: 0,
            num_inner_iterations: 0,
            last_problem_norm_fpr: -1.0,
            lagrange_multipliers: None,
            solve_time: std::time::Duration::from_nanos(1),
        }
    }

    pub fn with_solve_time(mut self, duration: std::time::Duration) -> Self {
        self.solve_time = duration;
        self
    }

    pub fn with_outer_iterations(mut self, outer_iters: usize) -> Self {
        self.num_outer_iterations = outer_iters;
        self
    }

    pub fn with_inner_iterations(mut self, inner_iters: usize) -> Self {
        self.num_inner_iterations = inner_iters;
        self
    }

    pub fn with_lagrange_multipliers(mut self, lagrange_multipliers: &[f64]) -> Self {
        self.lagrange_multipliers = Some(vec![0.0; lagrange_multipliers.len()]);
        if let Some(y) = &mut self.lagrange_multipliers {
            y.copy_from_slice(&lagrange_multipliers);
        }
        self
    }
}
