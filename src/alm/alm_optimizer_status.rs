use crate::core::ExitStatus;
use num::Float;

/// Solution statistics for `AlmOptimizer`
///
/// This structure has no public fields and no public setter methods.  
/// The idea is that only Optimization Engine can create optimizer
/// `AlmOptimizerStatus` instances.
///
#[derive(Debug)]
pub struct AlmOptimizerStatus<T = f64>
where
    T: Float,
{
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
    last_problem_norm_fpr: T,
    /// Lagrange multipliers vector
    lagrange_multipliers: Option<Vec<T>>,
    /// Total solve time
    solve_time: std::time::Duration,
    /// Last value of penalty parameter
    penalty: T,
    /// A measure of infeasibility of constraints F1(u; p) in C
    delta_y_norm: T,
    /// Norm of F2 at the solution, which is a measure of infeasibility
    /// of constraints F2(u; p) = 0
    f2_norm: T,
    /// Value of cost function at optimal solution (optimal cost)
    cost: T,
}

impl<T: Float> AlmOptimizerStatus<T> {
    /// Constructor for instances of `AlmOptimizerStatus`
    ///
    /// This method is only accessibly within this crate.
    ///
    /// Clients can then use getter methods to access the status of the
    /// algorithm (get statistics about the solving procedure etc)
    ///
    /// # Arguments
    ///
    /// - `exit_status`: Exit status of the algorithm
    ///
    /// # Returns
    ///
    /// New instance of `AlmOptimizerStatus`. Use the setter methods below
    /// to specify the algorithm status details.
    ///
    /// # Panics
    ///
    /// Does not panic
    ///
    pub(crate) fn new(exit_status: ExitStatus) -> Self {
        AlmOptimizerStatus {
            exit_status,
            num_outer_iterations: 0,
            num_inner_iterations: 0,
            last_problem_norm_fpr: -T::one(),
            lagrange_multipliers: None,
            solve_time: std::time::Duration::from_nanos(0),
            penalty: T::zero(),
            delta_y_norm: T::zero(),
            f2_norm: T::zero(),
            cost: T::zero(),
        }
    }

    /// Setter method for the total solve time
    ///
    /// # Arguments
    ///
    /// - `duration`: total time duration to solve the problem
    ///
    /// # Panics
    ///
    /// Does not panic
    ///
    pub(crate) fn with_solve_time(mut self, duration: std::time::Duration) -> Self {
        self.solve_time = duration;
        self
    }

    /// Setter method for the number of outer ALM/PM-type iterations
    ///
    ///
    /// # Arguments
    ///
    /// - `outer_iters`: number of outer iterations
    ///
    ///
    /// # Panics
    ///
    /// Does not panic
    ///
    pub(crate) fn with_outer_iterations(mut self, outer_iters: usize) -> Self {
        self.num_outer_iterations = outer_iters;
        self
    }

    /// Setter method for the total number of inner iterations
    ///
    /// # Arguments
    ///
    /// - `outer_iters`: total inner iteration count
    ///
    /// # Panics
    ///
    /// Does not panic
    ///
    pub(crate) fn with_inner_iterations(mut self, inner_iters: usize) -> Self {
        self.num_inner_iterations = inner_iters;
        self
    }

    /// Setter method for the vector of Lagrange multipliers at the solution
    ///
    /// # Arguments
    ///
    /// - `lagrange_multipliers`: vector of Lagrange multipliers (which is copied
    ///   into an internal field of `AlmOptimizerStatus`)
    ///
    /// # Panics
    ///
    /// Does not panic; it is the responsibility of the caller to provide a vector of
    /// Lagrange multipliers of correct length
    ///
    pub(crate) fn with_lagrange_multipliers(mut self, lagrange_multipliers: &[T]) -> Self {
        self.lagrange_multipliers = Some(vec![]);
        if let Some(y) = &mut self.lagrange_multipliers {
            y.extend_from_slice(lagrange_multipliers);
        }
        self
    }

    /// Setter method for the penalty parameter
    ///
    ///
    /// # Panics
    ///
    /// The method panics if the provided penalty parameter is negative
    ///
    pub(crate) fn with_penalty(mut self, penalty: T) -> Self {
        assert!(
            penalty >= T::zero(),
            "the penalty parameter should not be negative"
        );
        self.penalty = penalty;
        self
    }

    /// Setter method for the norm of the fixed-point residual of the last
    /// solved inner optimization problem (solved with PANOC)
    ///
    /// # Panics
    ///
    /// The method panics if the provided norm of the fixed-point residual is
    /// negative
    ///
    pub(crate) fn with_last_problem_norm_fpr(mut self, last_problem_norm_fpr: T) -> Self {
        assert!(
            last_problem_norm_fpr >= T::zero(),
            "last_problem_norm_fpr should not be negative"
        );
        self.last_problem_norm_fpr = last_problem_norm_fpr;
        self
    }

    pub(crate) fn with_delta_y_norm(mut self, delta_y_norm: T) -> Self {
        assert!(
            delta_y_norm >= T::zero(),
            "delta_y_norm must be nonnegative"
        );
        self.delta_y_norm = delta_y_norm;
        self
    }

    pub(crate) fn with_f2_norm(mut self, f2_norm: T) -> Self {
        assert!(f2_norm >= T::zero(), "f2_norm must be nonnegative");
        self.f2_norm = f2_norm;
        self
    }

    pub(crate) fn with_cost(mut self, cost: T) -> Self {
        self.cost = cost;
        self
    }

    // -------------------------------------------------
    // Update Methods
    // -------------------------------------------------

    /// Update cost (to be used when the cost needs to be scaled as a result of preconditioning)
    pub fn update_cost(&mut self, new_cost: T) {
        self.cost = new_cost;
    }

    /// Update ALM infeasibility
    pub fn update_f1_infeasibility(&mut self, new_alm_infeasibility: T) {
        self.delta_y_norm = new_alm_infeasibility;
    }

    /// Update PM infeasibility
    pub fn update_f2_norm(&mut self, new_pm_infeasibility: T) {
        self.f2_norm = new_pm_infeasibility;
    }

    // -------------------------------------------------
    // Getter Methods
    // -------------------------------------------------

    /// Exit status of solver
    ///
    /// # Panics
    ///
    /// Does not panic
    pub fn exit_status(&self) -> ExitStatus {
        self.exit_status
    }

    /// Number of outer iterations
    ///
    /// # Panics
    ///
    /// Does not panic
    ///
    pub fn num_outer_iterations(&self) -> usize {
        self.num_outer_iterations
    }

    /// Total count of inner iterations performed by `PANOCOptimizer`
    ///
    /// # Panics
    ///
    /// Does not panic
    ///
    pub fn num_inner_iterations(&self) -> usize {
        self.num_inner_iterations
    }

    /// Vector of Lagrange multipliers at the solution
    ///
    /// The method returns a reference to an `Option<Vec<T>>` which contains
    /// the vector of Lagrange multipliers at the solution, or is `None` if
    /// the problem has no ALM-type constraints.
    ///
    /// # Panics
    ///
    /// Does not panic
    ///
    pub fn lagrange_multipliers(&self) -> &Option<Vec<T>> {
        &self.lagrange_multipliers
    }

    /// Norm of the fixed-point residual of the last inner problem
    ///
    /// # Panics
    ///
    /// Does not panic
    ///
    pub fn last_problem_norm_fpr(&self) -> T {
        self.last_problem_norm_fpr
    }

    /// Total time to solve the problem (runtime of method `AlmOptimizer.solve()`)
    ///
    /// # Panics
    ///
    /// Does not panic
    ///
    pub fn solve_time(&self) -> std::time::Duration {
        self.solve_time
    }

    /// Penalty parameter at the solution
    ///
    /// # Panics
    ///
    /// Does not panic
    pub fn penalty(&self) -> T {
        self.penalty
    }

    /// Norm of Delta y divided by max{c, 1} - measure of infeasibility
    pub fn delta_y_norm_over_c(&self) -> T {
        let c = self.penalty();
        self.delta_y_norm / if c < T::one() { T::one() } else { c }
    }

    /// Norm of F2(u) - measure of infeasibility of F2(u) = 0
    pub fn f2_norm(&self) -> T {
        self.f2_norm
    }

    /// Value of the cost function at the solution
    pub fn cost(&self) -> T {
        self.cost
    }
}
