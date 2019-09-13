use crate::{
    constraints,
    continuation::{
        homotopy_problem::HomotopyProblem, ContinuationMode, HomotopyCache, HomotopySolverStatus,
    },
    core::{panoc, ExitStatus, Optimizer, Problem},
    matrix_operations, SolverError,
};

const DEFAULT_CONSTRAINT_TOLERANCE: f64 = 1e-4;
const DEFAULT_MAX_OUTER_ITERATIONS: usize = 10;
const DEFAULT_MAX_INNER_ITERATIONS: usize = 500;

/// Homotopy optimizer
///
/// This struct solves a parametric optimization problem - which has been defined using
/// HomotopyProblem - and solves it with the homotopy method.
pub struct HomotopyOptimizer<
    'a,
    ParametricPenaltyFunctionType,
    ParametricGradientType,
    ConstraintType,
    ParametricCostType,
> where
    ParametricPenaltyFunctionType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricGradientType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricCostType: Fn(&[f64], &[f64], &mut f64) -> Result<(), SolverError>,
    ConstraintType: constraints::Constraint,
{
    /// Definition of parametric problem
    homotopy_problem: HomotopyProblem<
        ParametricPenaltyFunctionType,
        ParametricGradientType,
        ConstraintType,
        ParametricCostType,
    >,
    homotopy_cache: &'a mut HomotopyCache,
    /// Tolerance on constraint satisfaction
    constraint_tolerance: f64,
    /// Maximum number of outer iterations
    max_outer_iterations: usize,
    /// Maximum number of inner iterations
    max_inner_iterations: usize,
    /// Maximum duration
    max_duration: Option<std::time::Duration>,
    /// Initial tolerance
    ///
    /// We start with a certain inner tolerance for the inner prolems
    /// and we decrease it until we reach the target tolerance, which
    /// is provided within `homotopy_cache`; must be positive
    initial_tolerance: f64,
    /// Update factor for inner tolerance
    ///
    /// We multiply the inner tolerance by this scalar until we reach
    /// the target inner tolerance, which is provided within `homotopy_cache`.
    /// This must be a scalar in (0,1)
    inner_tolerance_update_factor: f64,
}

impl<
        'a,
        ParametricPenaltyFunctionType,
        ParametricGradientType,
        ConstraintType,
        ParametricCostType,
    >
    HomotopyOptimizer<
        'a,
        ParametricPenaltyFunctionType,
        ParametricGradientType,
        ConstraintType,
        ParametricCostType,
    >
where
    ParametricPenaltyFunctionType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricGradientType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricCostType: Fn(&[f64], &[f64], &mut f64) -> Result<(), SolverError>,
    ConstraintType: constraints::Constraint,
{
    /// Constructor for `HomotopyOptimizer`
    ///
    ///
    /// ## Arguments
    /// - homotopy_problem: definition of homotopy problem
    /// - panoc_cache: re-useable instance of `PANOCCache`
    ///
    /// ## Returns
    ///
    /// - New instance of `HomotopyOptimizer`
    ///
    /// ## Panics
    ///
    /// Does not panic
    ///
    /// ## Example
    ///
    /// ```
    /// use optimization_engine::{
    ///     SolverError,
    ///     continuation, constraints::*,
    ///     core::panoc::PANOCCache
    ///    };
    ///
    /// fn main() {
    ///     let n = 2;
    ///     let lbfgs_mem = 10;
    ///     let mut panoc_cache = PANOCCache::new(n, 1e-5, lbfgs_mem);
    ///     let mut homotopy_cache = continuation::HomotopyCache::new(panoc_cache);
    ///
    ///     /* cost function, f(u; q) */
    ///     let cost_fun = |u: &[f64], q: &[f64], cost: &mut f64| -> Result<(), SolverError> {
    ///         // your implementation goes here
    ///         Ok(())
    ///     };
    ///
    ///     /* parametric gradient, df(u, q) */
    ///     let grad_fun = |u: &[f64], q: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
    ///         // your implementation goes here
    ///             Ok(())
    ///         };
    ///
    ///     /* penalty-type constraints: c(u; p) */
    ///     let penalty_constr_fun =
    ///     |u: &[f64], q: &[f64], constraints: &mut [f64]| -> Result<(), SolverError> {
    ///        // your implementation goes here
    ///        Ok(())
    ///     };
    ///
    ///     // Constraints...
    ///     let bounds = Ball2::new(None, 1.5);
    ///
    ///     // Define homotopy problem
    ///     let  homotopy_problem = continuation::HomotopyProblem::new(
    ///        bounds,
    ///        grad_fun,
    ///        cost_fun,
    ///        penalty_constr_fun,
    ///        1
    ///     );
    ///
    ///     let mut homotopy_optimizer =
    ///         continuation::HomotopyOptimizer::new(homotopy_problem, &mut homotopy_cache);
    /// }
    /// ```
    ///
    pub fn new(
        homotopy_problem: HomotopyProblem<
            ParametricPenaltyFunctionType,
            ParametricGradientType,
            ConstraintType,
            ParametricCostType,
        >,
        homotopy_cache: &'a mut HomotopyCache,
    ) -> HomotopyOptimizer<
        'a,
        ParametricPenaltyFunctionType,
        ParametricGradientType,
        ConstraintType,
        ParametricCostType,
    > {
        let hom_cache = &homotopy_cache;
        let target_tolerance = hom_cache.panoc_cache.tolerance;
        HomotopyOptimizer {
            homotopy_problem,
            homotopy_cache,
            constraint_tolerance: DEFAULT_CONSTRAINT_TOLERANCE,
            max_outer_iterations: DEFAULT_MAX_OUTER_ITERATIONS,
            max_inner_iterations: DEFAULT_MAX_INNER_ITERATIONS,
            max_duration: None,
            initial_tolerance: target_tolerance,
            inner_tolerance_update_factor: 0.1,
        }
    }

    fn initialize_param(&self, q_augmented_params: &mut [f64]) {
        let cache = &self.homotopy_cache;
        let idx_list = &cache.idx;
        let from_list = &cache.from;
        for (i, from_value) in idx_list.iter().zip(from_list.iter()) {
            q_augmented_params[*i] = *from_value;
        }
    }

    /// Specify the initial inner tolerance
    ///
    /// ## Arguments
    ///
    /// - init_inner_tolerance: initial value of the inner tolerance
    ///   (must be positive)
    ///
    /// ## Panics
    ///
    /// If `init_inner_tolerance` is nonpositive or if it is lower than the target
    /// tolerance
    ///
    pub fn with_initial_inner_tolerance(mut self, init_inner_tolerance: f64) -> Self {
        assert!(
            init_inner_tolerance > 0.0,
            "init_inner_tolerance must be positive"
        );
        assert!(
            init_inner_tolerance >= self.homotopy_cache.panoc_cache.tolerance,
            "init_inner_tolerance must be at least equal to the target tolerance"
        );
        self.initial_tolerance = init_inner_tolerance;
        self
    }

    /// Specify the inner tolerance update factor
    ///
    /// The tolerance of the inner solver is initialised using
    /// `with_initial_inner_tolerance` and at every outer iteration
    /// it is updated by multiplying with this factor.
    ///
    /// ## Arguments
    ///
    /// -  inner_tolerance_update_factor: update factor of inner tolerance
    ///    (must be in (0,1))
    ///
    /// ## Panics
    ///
    /// If `inner_tolerance_update_factor` is nonpositive of larger than
    /// or equal to 1.0.
    ///
    pub fn with_inner_tolerance_update_factor(
        mut self,
        inner_tolerance_update_factor: f64,
    ) -> Self {
        assert!(
            inner_tolerance_update_factor < 1.0 && inner_tolerance_update_factor > 0.0,
            "inner tol update factor must be a number in (0,1)"
        );
        self.inner_tolerance_update_factor = inner_tolerance_update_factor;
        self
    }

    /// Specify tolerance on constraint violation
    ///
    /// ## Arguments
    ///
    /// - constraint_tolerance: tolerance
    ///
    /// ## Returns
    ///
    /// The current mutable instance of `HomotopyOptimizer`
    ///
    /// ## Panics
    ///
    /// Panics if the provided constraint tolerance is nonpositive
    ///
    pub fn with_constraint_tolerance(mut self, constraint_tolerance: f64) -> Self {
        assert!(
            constraint_tolerance > 0.0,
            "constraint_tolerance must be positive"
        );
        self.constraint_tolerance = constraint_tolerance;
        self
    }

    /// Specify maximum number of outer iterations
    ///
    /// ## Arguments
    ///
    /// - max_outer_iterations: maximum number of iterations of outer solvers
    ///
    /// ## Returns
    ///
    /// The current mutable instance of `HomotopyOptimizer`
    ///
    pub fn with_max_outer_iterations(mut self, max_outer_iterations: usize) -> Self {
        self.max_outer_iterations = max_outer_iterations;
        self
    }

    /// Specify the maximum number of iterations of each inner solver
    ///
    /// ## Arguments
    ///
    /// - max_inner_iterations: maximum number of iterations of inner solvers
    ///
    /// ## Returns
    ///
    /// The current mutable instance of `HomotopyOptimizer`
    ///
    pub fn with_max_inner_iterations(mut self, max_inner_iterations: usize) -> Self {
        self.max_inner_iterations = max_inner_iterations;
        self
    }

    /// Specify the maximum time duration for the homotopy solver
    ///
    /// If the maximum duration of execution is exceeded, the solver will
    /// return the status code `ExitStatus::NotConvergedOutOfTime`; see
    /// [`#solve`](#method.solve)
    ///
    /// ## Arguments
    ///
    /// - max_duration: maximum duration as an instance of `std::time::Duration`
    ///
    /// ## Returns
    ///
    /// - A reference to the current mutable instance of `HomotopyOptimizer`
    ///
    /// ## Panics
    ///
    /// Does not panic.
    pub fn with_max_duration(mut self, max_duration: std::time::Duration) -> Self {
        self.max_duration = Some(max_duration);
        self
    }

    // TODO: return a status code (target_reached_flag); this way we will know
    // whether to continue iterating
    //
    // TODO: Now all parameters are updated; maybe update only those which
    // violate the termination conditions (c)
    fn update_continuation_parameters(&self, p_: &mut [f64]) {
        let cache = &self.homotopy_cache;
        let idx_list = &cache.idx;
        let transition_list = &cache.transition_mode;
        for (i, transition_mode) in idx_list.iter().zip(transition_list.iter()) {
            match transition_mode {
                ContinuationMode::Arithmetic(s) => p_[*i] += s,
                ContinuationMode::Geometric(s) => p_[*i] *= s,
                _ => (),
            }
        }
    }

    /// Solve problem by homotopy method
    ///
    /// ## Arguments:
    /// - `u` - on entry, this is the initial guess of the solution, on exit,
    ///   it gets updated with the approximate solution that the solver computed
    ///   (check also the exit status)
    /// - `q_augmented_param` - vector of parameters of the parametric problem
    ///
    /// ## Returns
    ///
    /// - A `Result`, which, if successful contains an instance of `HomotopySolverStatus`,
    ///   otherwise, it returns an appropriate error
    ///
    /// ## Panic
    ///
    /// To be updated
    pub fn solve(
        &'a mut self,
        u: &mut [f64],
        q_augmented_param: &[f64],
    ) -> Result<HomotopySolverStatus, SolverError> {
        let now = std::time::Instant::now();
        let mut q_augmented_param_vec: Vec<f64> = q_augmented_param.to_vec();
        self.initialize_param(&mut q_augmented_param_vec);

        // Another consideration is the total time; the time should be monitored
        // and every next instance of the (inner) solver should be given the
        // time that is left
        let mut last_norm_fpr: f64 = 1.;
        let mut num_outer_iterations = 0;
        let mut num_inner_iterations = 0;
        let num_penalty_constraints = self.homotopy_problem.num_penalty_constraints;
        let mut constraint_values: Vec<f64> = vec![0.0; std::cmp::max(1, num_penalty_constraints)];
        let mut available_time_left = self.max_duration;
        let mut exit_status = ExitStatus::Converged;
        let mut inner_tolerance = self.initial_tolerance;
        let target_inner_tolerance = self.homotopy_cache.panoc_cache.tolerance;

        // OUTER ITERATIONS
        for _iter_outer in 1..=self.max_outer_iterations {
            // Check for available time...
            if let Some(max_duration) = self.max_duration {
                available_time_left = max_duration.checked_sub(now.elapsed());
                if None == available_time_left {
                    // no time left!
                    exit_status = ExitStatus::NotConvergedOutOfTime;
                    break;
                }
            }

            self.homotopy_cache.panoc_cache.tolerance = inner_tolerance;

            num_outer_iterations += 1;
            let homotopy_problem = &self.homotopy_problem;
            let f_ = |u: &[f64], cost: &mut f64| -> Result<(), SolverError> {
                (homotopy_problem.parametric_cost)(u, &q_augmented_param_vec, cost)
            };
            let df_ = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
                (homotopy_problem.parametric_gradient)(u, &q_augmented_param_vec, grad)
            };
            let inner_problem = Problem::new(&self.homotopy_problem.constraints, df_, f_);

            let mut inner_panoc = if available_time_left != None {
                panoc::PANOCOptimizer::new(inner_problem, &mut self.homotopy_cache.panoc_cache)
                    .with_max_iter(self.max_inner_iterations)
                    .with_max_duration(available_time_left.unwrap())
            } else {
                panoc::PANOCOptimizer::new(inner_problem, &mut self.homotopy_cache.panoc_cache)
                    .with_max_iter(self.max_inner_iterations)
            };

            //TODO: If inner problem does not converge, check whether it is salvageable
            let status = inner_panoc.solve(u)?;
            num_inner_iterations += status.iterations();
            last_norm_fpr = status.norm_fpr();
            exit_status = status.exit_status();
            (homotopy_problem.penalty_function)(u, &q_augmented_param_vec, &mut constraint_values)?;
            let continue_outer_iterations = constraint_values
                .iter()
                .any(|&ci| ci.abs() > self.constraint_tolerance);
            if !continue_outer_iterations {
                break;
            } else {
                self.update_continuation_parameters(&mut q_augmented_param_vec);
                exit_status = ExitStatus::NotConvergedIterations;
            }

            // Update the current inner tolerance (until the target inner tolerance in reached)
            inner_tolerance =
                target_inner_tolerance.max(self.inner_tolerance_update_factor * inner_tolerance);
        }

        // TODO: return correct status code
        let max_constraint_violation = matrix_operations::norm_inf(&constraint_values);
        Ok(HomotopySolverStatus::new(
            exit_status,
            num_outer_iterations,
            num_inner_iterations,
            last_norm_fpr,
            max_constraint_violation,
            now.elapsed(),
        ))
    }
}
