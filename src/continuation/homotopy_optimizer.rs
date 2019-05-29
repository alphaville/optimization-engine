use crate::{
    constraints,
    continuation::homotopy_problem::HomotopyProblem,
    continuation::ContinuationMode,
    continuation::HomotopySolverStatus,
    core::panoc::*,
    core::{panoc, ExitStatus, Optimizer, Problem},
    matrix_operations, Error,
};

const DEFAULT_CONSTRAINT_TOLERANCE: f64 = 1e-4;
const DEFAULT_MAX_OUTER_ITERATIONS: usize = 10;
const DEFAULT_MAX_INNER_ITERATIONS: usize = 500;

/// Homotopy optimizer
pub struct HomotopyOptimizer<
    'a,
    ParametricPenaltyFunctionType,
    ParametricGradientType,
    ConstraintType,
    ParametricCostType,
> where
    ParametricPenaltyFunctionType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), Error>,
    ParametricGradientType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), Error>,
    ParametricCostType: Fn(&[f64], &[f64], &mut f64) -> Result<(), Error>,
    ConstraintType: constraints::Constraint,
{
    /// Definition of parametric problem
    homotopy_problem: &'a HomotopyProblem<
        ParametricPenaltyFunctionType,
        ParametricGradientType,
        ConstraintType,
        ParametricCostType,
    >,
    /// Instance of PANOCCache
    panoc_cache: &'a mut panoc::PANOCCache,
    /// Tolerance on constraint satisfaction
    constraint_tolerance: f64,
    /// Maximum number of outer iterations
    max_outer_iterations: usize,
    /// Maximum number of inner iterations
    max_inner_iterations: usize,
    /// Maximum duration
    max_duration: Option<std::time::Duration>,
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
    ParametricPenaltyFunctionType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), Error>,
    ParametricGradientType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), Error>,
    ParametricCostType: Fn(&[f64], &[f64], &mut f64) -> Result<(), Error>,
    ConstraintType: constraints::Constraint,
{
    /// Constructor for `HomotopyOptimizer`
    ///
    ///
    /// ## Arguments
    /// - homotopy_problem
    /// - panoc_cache
    /// - constraint_tolerance
    /// - max_outer_iterations
    ///
    /// ## Returns
    ///
    /// - New instance of `HomotopyOptimizer`
    ///
    pub fn new(
        homotopy_problem: &'a HomotopyProblem<
            ParametricPenaltyFunctionType,
            ParametricGradientType,
            ConstraintType,
            ParametricCostType,
        >,
        panoc_cache: &'a mut panoc::PANOCCache,
    ) -> HomotopyOptimizer<
        'a,
        ParametricPenaltyFunctionType,
        ParametricGradientType,
        ConstraintType,
        ParametricCostType,
    > {
        HomotopyOptimizer {
            homotopy_problem: homotopy_problem,
            panoc_cache: panoc_cache,
            constraint_tolerance: DEFAULT_CONSTRAINT_TOLERANCE,
            max_outer_iterations: DEFAULT_MAX_OUTER_ITERATIONS,
            max_inner_iterations: DEFAULT_MAX_INNER_ITERATIONS,
            max_duration: None,
        }
    }

    fn initialize_param(&self, q_augmented_params: &mut [f64]) {
        let homotopy_problem = &self.homotopy_problem;
        let idx_list = &homotopy_problem.idx;
        let from_list = &homotopy_problem.from;
        for (i, from_value) in idx_list.iter().zip(from_list.iter()) {
            q_augmented_params[*i] = *from_value;
        }
    }

    pub fn with_constraint_tolerance(mut self, constraint_tolerance: f64) -> Self {
        self.constraint_tolerance = constraint_tolerance;
        self
    }

    pub fn with_max_outer_iterations(mut self, max_outer_iterations: usize) -> Self {
        self.max_outer_iterations = max_outer_iterations;
        self
    }

    pub fn with_max_inner_iterations(mut self, max_inner_iterations: usize) -> Self {
        self.max_inner_iterations = max_inner_iterations;
        self
    }

    pub fn with_max_duration(&mut self, max_duration: std::time::Duration) -> &Self {
        self.max_duration = Some(max_duration);
        self
    }

    // TODO: return a status code (target_reached_flag); this way we will know
    // whether to continue iterating
    //
    // TODO: Now all parameters are updated; maybe update only those which
    // violate the termination conditions (c)
    fn update_continuation_parameters(&self, p_: &mut [f64]) {
        let homotopy_problem = &self.homotopy_problem;
        let idx_list = &homotopy_problem.idx;
        let transition_list = &homotopy_problem.transition_mode;
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
    pub fn solve(
        &'a mut self,
        u: &mut [f64],
        q_augmented_param: &[f64],
    ) -> Result<HomotopySolverStatus, Error> {
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
        let mut constraint_values: Vec<f64> = vec![0.0; 1 + num_penalty_constraints];
        let mut available_time_left = self.max_duration;

        let mut exit_status = ExitStatus::Converged;

        // OUTER ITERATIONS
        for _iter_outer in 1..=self.max_outer_iterations {
            // Figure out whether there is any time left
            if let Some(max_duration) = self.max_duration {
                available_time_left = max_duration.checked_sub(now.elapsed());
                if None == available_time_left {
                    // no time left!
                    exit_status = ExitStatus::NotConvergedOutOfTime;
                    break;
                }
            }

            num_outer_iterations += 1;
            let homotopy_problem = &self.homotopy_problem;
            let f_ = |u: &[f64], cost: &mut f64| -> Result<(), Error> {
                (homotopy_problem.parametric_cost)(u, &q_augmented_param_vec, cost)
            };
            let df_ = |u: &[f64], grad: &mut [f64]| -> Result<(), Error> {
                (homotopy_problem.parametric_gradient)(u, &q_augmented_param_vec, grad)
            };
            let inner_problem = Problem::new(&self.homotopy_problem.constraints, df_, f_);
            let mut inner_panoc = panoc::PANOCOptimizer::new(inner_problem, &mut self.panoc_cache);

            if available_time_left != None {
                inner_panoc.with_max_duration(available_time_left.unwrap());
            }

            //TODO: check status of inner solver... what happens if the inner solver has not
            //converged? we need to come up with a heuristic. Is the situation salvagable?
            //If ||norm_fpr|| is "reasonably" low, we can still continue hoping that the
            //next problem will converge (this is likely to happen)
            let status = inner_panoc.solve(u);
            num_inner_iterations += status.iterations();
            last_norm_fpr = status.norm_fpr();
            (homotopy_problem.penalty_function)(u, &q_augmented_param_vec, &mut constraint_values)?;
            let continue_outer_iterations = constraint_values
                .iter()
                .any(|&ci| ci > self.constraint_tolerance);
            if !continue_outer_iterations {
                break;
            } else {
                self.update_continuation_parameters(&mut q_augmented_param_vec);
            }
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
