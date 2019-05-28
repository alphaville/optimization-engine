use crate::{
    constraints,
    continuation::homotopy_problem::HomotopyProblem,
    continuation::ContinuationMode,
    continuation::HomotopySolverStatus,
    core::panoc::*,
    core::{panoc, Optimizer, Problem},
    Error,
};

const DEFAULT_CONSTRAINT_TOLERANCE: f64 = 1e-4;
const DEFAULT_MAX_OUTER_ITERATIONS: usize = 10;

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
    homotopy_problem: &'a HomotopyProblem<
        ParametricPenaltyFunctionType,
        ParametricGradientType,
        ConstraintType,
        ParametricCostType,
    >,
    panoc_cache: &'a mut panoc::PANOCCache,
    constraint_tolerance: f64,
    max_outer_iterations: usize,
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

    // TODO: return a status code (target_reached_flag); this way we will know
    // whether to continue iterating
    //
    // TODO: Now all parameters are updated; maybe update only those which
    // violate the termination conditions (c)
    fn update_continuation_parameters(&self, p_: &mut [f64]) {
        let homotopy_problem = &self.homotopy_problem;
        let idx_list = &homotopy_problem.idx;
        println!("idx_list = {:?}", idx_list);
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
        let mut q_augmented_param_vec: Vec<f64> = q_augmented_param.to_vec();
        self.initialize_param(&mut q_augmented_param_vec);

        // Another consideration is the total time; the time should be monitored
        // and every next instance of the (inner) solver should be given the
        // time that is left
        let mut num_outer_iterations = 0;
        let mut num_inner_iterations = 0;
        for iter_outer in 1..=self.max_outer_iterations {
            num_outer_iterations += 1;
            println!("\niter = {}", iter_outer);

            let homotopy_problem = &self.homotopy_problem;
            let f_ = |u: &[f64], cost: &mut f64| -> Result<(), Error> {
                (homotopy_problem.parametric_cost)(u, &q_augmented_param_vec, cost)
            };
            let df_ = |u: &[f64], grad: &mut [f64]| -> Result<(), Error> {
                (homotopy_problem.parametric_gradient)(u, &q_augmented_param_vec, grad)
            };
            let problem_ = Problem::new(&self.homotopy_problem.constraints, df_, f_);
            let mut panoc_ = panoc::PANOCOptimizer::new(problem_, &mut self.panoc_cache);

            //TODO: check status of inner solver... what happens if the inner solver has not
            //converged? we need to come up with a heuristic. Is the situation salvagable?
            //If ||norm_fpr|| is "reasonably" low, we can still continue hoping that the
            //next problem will converge (this is likely to happen)
            let status = panoc_.solve(u);
            num_inner_iterations += status.iterations();
            println!("{:#?}", status);

            let mut c = [0.];
            (homotopy_problem.penalty_function)(u, &q_augmented_param_vec, &mut c)?;
            println!("constraints = {:?}", c);
            let continue_outer_iterations = c.iter().any(|&ci| ci > self.constraint_tolerance);
            if !continue_outer_iterations {
                break;
            } else {
                self.update_continuation_parameters(&mut q_augmented_param_vec);
            }
            println!("updated p = {:?}", q_augmented_param_vec);
        }
        // Need to return a different type of  SolverStatus (HomotopySolverStatus)
        // with the number of outer iterations, maximum number of inner iterations,
        // total outer time etc
        Ok(HomotopySolverStatus::new(
            true,
            num_outer_iterations,
            num_inner_iterations,
            0.,
        ))
    }
}
