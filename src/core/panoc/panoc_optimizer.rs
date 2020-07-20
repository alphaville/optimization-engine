//! PANOC optimizer
//!
use crate::{
    constraints,
    core::{
        panoc::panoc_engine::PANOCEngine, panoc::PANOCCache, AlgorithmEngine, ExitStatus,
        Optimizer, Problem, SolverStatus,
    },
    matrix_operations, FunctionCallResult, SolverError,
};
use std::time;

const MAX_ITER: usize = 100_usize;

/// Optimizer using the PANOC algorithm
///
///
pub struct PANOCOptimizer<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> FunctionCallResult,
    CostType: Fn(&[f64], &mut f64) -> FunctionCallResult,
    ConstraintType: constraints::Constraint,
{
    panoc_engine: PANOCEngine<'a, GradientType, ConstraintType, CostType>,
    max_iter: usize,
    max_duration: Option<time::Duration>,
}

impl<'a, GradientType, ConstraintType, CostType>
    PANOCOptimizer<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> FunctionCallResult,
    CostType: Fn(&[f64], &mut f64) -> FunctionCallResult,
    ConstraintType: constraints::Constraint,
{
    /// Constructor of `PANOCOptimizer`
    ///
    /// ## Arguments
    ///
    /// - problem: definition of optimization problem
    /// - cache: cache object constructed once
    ///
    /// ## Panic
    ///
    /// Does not panic
    pub fn new(
        problem: Problem<'a, GradientType, ConstraintType, CostType>,
        cache: &'a mut PANOCCache,
    ) -> Self {
        PANOCOptimizer {
            panoc_engine: PANOCEngine::new(problem, cache),
            max_iter: MAX_ITER,
            max_duration: None,
        }
    }

    /// Sets the tolerance on the norm of the fixed-point residual
    ///
    /// The algorithm will exit if the form of gamma*FPR drops below
    /// this tolerance
    ///
    /// ## Panics
    ///
    /// The method panics if the specified tolerance is not positive
    pub fn with_tolerance(mut self, tolerance: f64) -> Self {
        assert!(tolerance > 0.0, "tolerance must be larger than 0");

        self.panoc_engine.cache.tolerance = tolerance;
        self
    }

    /// Specify the tolerance $\epsilon$ related to the AKKT condition
    ///
    /// $$
    /// \Vert{}\gamma^{-1}(u-u^+) + \nabla f(u) - \nabla f(u^+){}\Vert \leq \epsilon
    /// $$
    ///
    /// ## Arguments
    ///
    /// - `akkt_tolerance`: the AKKT-specific tolerance
    ///
    ///
    /// ## Returns
    ///
    /// Returns the current mutable and updated instance of the provided object
    ///
    ///  
    /// ## Panics
    ///
    /// The method panics if the provided value of the AKKT-specific tolerance is
    /// not positive.
    ///
    pub fn with_akkt_tolerance(self, akkt_tolerance: f64) -> Self {
        assert!(akkt_tolerance > 0.0, "akkt_tolerance must be positive");
        self.panoc_engine.cache.set_akkt_tolerance(akkt_tolerance);
        self
    }

    /// Sets the maximum number of iterations
    ///
    /// ## Panics
    ///
    /// Panics if the provided number of iterations is equal to zero
    pub fn with_max_iter(mut self, max_iter: usize) -> Self {
        assert!(max_iter > 0, "max_iter must be larger than 0");

        self.max_iter = max_iter;
        self
    }

    /// Sets the maximum solution time, useful in real-time applications
    pub fn with_max_duration(mut self, max_duation: time::Duration) -> Self {
        self.max_duration = Some(max_duation);
        self
    }
}

impl<'life, GradientType, ConstraintType, CostType> Optimizer
    for PANOCOptimizer<'life, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> FunctionCallResult + 'life,
    CostType: Fn(&[f64], &mut f64) -> FunctionCallResult,
    ConstraintType: constraints::Constraint + 'life,
{
    fn solve(&mut self, u: &mut [f64]) -> Result<SolverStatus, SolverError> {
        let now = time::Instant::now();

        /*
         * Initialise [call panoc_engine.init()]
         * and check whether it returns Ok(())
         */
        self.panoc_engine.init(u)?;

        /* Main loop */
        let mut num_iter: usize = 0;
        let mut continue_num_iters = true;
        let mut continue_runtime = true;

        let mut step_flag = self.panoc_engine.step(u)?;
        if let Some(dur) = self.max_duration {
            while step_flag && continue_num_iters && continue_runtime {
                num_iter += 1;
                continue_num_iters = num_iter < self.max_iter;
                continue_runtime = now.elapsed() <= dur;
                step_flag = self.panoc_engine.step(u)?;
            }
        } else {
            while step_flag && continue_num_iters {
                num_iter += 1;
                continue_num_iters = num_iter < self.max_iter;
                step_flag = self.panoc_engine.step(u)?;
            }
        }

        // check for possible NaN/inf
        if !matrix_operations::is_finite(&u) {
            return Err(SolverError::NotFiniteComputation);
        }

        // exit status
        let exit_status = if !continue_num_iters {
            ExitStatus::NotConvergedIterations
        } else if !continue_runtime {
            ExitStatus::NotConvergedOutOfTime
        } else {
            ExitStatus::Converged
        };

        // copy u_half_step into u (the algorithm should return u_bar,
        // because it's always feasible, while u may violate the constraints)
        u.copy_from_slice(&self.panoc_engine.cache.u_half_step);

        // export solution status (exit status, num iterations and more)
        Ok(SolverStatus::new(
            exit_status,
            num_iter,
            now.elapsed(),
            self.panoc_engine.cache.norm_gamma_fpr,
            self.panoc_engine.cache.cost_value,
        ))
    }
}

/* --------------------------------------------------------------------------------------------- */
/*       TESTS                                                                                   */
/* --------------------------------------------------------------------------------------------- */
#[cfg(test)]
mod tests {

    use crate::core::constraints::*;
    use crate::core::panoc::*;
    use crate::core::*;
    use crate::{mocks, FunctionCallResult};

    #[test]
    fn t_panoc_optimizer_rosenbrock() {
        /* USER PARAMETERS */
        let tolerance = 1e-6;
        let a_param = 1.0;
        let b_param = 200.0;
        let n_dimension = 2;
        let lbfgs_memory = 8;
        let max_iters = 80;
        let mut u_solution = [-1.5, 0.9];

        /* COST FUNCTION */
        let cost_gradient = |u: &[f64], grad: &mut [f64]| -> FunctionCallResult {
            mocks::rosenbrock_grad(a_param, b_param, u, grad);
            Ok(())
        };
        let cost_function = |u: &[f64], c: &mut f64| -> FunctionCallResult {
            *c = mocks::rosenbrock_cost(a_param, b_param, u);
            Ok(())
        };
        /* CONSTRAINTS */
        let radius = 2.0;
        let bounds = constraints::Ball2::new(None, radius);
        let mut panoc_cache = PANOCCache::new(n_dimension, tolerance, lbfgs_memory);
        let problem = Problem::new(&bounds, cost_gradient, cost_function);
        let mut panoc = PANOCOptimizer::new(problem, &mut panoc_cache).with_max_iter(max_iters);
        let now = std::time::Instant::now();
        let status = panoc.solve(&mut u_solution).unwrap();

        println!("{} iterations", status.iterations());
        println!("elapsed {:?}", now.elapsed());

        assert_eq!(max_iters, panoc.max_iter);
        assert!(status.has_converged());
        assert!(status.iterations() < max_iters);
        assert!(status.norm_fpr() < tolerance);

        /* CHECK FEASIBILITY */
        let mut u_project = [0.0; 2];
        u_project.copy_from_slice(&u_solution);
        bounds.project(&mut u_project);
        unit_test_utils::assert_nearly_equal_array(
            &u_solution,
            &u_project,
            1e-12,
            1e-16,
            "infeasibility detected",
        );
    }

    #[test]
    fn t_panoc_in_loop() {
        /* USER PARAMETERS */
        let tolerance = 1e-5;
        let mut a_param = 1.0;
        let mut b_param = 100.0;
        let n_dimension = 2;
        let lbfgs_memory = 10;
        let max_iters = 100;
        let mut u_solution = [-1.5, 0.9];
        let mut panoc_cache = PANOCCache::new(n_dimension, tolerance, lbfgs_memory);
        for i in 1..=100 {
            b_param *= 1.01;
            a_param -= 1e-3;
            // Note: updating `radius` like this because `radius += 0.01` builds up small
            // numerical errors and is less reliable
            let radius = 1.0 + 0.01 * (i as f64);
            let cost_gradient = |u: &[f64], grad: &mut [f64]| -> FunctionCallResult {
                mocks::rosenbrock_grad(a_param, b_param, u, grad);
                Ok(())
            };
            let cost_function = |u: &[f64], c: &mut f64| -> FunctionCallResult {
                *c = mocks::rosenbrock_cost(a_param, b_param, u);
                Ok(())
            };
            let bounds = constraints::Ball2::new(None, radius);
            let problem = Problem::new(&bounds, cost_gradient, cost_function);
            let mut panoc = PANOCOptimizer::new(problem, &mut panoc_cache).with_max_iter(max_iters);

            let status = panoc.solve(&mut u_solution).unwrap();

            println!("status = {:#?}", status);
            println!(
                "parameters: (a={:.4}, b={:.4}, r={:.4}), iters = {}",
                a_param,
                b_param,
                radius,
                status.iterations()
            );

            /* CHECK FEASIBILITY */
            // The norm of u must be <= radius
            let norm_u = crate::matrix_operations::norm2(&u_solution);
            assert!(
                norm_u <= radius + 5e-16,
                "infeasibility in problem solution"
            );

            assert_eq!(max_iters, panoc.max_iter);
            assert!(status.has_converged());
            assert!(status.iterations() < max_iters);
            assert!(status.norm_fpr() < tolerance);
        }
    }

    #[test]
    fn t_panoc_optimizer_akkt_tolerance() {
        /* USER PARAMETERS */
        let tolerance = 1e-6;
        let akkt_tolerance = 1e-6;
        let a_param = 1.0;
        let b_param = 200.0;
        let n_dimension = 2;
        let lbfgs_memory = 8;
        let max_iters = 580;
        let mut u_solution = [-1.5, 0.9];

        let cost_gradient = |u: &[f64], grad: &mut [f64]| -> FunctionCallResult {
            mocks::rosenbrock_grad(a_param, b_param, u, grad);
            Ok(())
        };
        let cost_function = |u: &[f64], c: &mut f64| -> FunctionCallResult {
            *c = mocks::rosenbrock_cost(a_param, b_param, u);
            Ok(())
        };

        let radius = 1.2;
        let bounds = constraints::Ball2::new(None, radius);

        let mut panoc_cache = PANOCCache::new(n_dimension, tolerance, lbfgs_memory);
        let problem = Problem::new(&bounds, cost_gradient, cost_function);

        let mut panoc = PANOCOptimizer::new(problem, &mut panoc_cache)
            .with_max_iter(max_iters)
            .with_akkt_tolerance(akkt_tolerance);

        let status = panoc.solve(&mut u_solution).unwrap();

        assert_eq!(max_iters, panoc.max_iter);
        assert!(status.has_converged());
        assert!(status.iterations() < max_iters);
        assert!(status.norm_fpr() < tolerance);
    }
}
