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
use lbfgs::LbfgsPrecision;
use num::Float;
use std::iter::Sum;
use std::time;

const MAX_ITER: usize = 100_usize;

/// Optimizer using the PANOC algorithm
///
///
pub struct PANOCOptimizer<'a, GradientType, ConstraintType, CostType, T = f64>
where
    T: Float + LbfgsPrecision + Sum<T>,
    GradientType: Fn(&[T], &mut [T]) -> FunctionCallResult,
    CostType: Fn(&[T], &mut T) -> FunctionCallResult,
    ConstraintType: constraints::Constraint<T>,
{
    panoc_engine: PANOCEngine<'a, GradientType, ConstraintType, CostType, T>,
    max_iter: usize,
    max_duration: Option<time::Duration>,
}

impl<'a, GradientType, ConstraintType, CostType, T>
    PANOCOptimizer<'a, GradientType, ConstraintType, CostType, T>
where
    T: Float + LbfgsPrecision + Sum<T>,
    GradientType: Fn(&[T], &mut [T]) -> FunctionCallResult,
    CostType: Fn(&[T], &mut T) -> FunctionCallResult,
    ConstraintType: constraints::Constraint<T>,
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
    #[must_use]
    pub fn new(
        problem: Problem<'a, GradientType, ConstraintType, CostType, T>,
        cache: &'a mut PANOCCache<T>,
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
    #[must_use]
    pub fn with_tolerance(self, tolerance: T) -> Self {
        assert!(tolerance > T::zero(), "tolerance must be larger than 0");

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
    #[must_use]
    pub fn with_akkt_tolerance(self, akkt_tolerance: T) -> Self {
        assert!(
            akkt_tolerance > T::zero(),
            "akkt_tolerance must be positive"
        );
        self.panoc_engine.cache.set_akkt_tolerance(akkt_tolerance);
        self
    }

    /// Sets the maximum number of iterations
    ///
    /// ## Panics
    ///
    /// Panics if the provided number of iterations is equal to zero
    #[must_use]
    pub fn with_max_iter(mut self, max_iter: usize) -> Self {
        assert!(max_iter > 0, "max_iter must be larger than 0");

        self.max_iter = max_iter;
        self
    }

    /// Sets the maximum solution time, useful in real-time applications
    #[must_use]
    pub fn with_max_duration(mut self, max_duation: time::Duration) -> Self {
        self.max_duration = Some(max_duation);
        self
    }

    /// Solves the optimization problem for decision variables of scalar type `T`.
    pub fn solve(&mut self, u: &mut [T]) -> Result<SolverStatus<T>, SolverError> {
        let now = web_time::Instant::now();

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

        // Score the latest feasible half step before exiting: if we stopped
        // because of time or iteration limits, it may be better than the last
        // one that was fully evaluated inside `step`.
        self.panoc_engine.cache_best_half_step(u);

        // check for possible NaN/inf
        if !matrix_operations::is_finite(u) {
            return Err(SolverError::NotFiniteComputation(
                "final PANOC iterate contains a non-finite value",
            ));
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
        u.copy_from_slice(&self.panoc_engine.cache.best_u_half_step);

        let best_cost_value = self.panoc_engine.cost_value_at_best_half_step()?;

        // export solution status (exit status, num iterations and more)
        Ok(SolverStatus::new(
            exit_status,
            num_iter,
            now.elapsed(),
            self.panoc_engine.cache.best_norm_gamma_fpr,
            best_cost_value,
        ))
    }
}

impl<'life, GradientType, ConstraintType, CostType, T> Optimizer<T>
    for PANOCOptimizer<'life, GradientType, ConstraintType, CostType, T>
where
    T: Float + LbfgsPrecision + Sum<T>,
    GradientType: Fn(&[T], &mut [T]) -> FunctionCallResult + 'life,
    CostType: Fn(&[T], &mut T) -> FunctionCallResult,
    ConstraintType: constraints::Constraint<T> + 'life,
{
    fn solve(&mut self, u: &mut [T]) -> Result<SolverStatus<T>, SolverError> {
        PANOCOptimizer::solve(self, u)
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
        let now = web_time::Instant::now();
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
        bounds.project(&mut u_project).unwrap();
        unit_test_utils::assert_nearly_equal_array(
            &u_solution,
            &u_project,
            1e-12,
            1e-16,
            "infeasibility detected",
        );
    }

    #[test]
    fn t_panoc_optimizer_rosenbrock_f32() {
        let tolerance = 1e-4_f32;
        let a_param = 1.0_f32;
        let b_param = 200.0_f32;
        let n_dimension = 2;
        let lbfgs_memory = 8;
        let max_iters = 120;
        let mut u_solution = [-1.5_f32, 0.9_f32];

        let cost_gradient = |u: &[f32], grad: &mut [f32]| -> FunctionCallResult {
            mocks::rosenbrock_grad(a_param, b_param, u, grad);
            Ok(())
        };
        let cost_function = |u: &[f32], c: &mut f32| -> FunctionCallResult {
            *c = mocks::rosenbrock_cost(a_param, b_param, u);
            Ok(())
        };

        let radius = 2.0_f32;
        let bounds = constraints::Ball2::new(None, radius);
        let mut panoc_cache = PANOCCache::<f32>::new(n_dimension, tolerance, lbfgs_memory);
        let problem = Problem::new(&bounds, cost_gradient, cost_function);
        let mut panoc = PANOCOptimizer::new(problem, &mut panoc_cache).with_max_iter(max_iters);
        let status = panoc.solve(&mut u_solution).unwrap();

        assert_eq!(max_iters, panoc.max_iter);
        assert!(status.has_converged());
        assert!(status.iterations() < max_iters);
        assert!(status.norm_fpr() < tolerance);

        let mut u_project = [0.0_f32; 2];
        u_project.copy_from_slice(&u_solution);
        bounds.project(&mut u_project);
        assert!((u_solution[0] - u_project[0]).abs() < 1e-5_f32);
        assert!((u_solution[1] - u_project[1]).abs() < 1e-5_f32);
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

    #[test]
    fn t_panoc_optimizer_premature_exit_returns_best_previous_half_step() {
        let tolerance = 1e-6;
        let radius = 0.05;
        let n_dimension = 3;
        let lbfgs_memory = 10;

        let mut found_nonlast_best_half_step = false;

        for max_iters in 2..=25 {
            let bounds = constraints::Ball2::new(None, radius);
            let problem = Problem::new(
                &bounds,
                mocks::hard_quadratic_gradient,
                mocks::hard_quadratic_cost,
            );
            let mut panoc_cache = PANOCCache::new(n_dimension, tolerance, lbfgs_memory);
            let mut panoc = PANOCOptimizer::new(problem, &mut panoc_cache).with_max_iter(max_iters);
            let mut u_solution = [-20.0, 10.0, 0.2];

            let status = panoc.solve(&mut u_solution).unwrap();

            let distance_to_last_half_step =
                crate::matrix_operations::norm_inf_diff(&u_solution, &panoc_cache.u_half_step);

            if status.exit_status() == ExitStatus::NotConvergedIterations
                && distance_to_last_half_step > 1e-12
            {
                found_nonlast_best_half_step = true;

                unit_test_utils::assert_nearly_equal_array(
                    &u_solution,
                    &panoc_cache.best_u_half_step,
                    1e-12,
                    1e-12,
                    "returned solution should equal the best cached half step",
                );
                assert!(
                    status.norm_fpr() < panoc_cache.norm_gamma_fpr,
                    "returned FPR should be strictly better than the last half step"
                );
            }
        }

        assert!(
            found_nonlast_best_half_step,
            "did not find a premature exit where the best half step differs from the last one"
        );
    }
}
