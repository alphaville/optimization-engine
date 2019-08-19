//! PANOC optimizer
//!
use crate::{
    constraints,
    core::{
        panoc::panoc_engine::PANOCEngine, panoc::PANOCCache, AlgorithmEngine, ExitStatus,
        Optimizer, Problem, SolverStatus,
    },
    matrix_operations, SolverError,
};
use std::time;

const MAX_ITER: usize = 100_usize;

pub struct PANOCOptimizer<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> Result<(), SolverError>,
    CostType: Fn(&[f64], &mut f64) -> Result<(), SolverError>,
    ConstraintType: constraints::Constraint,
{
    panoc_engine: PANOCEngine<'a, GradientType, ConstraintType, CostType>,
    max_iter: usize,
    max_duration: Option<time::Duration>,
}

impl<'a, GradientType, ConstraintType, CostType>
    PANOCOptimizer<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> Result<(), SolverError>,
    CostType: Fn(&[f64], &mut f64) -> Result<(), SolverError>,
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
    ) -> PANOCOptimizer<'a, GradientType, ConstraintType, CostType> {
        PANOCOptimizer {
            panoc_engine: PANOCEngine::new(problem, cache),
            max_iter: MAX_ITER,
            max_duration: None,
        }
    }

    /// Sets the tolerance
    ///
    /// ## Panics
    ///
    /// The method panics if the specified tolerance is not positive
    pub fn with_tolerance(
        mut self,
        tolerance: f64,
    ) -> PANOCOptimizer<'a, GradientType, ConstraintType, CostType> {
        assert!(tolerance > 0.0, "tolerance must be larger than 0");

        self.panoc_engine.cache.tolerance = tolerance;
        self
    }

    /// Sets the maximum number of iterations
    ///
    /// ## Panic
    ///
    /// Panics if the provided number of iterations is equal to zero
    pub fn with_max_iter(
        mut self,
        max_iter: usize,
    ) -> PANOCOptimizer<'a, GradientType, ConstraintType, CostType> {
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
    GradientType: Fn(&[f64], &mut [f64]) -> Result<(), SolverError> + 'life,
    CostType: Fn(&[f64], &mut f64) -> Result<(), SolverError>,
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

    use crate::core::panoc::*;
    use crate::core::*;
    use crate::{mocks, SolverError};

    #[test]
    fn t_panoc_optimizer_rosenbrock() {
        /* USER PARAMETERS */
        let tolerance = 1e-6;
        let a = 1.0;
        let b = 200.0;
        let n = 2;
        let lbfgs_memory = 8;
        let max_iters = 80;
        let mut u = [-1.5, 0.9];

        /* COST FUNCTION */
        let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
            mocks::rosenbrock_grad(a, b, u, grad);
            Ok(())
        };
        let f = |u: &[f64], c: &mut f64| -> Result<(), SolverError> {
            *c = mocks::rosenbrock_cost(a, b, u);
            Ok(())
        };
        /* CONSTRAINTS */
        let radius = 2.0;
        let bounds = constraints::Ball2::new(None, radius);
        let mut panoc_cache = PANOCCache::new(n, tolerance, lbfgs_memory);
        let problem = Problem::new(&bounds, df, f);
        let mut panoc = PANOCOptimizer::new(problem, &mut panoc_cache).with_max_iter(max_iters);
        let now = std::time::Instant::now();
        let status = panoc.solve(&mut u).unwrap();

        println!("{} iterations", status.iterations());
        println!("elapsed {:?}", now.elapsed());

        assert_eq!(max_iters, panoc.max_iter);
        assert!(status.has_converged());
        assert!(status.iterations() < max_iters);
        assert!(status.norm_fpr() < tolerance);
    }

    #[test]
    fn t_panoc_in_loop() {
        /* USER PARAMETERS */
        let tolerance = 1e-6;
        let mut a = 1.0;
        let mut b = 100.0;
        let n = 2;
        let lbfgs_memory = 10;
        let max_iters = 100;
        let mut u = [-1.5, 0.9];
        let mut panoc_cache = PANOCCache::new(n, tolerance, lbfgs_memory);
        let mut radius = 1.0;
        for _ in 1..100 {
            b *= 1.01;
            a -= 1e-3;
            radius += 0.001;
            let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
                mocks::rosenbrock_grad(a, b, u, grad);
                Ok(())
            };
            let f = |u: &[f64], c: &mut f64| -> Result<(), SolverError> {
                *c = mocks::rosenbrock_cost(a, b, u);
                Ok(())
            };
            let bounds = constraints::Ball2::new(None, radius);
            let problem = Problem::new(&bounds, df, f);
            let mut panoc = PANOCOptimizer::new(problem, &mut panoc_cache).with_max_iter(max_iters);

            let status = panoc.solve(&mut u).unwrap();

            println!(
                "parameters: (a={:.4}, b={:.4}, r={:.4}), iters = {}",
                a,
                b,
                radius,
                status.iterations()
            );
            println!("u = {:#.6?}", u);

            assert_eq!(max_iters, panoc.max_iter);
            assert!(status.has_converged());
            assert!(status.iterations() < max_iters);
            assert!(status.norm_fpr() < tolerance);
        }

        /* PROBLEM STATEMENT */
    }
}
