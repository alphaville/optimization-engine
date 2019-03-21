//! FBS Algorithm
//!
use super::super::AlgorithmEngine;
use super::super::Optimizer;
use super::super::SolverStatus;
use super::PANOCEngine;
use super::PANOCOptimizer;
use crate::constraints;

const MAX_ITER: usize = 100_usize;

impl<'a, GradientType, ConstraintType, CostType>
    PANOCOptimizer<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32,
    CostType: Fn(&[f64], &mut f64) -> i32,
    ConstraintType: constraints::Constraint,
{
    pub fn new(
        panoc_engine: &'a mut PANOCEngine<'a, GradientType, ConstraintType, CostType>,
    ) -> PANOCOptimizer<'a, GradientType, ConstraintType, CostType> {
        PANOCOptimizer {
            panoc_engine: panoc_engine,
            max_iter: MAX_ITER,
        }
    }

    /// Sets the tolerance
    ///
    /// ## Panics
    ///
    /// The method panics if the specified tolerance is not positive
    pub fn with_tolerance(
        &mut self,
        tolerance: f64,
    ) -> &mut PANOCOptimizer<'a, GradientType, ConstraintType, CostType> {
        assert!(tolerance > 0.0);

        self.panoc_engine.cache.tolerance = tolerance;
        self
    }

    /// Sets the maximum number of iterations
    pub fn with_max_iter(
        &mut self,
        max_iter: usize,
    ) -> &mut PANOCOptimizer<'a, GradientType, ConstraintType, CostType> {
        self.max_iter = max_iter;
        self
    }
}

impl<'life, GradientType, ConstraintType, CostType> Optimizer
    for PANOCOptimizer<'life, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32 + 'life,
    CostType: Fn(&[f64], &mut f64) -> i32,
    ConstraintType: constraints::Constraint + 'life,
{
    fn solve(&mut self, u: &mut [f64]) -> SolverStatus {
        self.panoc_engine.init(u);
        let mut num_iter: usize = 0;
        while self.panoc_engine.step(u) && num_iter < self.max_iter {
            num_iter += 1;
        }

        // export solution status
        SolverStatus::new(
            num_iter < self.max_iter,
            num_iter,
            self.panoc_engine.cache.norm_gamma_fpr,
            self.panoc_engine.cache.cost_value,
        )
    }
}

/* --------------------------------------------------------------------------------------------- */
/*       TESTS                                                                                   */
/* --------------------------------------------------------------------------------------------- */
#[cfg(test)]
mod tests {

    use crate::core::panoc::*;
    use crate::core::*;
    use crate::mocks;
    use std::num::NonZeroUsize;

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
        let df = |u: &[f64], grad: &mut [f64]| -> i32 {
            mocks::rosenbrock_grad(a, b, u, grad);
            0
        };
        let f = |u: &[f64], c: &mut f64| -> i32 {
            *c = mocks::rosenbrock_cost(a, b, u);
            0
        };
        /* CONSTRAINTS */
        let radius = 2.0;
        let bounds = constraints::Ball2::new_at_origin_with_radius(radius);
        let mut panoc_cache = PANOCCache::new(
            NonZeroUsize::new(n).unwrap(),
            tolerance,
            NonZeroUsize::new(lbfgs_memory).unwrap(),
        );
        let problem = Problem::new(bounds, df, f);
        let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);
        let mut panoc = PANOCOptimizer::new(&mut panoc_engine);
        panoc.with_max_iter(max_iters);
        let now = std::time::Instant::now();
        let status = panoc.solve(&mut u);

        println!("{} iterations", status.num_iter);
        println!("elapsed {:?}", now.elapsed());

        assert_eq!(max_iters, panoc.max_iter);
        assert!(status.converged);
        assert!(status.num_iter < max_iters);
        assert!(status.fpr_norm < tolerance);
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
        let mut panoc_cache = PANOCCache::new(
            NonZeroUsize::new(n).unwrap(),
            tolerance,
            NonZeroUsize::new(lbfgs_memory).unwrap(),
        );
        let mut radius = 1.0;
        for _ in 1..100 {
            b *= 1.01;
            a -= 1e-3;
            radius += 0.001;
            let df = |u: &[f64], grad: &mut [f64]| -> i32 {
                mocks::rosenbrock_grad(a, b, u, grad);
                0
            };
            let f = |u: &[f64], c: &mut f64| -> i32 {
                *c = mocks::rosenbrock_cost(a, b, u);
                0
            };
            let bounds = constraints::Ball2::new_at_origin_with_radius(radius);
            let problem = Problem::new(bounds, df, f);
            let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);
            let mut panoc = PANOCOptimizer::new(&mut panoc_engine);
            panoc.with_max_iter(max_iters);

            let status = panoc.solve(&mut u);

            println!(
                "parameters: (a={:.4}, b={:.4}, r={:.4}), iters = {}",
                a, b, radius, status.num_iter
            );
            println!("u = {:#.6?}", u);

            assert_eq!(max_iters, panoc.max_iter);
            assert!(status.converged);
            assert!(status.num_iter < max_iters);
            assert!(status.fpr_norm < tolerance);
        }

        /* PROBLEM STATEMENT */
    }
}
