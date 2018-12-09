//! FBS Algorithm
//!
use super::super::AlgorithmEngine;
use super::super::Optimizer;
use super::super::SolverStatus;
use super::FBSEngine;
use super::FBSOptimizer;
use crate::constraints;

const MAX_ITER: usize = 100_usize;

impl<'a, GradientType, ConstraintType, CostType>
    FBSOptimizer<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32,
    CostType: Fn(&[f64], &mut f64) -> i32,
    ConstraintType: constraints::Constraint,
{
    pub fn new(
        fbs_engine: &'a mut FBSEngine<'a, GradientType, ConstraintType, CostType>,
    ) -> FBSOptimizer<'a, GradientType, ConstraintType, CostType> {
        FBSOptimizer {
            fbs_engine: fbs_engine,
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
    ) -> &mut FBSOptimizer<'a, GradientType, ConstraintType, CostType> {
        assert!(tolerance > 0.0);

        self.fbs_engine.cache.tolerance = tolerance;
        self
    }

    /// Sets the maximum number of iterations
    pub fn with_max_iter(
        &mut self,
        max_iter: usize,
    ) -> &mut FBSOptimizer<'a, GradientType, ConstraintType, CostType> {
        self.max_iter = max_iter;
        self
    }
}

impl<'life, GradientType, ConstraintType, CostType> Optimizer
    for FBSOptimizer<'life, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32 + 'life,
    CostType: Fn(&[f64], &mut f64) -> i32,
    ConstraintType: constraints::Constraint + 'life,
{
    fn solve(&mut self, u: &mut [f64]) -> SolverStatus {
        self.fbs_engine.init(u);
        let mut num_iter: usize = 0;
        while self.fbs_engine.step(u) && num_iter < self.max_iter {
            num_iter += 1;
        }

        // cost at the solution
        let mut cost_value = 0.0;

        assert_eq!(
            0,
            (self.fbs_engine.problem.cost)(u, &mut cost_value),
            "The computation of the cost value at the solution failed"
        );

        // export solution status
        SolverStatus::new(
            num_iter < self.max_iter,
            num_iter,
            self.fbs_engine.cache.norm_fpr,
            cost_value,
        )
    }
}
