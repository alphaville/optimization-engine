//! FBS Algorithm
//!
use crate::{
    constraints,
    core::{
        fbs::fbs_engine::FBSEngine, fbs::FBSCache, AlgorithmEngine, ExitStatus, Optimizer, Problem,
        SolverStatus,
    },
    SolverError,
};
use std::time;

const MAX_ITER: usize = 100_usize;

/// Optimiser using forward-backward splitting iterations (projected gradient)
///
/// Note that an `FBSOptimizer` holds a reference to an instance of `FBSEngine`
/// which needs to be created externally. A mutable reference to that `FBSEgnine`
/// is provided to the optimizer.
///
/// The `FBSEngine` is supposed to be updated whenever you need to solve
/// a different optimization problem.
///
///
pub struct FBSOptimizer<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> Result<(), SolverError>,
    CostType: Fn(&[f64], &mut f64) -> Result<(), SolverError>,
    ConstraintType: constraints::Constraint,
{
    fbs_engine: FBSEngine<'a, GradientType, ConstraintType, CostType>,
    max_iter: usize,
    max_duration: Option<time::Duration>,
}

impl<'a, GradientType, ConstraintType, CostType>
    FBSOptimizer<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> Result<(), SolverError>,
    CostType: Fn(&[f64], &mut f64) -> Result<(), SolverError>,
    ConstraintType: constraints::Constraint,
{
    pub fn new(
        problem: Problem<'a, GradientType, ConstraintType, CostType>,
        cache: &'a mut FBSCache,
    ) -> FBSOptimizer<'a, GradientType, ConstraintType, CostType> {
        FBSOptimizer {
            fbs_engine: FBSEngine::new(problem, cache),
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
    ) -> FBSOptimizer<'a, GradientType, ConstraintType, CostType> {
        assert!(tolerance > 0.0);

        self.fbs_engine.cache.tolerance = tolerance;
        self
    }

    /// Sets the maximum number of iterations
    pub fn with_max_iter(
        mut self,
        max_iter: usize,
    ) -> FBSOptimizer<'a, GradientType, ConstraintType, CostType> {
        self.max_iter = max_iter;
        self
    }

    /// Sets the maximum number of iterations
    pub fn with_max_duration(
        mut self,
        max_duration: time::Duration,
    ) -> FBSOptimizer<'a, GradientType, ConstraintType, CostType> {
        self.max_duration = Some(max_duration);
        self
    }
}

impl<'life, GradientType, ConstraintType, CostType> Optimizer
    for FBSOptimizer<'life, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> Result<(), SolverError> + 'life,
    CostType: Fn(&[f64], &mut f64) -> Result<(), SolverError> + 'life,
    ConstraintType: constraints::Constraint + 'life,
{
    fn solve(&mut self, u: &mut [f64]) -> Result<SolverStatus, SolverError> {
        let now = time::Instant::now();

        if let Ok(_) = self.fbs_engine.init(u) {
            let mut num_iter: usize = 0;
            if let Some(dur) = self.max_duration {
                while self.fbs_engine.step(u) == Ok(true)
                    && num_iter < self.max_iter
                    && dur <= now.elapsed()
                {
                    num_iter += 1;
                }
            } else {
                while self.fbs_engine.step(u) == Ok(true) && num_iter < self.max_iter {
                    num_iter += 1;
                }
            }

            // cost at the solution
            let mut cost_value = 0.0;

            assert_eq!(
                Ok(()),
                (self.fbs_engine.problem.cost)(u, &mut cost_value),
                "The computation of the cost value at the solution failed"
            );

            // export solution status
            Ok(SolverStatus::new(
                if num_iter < self.max_iter {
                    ExitStatus::Converged
                } else {
                    ExitStatus::NotConvergedIterations
                },
                num_iter,
                now.elapsed(),
                self.fbs_engine.cache.norm_fpr,
                cost_value,
            ))
        } else {
            // The cost function failed somewhere
            Ok(SolverStatus::new(
                ExitStatus::NotConvergedIterations,
                0,
                now.elapsed(),
                std::f64::INFINITY,
                std::f64::INFINITY,
            ))
        }
    }
}
