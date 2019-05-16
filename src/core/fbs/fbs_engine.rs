//! FBS Engine
//!
use super::super::AlgorithmEngine;
use super::FBSCache;
use super::FBSEngine;
use super::Problem;
use crate::constraints;
use crate::matrix_operations;

impl<'a, GradientType, ConstraintType, CostType>
    FBSEngine<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32,
    CostType: Fn(&[f64], &mut f64) -> i32,
    ConstraintType: constraints::Constraint,
{
    /// Constructor for instances of `FBSEngine`
    ///
    /// ## Arguments
    ///
    /// - `problem` problem definition (cost function, gradient of the cost, constraints)
    /// - mutable reference to a `cache` a cache (which is created once); the cache is reuseable
    ///
    /// ## Returns
    ///
    /// An new instance of `FBSEngine`
    pub fn new(
        problem: Problem<GradientType, ConstraintType, CostType>,
        cache: &'a mut FBSCache,
    ) -> FBSEngine<'a, GradientType, ConstraintType, CostType> {
        FBSEngine { problem, cache }
    }

    fn gradient_step(&mut self, u_current: &mut [f64]) {
        assert_eq!(
            0,
            (self.problem.gradf)(u_current, &mut self.cache.work_gradient_u),
            "The computation of the gradient of the cost failed miserably"
        );

        // take a gradient step: u_currect -= gamma * gradient
        u_current
            .iter_mut()
            .zip(self.cache.work_gradient_u.iter())
            .for_each(|(u, w)| *u -= self.cache.gamma * *w);
    }

    fn projection_step(&mut self, u_current: &mut [f64]) {
        self.problem.constraints.project(u_current);
    }
}

impl<'a, GradientType, ConstraintType, CostType> AlgorithmEngine
    for FBSEngine<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32 + 'a,
    CostType: Fn(&[f64], &mut f64) -> i32 + 'a,
    ConstraintType: constraints::Constraint + 'a,
{
    /// Take a forward-backward step and check whether the algorithm should terminate
    ///
    /// ## Arguments
    ///
    /// - `u_current` the current mutable
    ///
    /// ## Returns
    ///
    /// - A boolean flag which is`true` if and only if the algorith should not
    ///   terminate
    ///
    /// ## Panics
    ///
    /// The method may panick if the computation of the gradient of the cost function
    /// or the cost function panics.
    fn step(&mut self, u_current: &mut [f64]) -> bool {
        self.cache.work_u_previous.copy_from_slice(u_current); // cache the previous step
        self.gradient_step(u_current); // compute the gradient
        self.projection_step(u_current); // project
        self.cache.norm_fpr =
            matrix_operations::norm_inf_diff(u_current, &self.cache.work_u_previous);
        self.cache.norm_fpr > self.cache.tolerance
    }

    fn init(&mut self, _u_current: &mut [f64]) {}
}
