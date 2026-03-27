//! FBS Engine
//!
use crate::{
    constraints,
    core::{fbs::FBSCache, AlgorithmEngine, Problem},
    matrix_operations, FunctionCallResult, SolverError,
};
use num::Float;

/// The FBE Engine defines the steps of the FBE algorithm and the termination criterion
///
pub struct FBSEngine<'a, GradientType, ConstraintType, CostType, T = f64>
where
    T: Float,
    GradientType: Fn(&[T], &mut [T]) -> FunctionCallResult,
    CostType: Fn(&[T], &mut T) -> FunctionCallResult,
    ConstraintType: constraints::Constraint<T>,
{
    pub(crate) problem: Problem<'a, GradientType, ConstraintType, CostType, T>,
    pub(crate) cache: &'a mut FBSCache<T>,
}

impl<'a, GradientType, ConstraintType, CostType, T>
    FBSEngine<'a, GradientType, ConstraintType, CostType, T>
where
    T: Float,
    GradientType: Fn(&[T], &mut [T]) -> FunctionCallResult,
    CostType: Fn(&[T], &mut T) -> FunctionCallResult,
    ConstraintType: constraints::Constraint<T>,
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
        problem: Problem<'a, GradientType, ConstraintType, CostType, T>,
        cache: &'a mut FBSCache<T>,
    ) -> FBSEngine<'a, GradientType, ConstraintType, CostType, T> {
        FBSEngine { problem, cache }
    }

    fn gradient_step(&mut self, u_current: &mut [T]) {
        assert_eq!(
            Ok(()),
            (self.problem.gradf)(u_current, &mut self.cache.work_gradient_u),
            "The computation of the gradient of the cost failed miserably"
        );

        // take a gradient step: u_currect -= gamma * gradient
        u_current
            .iter_mut()
            .zip(self.cache.work_gradient_u.iter())
            .for_each(|(u, w)| *u = *u - self.cache.gamma * *w);
    }

    fn projection_step(&mut self, u_current: &mut [T]) {
        self.problem.constraints.project(u_current);
    }
}

impl<'a, GradientType, ConstraintType, CostType, T> AlgorithmEngine<T>
    for FBSEngine<'a, GradientType, ConstraintType, CostType, T>
where
    T: Float,
    GradientType: Fn(&[T], &mut [T]) -> FunctionCallResult + 'a,
    CostType: Fn(&[T], &mut T) -> FunctionCallResult + 'a,
    ConstraintType: constraints::Constraint<T> + 'a,
{
    /// Take a forward-backward step and check whether the algorithm should terminate
    ///
    /// ## Arguments
    ///
    /// - `u_current` the current mutable
    ///
    /// ## Returns
    ///
    /// - A boolean flag which is`true` if and only if the algorithm should not
    ///   terminate
    ///
    /// ## Panics
    ///
    /// The method may panick if the computation of the gradient of the cost function
    /// or the cost function panics.
    fn step(&mut self, u_current: &mut [T]) -> Result<bool, SolverError> {
        self.cache.work_u_previous.copy_from_slice(u_current); // cache the previous step
        self.gradient_step(u_current); // compute the gradient
        self.projection_step(u_current); // project
        self.cache.norm_fpr =
            matrix_operations::norm_inf_diff(u_current, &self.cache.work_u_previous);

        Ok(self.cache.norm_fpr > self.cache.tolerance)
    }

    fn init(&mut self, _u_current: &mut [T]) -> FunctionCallResult {
        Ok(())
    }
}
