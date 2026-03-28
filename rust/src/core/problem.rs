//! An optimization problem
//!
//! This struct defines an optimization problem in terms of its cost function
//! (cost function and its gradient) and constraints
//!
//! Cost functions are user defined. They can either be defined in Rust or in
//! C (and then invoked from Rust via an interface such as icasadi).
//!
use crate::{constraints, FunctionCallResult};
use std::marker::PhantomData;

/// Definition of an optimisation problem
///
/// The definition of an optimisation problem involves:
/// - the gradient of the cost function
/// - the cost function
/// - the set of constraints, which is described by implementations of
///   [Constraint](../../panoc_rs/constraints/trait.Constraint.html)
pub struct Problem<'a, GradientType, ConstraintType, CostType, T = f64>
where
    GradientType: Fn(&[T], &mut [T]) -> FunctionCallResult,
    CostType: Fn(&[T], &mut T) -> FunctionCallResult,
    ConstraintType: constraints::Constraint<T>,
{
    /// constraints
    pub(crate) constraints: &'a ConstraintType,
    /// gradient of the cost
    pub(crate) gradf: GradientType,
    /// cost function
    pub(crate) cost: CostType,
    marker: PhantomData<T>,
}

impl<'a, GradientType, ConstraintType, CostType, T>
    Problem<'a, GradientType, ConstraintType, CostType, T>
where
    GradientType: Fn(&[T], &mut [T]) -> FunctionCallResult,
    CostType: Fn(&[T], &mut T) -> FunctionCallResult,
    ConstraintType: constraints::Constraint<T>,
{
    /// Construct a new instance of an optimisation problem
    ///
    /// ## Arguments
    ///
    /// - `constraints` constraints
    /// - `cost_gradient` gradient of the cost function
    /// - `cost` cost function
    ///
    /// ## Returns
    ///
    /// New instance of `Problem`
    pub fn new(
        constraints: &'a ConstraintType,
        cost_gradient: GradientType,
        cost: CostType,
    ) -> Problem<'a, GradientType, ConstraintType, CostType, T> {
        Problem {
            constraints,
            gradf: cost_gradient,
            cost,
            marker: PhantomData,
        }
    }
}
