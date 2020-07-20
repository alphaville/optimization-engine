//! An optimization problem
//!
//! This struct defines an optimization problem in terms of its cost function
//! (cost function and its gradient) and constraints
//!
//! Cost functions are user defined. They can either be defined in Rust or in
//! C (and then invoked from Rust via an interface such as icasadi).
//!
use crate::{constraints, FunctionCallResult};

/// Definition of an optimisation problem
///
/// The definition of an optimisation problem involves:
/// - the gradient of the cost function
/// - the cost function
/// - the set of constraints, which is described by implementations of
///   [Constraint](../../panoc_rs/constraints/trait.Constraint.html)
pub struct Problem<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> FunctionCallResult,
    CostType: Fn(&[f64], &mut f64) -> FunctionCallResult,
    ConstraintType: constraints::Constraint,
{
    /// constraints
    pub(crate) constraints: &'a ConstraintType,
    /// gradient of the cost
    pub(crate) gradf: GradientType,
    /// cost function
    pub(crate) cost: CostType,
}

impl<'a, GradientType, ConstraintType, CostType> Problem<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> FunctionCallResult,
    CostType: Fn(&[f64], &mut f64) -> FunctionCallResult,
    ConstraintType: constraints::Constraint,
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
    ) -> Problem<'a, GradientType, ConstraintType, CostType> {
        Problem {
            constraints,
            gradf: cost_gradient,
            cost,
        }
    }
}
