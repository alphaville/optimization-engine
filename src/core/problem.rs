use crate::constraints;
pub use crate::core::Problem;

impl<GradientType, ConstraintType, CostType> Problem<GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32,
    CostType: Fn(&[f64], &mut f64) -> i32,
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
    pub fn new(
        constraints: ConstraintType,
        cost_gradient: GradientType,
        cost: CostType,
    ) -> Problem<GradientType, ConstraintType, CostType> {
        Problem {
            constraints: constraints,
            gradf: cost_gradient,
            cost: cost,
        }
    }
}
