#![deny(missing_docs)]
use crate::{constraints, SolverError};

/// Homotopy problem definition
///
/// Definition of a parametric optimization problem to be solved with
/// the homotopy method
pub struct HomotopyProblem<
    ParametricPenaltyFunctionType,
    ParametricGradientType,
    ConstraintType,
    ParametricCostType,
> where
    ParametricPenaltyFunctionType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricGradientType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricCostType: Fn(&[f64], &[f64], &mut f64) -> Result<(), SolverError>,
    ConstraintType: constraints::Constraint,
{
    /// constraints
    pub(crate) constraints: ConstraintType,
    /// gradient of parametric cost function, df(u; p)
    pub(crate) parametric_gradient: ParametricGradientType,
    /// parametric cost function, f(u; p)
    pub(crate) parametric_cost: ParametricCostType,
    /// penalty function, c(; p)
    pub(crate) penalty_function: ParametricPenaltyFunctionType,
    /// number of penalty constraints (dimension of range of c(u;p))
    pub(crate) num_penalty_constraints: usize,
}

impl<ParametricPenaltyFunctionType, ParametricGradientType, ConstraintType, ParametricCostType>
    HomotopyProblem<
        ParametricPenaltyFunctionType,
        ParametricGradientType,
        ConstraintType,
        ParametricCostType,
    >
where
    ParametricPenaltyFunctionType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricGradientType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricCostType: Fn(&[f64], &[f64], &mut f64) -> Result<(), SolverError>,
    ConstraintType: constraints::Constraint,
{
    /// Constructs instances of Homotopy problem
    ///
    /// ## Arguments
    ///
    /// - `constraints`: hard constraints
    /// - `parametric_gradient`: gradient of the parametric cost function, df(u; p)
    /// - `parametric_cost`: parametric cost function, f(u; p)
    /// - `penalty_function`: parametric penalty function: c(u; p)
    /// - `num_penalty_constraints`: number of penalty-type constraints. This is the
    ///   range dimension of c(u; p)
    ///
    pub fn new(
        constraints: ConstraintType,
        parametric_gradient: ParametricGradientType,
        parametric_cost: ParametricCostType,
        penalty_function: ParametricPenaltyFunctionType,
        num_penalty_constraints: usize,
    ) -> Self {
        HomotopyProblem {
            constraints,
            parametric_gradient,
            parametric_cost,
            penalty_function,
            num_penalty_constraints,
        }
    }
}
