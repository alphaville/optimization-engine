use crate::{
    constraints,
    continuation::ContinuationMode,
    core::{panoc, Optimizer},
    SolverError,
};

/// Homotopy problem definition
///
///
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
    /// gradient of paramtric cost function, df(u; p)
    pub(crate) parametric_gradient: ParametricGradientType,
    /// parametric cost function, f(u; p)
    pub(crate) parametric_cost: ParametricCostType,
    /// penalty function, c(; p)
    pub(crate) penalty_function: ParametricPenaltyFunctionType,
    /// indices of elements of c(u; p) on which to apply continuation
    pub(crate) idx: Vec<usize>,
    /// initial value of continuation
    pub(crate) from: Vec<f64>,
    /// fianl value of continuation
    pub(crate) to: Vec<f64>,
    /// transition mode of continuation
    pub(crate) transition_mode: Vec<ContinuationMode>,
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
    pub fn new(
        constraints: ConstraintType,
        parametric_gradient: ParametricGradientType,
        parametric_cost: ParametricCostType,
        penalty_function: ParametricPenaltyFunctionType,
        num_penalty_constraints: usize,
    ) -> HomotopyProblem<
        ParametricPenaltyFunctionType,
        ParametricGradientType,
        ConstraintType,
        ParametricCostType,
    > {
        HomotopyProblem {
            constraints: constraints,
            parametric_gradient: parametric_gradient,
            parametric_cost: parametric_cost,
            penalty_function: penalty_function,
            idx: Vec::new(),
            from: Vec::new(),
            to: Vec::new(),
            transition_mode: Vec::new(),
            num_penalty_constraints: num_penalty_constraints,
        }
    }

    pub fn add_continuation(
        &mut self,
        idx_: usize,
        from_: f64,
        to_: f64,
        transition_: ContinuationMode,
    ) {
        self.idx.push(idx_);
        self.from.push(from_);
        self.to.push(to_);
        self.transition_mode.push(transition_);
    }

    pub fn add_continuations(
        &mut self,
        idx_: &[usize],
        from_: &[f64],
        to_: &[f64],
        transition_: &[ContinuationMode],
    ) {
        self.idx.extend(idx_);
        self.from.extend(from_);
        self.to.extend(to_);
        self.transition_mode.extend(transition_);
    }
}
