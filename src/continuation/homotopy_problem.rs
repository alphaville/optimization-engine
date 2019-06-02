use crate::{constraints, continuation::ContinuationMode, SolverError};

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

    /// Adds a continuation directive
    ///
    /// ## Arguments
    ///
    /// - `idx`: indes of the vector c(u; p) on which continuation should be
    ///   applied
    /// - `from`: starting value
    /// - `to`: target value
    /// - `transition`: transition type (see ContinuationMode)
    pub fn add_continuation(
        &mut self,
        idx: usize,
        from: f64,
        to: f64,
        transition: ContinuationMode,
    ) {
        self.idx.push(idx);
        self.from.push(from);
        self.to.push(to);
        self.transition_mode.push(transition);
    }

    /// Adds multiple continuation directives
    ///
    /// ## Arguments
    ///
    /// - `idx`: indices of the vector c(u; p) on which continuation should be
    ///   applied
    /// - `from`: starting values
    /// - `to`: target values
    /// - `transition`: transition types (see ContinuationMode)
    ///
    pub fn add_continuations(
        &mut self,
        idx: &[usize],
        from: &[f64],
        to: &[f64],
        transition: &[ContinuationMode],
    ) {
        self.idx.extend(idx);
        self.from.extend(from);
        self.to.extend(to);
        self.transition_mode.extend(transition);
    }
}
