#![allow(unused_attributes, dead_code, unused_imports)] //TODO: WORK IN PROGRESS - Remove when done!

use crate::{
    constraints,
    core::{panoc, Optimizer},
    Error,
};

pub struct HomotopyProblem<
    ParametricPenaltyFunctionType,
    ParametricGradientType,
    ConstraintType,
    ParametricCostType,
> where
    ParametricPenaltyFunctionType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), Error>,
    ParametricGradientType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), Error>,
    ParametricCostType: Fn(&[f64], &[f64], &mut f64) -> Result<(), Error>,
    ConstraintType: constraints::Constraint,
{
    pub(crate) constraints: ConstraintType,
    pub(crate) parametric_gradient: ParametricGradientType,
    pub(crate) parametric_cost: ParametricCostType,
    pub(crate) penalty_function: ParametricPenaltyFunctionType,
    idx: Vec<usize>,
    from: Vec<f64>,
    to: Vec<f64>,
    transition_mode: Vec<i32>,
    num_parameters: usize,
}

impl<ParametricPenaltyFunctionType, ParametricGradientType, ConstraintType, ParametricCostType>
    HomotopyProblem<
        ParametricPenaltyFunctionType,
        ParametricGradientType,
        ConstraintType,
        ParametricCostType,
    >
where
    ParametricPenaltyFunctionType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), Error>,
    ParametricGradientType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), Error>,
    ParametricCostType: Fn(&[f64], &[f64], &mut f64) -> Result<(), Error>,
    ConstraintType: constraints::Constraint,
{
    pub fn new(
        constraints: ConstraintType,
        parametric_gradient: ParametricGradientType,
        parametric_cost: ParametricCostType,
        penalty_function: ParametricPenaltyFunctionType,
        num_parameters: usize,
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
            num_parameters: num_parameters,
        }
    }

    pub fn add_continuation(&mut self, idx_: usize, from_: f64, to_: f64, transition_: i32) {
        assert!(idx_ < self.num_parameters, "idx_ is out of bounds");
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
        transition_: &[i32],
    ) {
        self.idx.extend(idx_);
        self.from.extend(from_);
        self.to.extend(to_);
        self.transition_mode.extend(transition_);
    }
}

// Note: PenaltyFunction: c(u, params, result) -> Result.
