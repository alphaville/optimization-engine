#![allow(unused_attributes, dead_code, unused_imports)] //TODO: WORK IN PROGRESS - Remove when done!

use crate::{
    constraints,
    continuation::homotopy_problem::HomotopyProblem,
    core::panoc::*,
    core::{panoc, Optimizer, Problem},
    Error,
};

pub struct HomotopyOptimizer<
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
    homotopy_problem: HomotopyProblem<
        ParametricPenaltyFunctionType,
        ParametricGradientType,
        ConstraintType,
        ParametricCostType,
    >,
    panoc_cache: panoc::PANOCCache,
}

impl<ParametricPenaltyFunctionType, ParametricGradientType, ConstraintType, ParametricCostType>
    HomotopyOptimizer<
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
        homotopy_problem: HomotopyProblem<
            ParametricPenaltyFunctionType,
            ParametricGradientType,
            ConstraintType,
            ParametricCostType,
        >,
        panoc_cache: panoc::PANOCCache,
    ) -> HomotopyOptimizer<
        ParametricPenaltyFunctionType,
        ParametricGradientType,
        ConstraintType,
        ParametricCostType,
    > {
        HomotopyOptimizer {
            homotopy_problem: homotopy_problem,
            panoc_cache: panoc_cache,
        }
    }

    pub fn solve(&mut self, u: &mut [f64]) {
        let p_ = [1., 2., 3.];
        let f_ = |u: &[f64], cost: &mut f64| -> Result<(), Error> {
            (self.homotopy_problem.parametric_cost)(u, &p_, cost)
        };
        let df_ = |u: &[f64], grad: &mut [f64]| -> Result<(), Error> {
            (self.homotopy_problem.parametric_gradient)(u, &p_, grad)
        };
        let prob = crate::core::Problem::new(&self.homotopy_problem.constraints, df_, f_);
        
    }
}
