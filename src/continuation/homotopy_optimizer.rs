#![allow(unused_attributes, dead_code, unused_imports)] //TODO: WORK IN PROGRESS - Remove when done!

use crate::{
    constraints,
    continuation::homotopy_problem::HomotopyProblem,
    core::panoc::*,
    core::{panoc, Optimizer, Problem},
    Error,
};

pub struct HomotopyOptimizer<
    'cache_lifetime,
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
    panoc_cache: &'cache_lifetime mut panoc::PANOCCache,
}

impl<
        'cache_lifetime,
        ParametricPenaltyFunctionType,
        ParametricGradientType,
        ConstraintType,
        ParametricCostType,
    >
    HomotopyOptimizer<
        'cache_lifetime,
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
        panoc_cache: &'cache_lifetime mut panoc::PANOCCache,
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

    pub fn solve(&'cache_lifetime mut self, u: &mut [f64]) {
        let p_ = [10., 20., 3.];

        let problem = &self.homotopy_problem;
        let f_ = |u: &[f64], cost: &mut f64| -> Result<(), Error> {
            (problem.parametric_cost)(u, &p_, cost)
        };
        let df_ = |u: &[f64], grad: &mut [f64]| -> Result<(), Error> {
            (problem.parametric_gradient)(u, &p_, grad)
        };
        let problem_ = Problem::new(&self.homotopy_problem.constraints, df_, f_);

        let mut panoc_ = panoc::PANOCOptimizer::new(problem_, &mut self.panoc_cache);
        panoc_.solve(u);
        println!("u = {:#?}", u);
    }
}
