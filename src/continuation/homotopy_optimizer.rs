#![allow(unused_attributes, dead_code, unused_imports)] //TODO: WORK IN PROGRESS - Remove when done!

use crate::{
    constraints,
    continuation::homotopy_problem::HomotopyProblem,
    continuation::ContinuationMode,
    core::panoc::*,
    core::{panoc, Optimizer, Problem},
    Error,
};

pub struct HomotopyOptimizer<
    'a,
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
    homotopy_problem: &'a HomotopyProblem<
        ParametricPenaltyFunctionType,
        ParametricGradientType,
        ConstraintType,
        ParametricCostType,
    >,
    panoc_cache: &'a mut panoc::PANOCCache,
}

impl<
        'a,
        ParametricPenaltyFunctionType,
        ParametricGradientType,
        ConstraintType,
        ParametricCostType,
    >
    HomotopyOptimizer<
        'a,
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
        homotopy_problem: &'a HomotopyProblem<
            ParametricPenaltyFunctionType,
            ParametricGradientType,
            ConstraintType,
            ParametricCostType,
        >,
        panoc_cache: &'a mut panoc::PANOCCache,
    ) -> HomotopyOptimizer<
        'a,
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

    fn initialize_param(&self, _p: &mut [f64]) {}

    fn update_continuation_parameters(&self, p_: &mut [f64]) {
        let homotopy_problem = &self.homotopy_problem;
        let idx_list = &homotopy_problem.idx;
        println!("idx_list = {:?}", idx_list);
        let transition_list = &homotopy_problem.transition_mode;
        for (i, transition_mode) in idx_list.iter().zip(transition_list.iter()) {
            match transition_mode {
                ContinuationMode::Arithmetic(s) => p_[*i] += s,
                ContinuationMode::Geometric(s) => p_[*i] *= s,
                _ => (),
            }
        }
        p_[0] *= 2.0;
    }

    /// Solve problem by homotopy method
    ///
    pub fn solve(&'a mut self, u: &mut [f64], q: &[f64]) -> Result<(), Error> {
        //NOTE: Above, by `param` we mean `q`...

        let mut p_: Vec<f64> = q.to_vec();
        // If NCP = 0, there is no need to apply the homotopy method
        // Just run the problem once; think of a way to do this neatly!
        // Maybe, at the end of the loop, do: if NCP == 0: break

        // Another consideration is the total time; the time should be monitored
        // and every next instance of the (inner) solver should be given the
        // time that is left
        for _i in 1..10 {
            let homotopy_problem = &self.homotopy_problem;
            let f_ = |u: &[f64], cost: &mut f64| -> Result<(), Error> {
                (homotopy_problem.parametric_cost)(u, &p_, cost)
            };
            let df_ = |u: &[f64], grad: &mut [f64]| -> Result<(), Error> {
                (homotopy_problem.parametric_gradient)(u, &p_, grad)
            };
            let problem_ = Problem::new(&self.homotopy_problem.constraints, df_, f_);
            let mut panoc_ = panoc::PANOCOptimizer::new(problem_, &mut self.panoc_cache);

            //TODO: check status of inner solver...
            let status = panoc_.solve(u);
            println!("{:#?}", status);

            let mut c = [0.];
            (homotopy_problem.penalty_function)(u, &p_, &mut c)?;
            let continue_outer_iterations = c.iter().any(|&ci| ci > 0.01);
            if !continue_outer_iterations {
                break;
            } else {
                self.update_continuation_parameters(&mut p_);
            }
            println!("updated p = {:?}", p_);
        }
        // Need to return a different type of  SolverStatus (HomotopySolverStatus)
        // with the number of outer iterations, maximum number of inner iterations,
        // total outer time etc
        Ok(())
    }
}
