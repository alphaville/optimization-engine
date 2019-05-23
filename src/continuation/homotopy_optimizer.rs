use crate::{
    constraints,
    core::{panoc, Optimizer},
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
    penalty_function: ParametricPenaltyFunctionType,
    parametric_gradient: ParametricGradientType,
    parametric_cost: ParametricCostType,
    constraints: ConstraintType,
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
        penalty_function: ParametricPenaltyFunctionType,
        parametric_gradient: ParametricGradientType,
        parametric_cost: ParametricCostType,
        constraints: ConstraintType,
    ) -> HomotopyOptimizer<
        ParametricPenaltyFunctionType,
        ParametricGradientType,
        ConstraintType,
        ParametricCostType,
    > {
        HomotopyOptimizer {
            penalty_function: penalty_function,
            parametric_gradient: parametric_gradient,
            parametric_cost: parametric_cost,
            constraints: constraints,
        }
    }
}

// Note: PenaltyFunction: c(u, params, result) -> Result.

/* --------------------------------------------------------------------------------------------- */
/*       TESTS                                                                                   */
/* --------------------------------------------------------------------------------------------- */
#[cfg(test)]
mod tests {

    use crate::continuation;
    use crate::core::constraints::Ball2;
    use crate::core::panoc::*;
    use crate::core::*;
    use crate::{mocks, Error};
    use std::num::NonZeroUsize;

    #[test]
    fn t_homotopy_basic() -> Result<(), Error> {
        /* USER PARAMETERS */
        let df = |u: &[f64], p: &[f64], grad: &mut [f64]| -> Result<(), Error> {
            mocks::rosenbrock_grad(p[0], p[1], u, grad);
            Ok(())
        };

        let f = |u: &[f64], p: &[f64], cost: &mut f64| -> Result<(), Error> {
            *cost = mocks::rosenbrock_cost(p[0], p[1], u);
            Ok(())
        };

        let cp = |u: &[f64], p: &[f64], constraints: &mut [f64]| -> Result<(), Error> {
            constraints[0] = 0.0;
            Ok(())
        };

        let bounds = Ball2::new(None, 2.0);

        let homotopy_optimizer = continuation::HomotopyOptimizer::new(cp, df, f, bounds);

        Ok(())
    }

}
