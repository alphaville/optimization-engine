use crate::{constraints, core::panoc, Error};

pub struct HomotopyOptimizer<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> Result<(), Error>,
    CostType: Fn(&[f64], &mut f64) -> Result<(), Error>,
    ConstraintType: constraints::Constraint,
{
    panoc: &'a mut panoc::PANOCOptimizer<'a, GradientType, ConstraintType, CostType>,
}

impl<'a, GradientType, ConstraintType, CostType>
    HomotopyOptimizer<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> Result<(), Error>,
    CostType: Fn(&[f64], &mut f64) -> Result<(), Error>,
    ConstraintType: constraints::Constraint,
{
    pub fn new(
        panoc: &'a mut panoc::PANOCOptimizer<'a, GradientType, ConstraintType, CostType>,
    ) -> HomotopyOptimizer<'a, GradientType, ConstraintType, CostType> {
        HomotopyOptimizer { panoc: panoc }
    }
}

/* --------------------------------------------------------------------------------------------- */
/*       TESTS                                                                                   */
/* --------------------------------------------------------------------------------------------- */
#[cfg(test)]
mod tests {

    use crate::continuation::*;
    use crate::core::panoc::*;
    use crate::core::*;
    use crate::{mocks, Error};
    use std::num::NonZeroUsize;

    #[test]
    fn t_homotopy_basic() {
        /* USER PARAMETERS */
        let tolerance = 1e-6;
        let a = 1.0;
        let b = 200.0;
        let n = 2;
        let lbfgs_memory = 8;
        let max_iters = 80;
        let mut u = [-1.5, 0.9];

        /* COST FUNCTION */
        let df = |u: &[f64], grad: &mut [f64]| -> Result<(), Error> {
            mocks::rosenbrock_grad(a, b, u, grad);
            Ok(())
        };
        let f = |u: &[f64], c: &mut f64| -> Result<(), Error> {
            *c = mocks::rosenbrock_cost(a, b, u);
            Ok(())
        };
        /* CONSTRAINTS */
        let radius = 2.0;
        let bounds = constraints::Ball2::new(None, radius);
        let mut panoc_cache = PANOCCache::new(
            NonZeroUsize::new(n).unwrap(),
            tolerance,
            NonZeroUsize::new(lbfgs_memory).unwrap(),
        );
        let problem = Problem::new(bounds, df, f);
        let mut panoc = PANOCOptimizer::new(problem, &mut panoc_cache).with_max_iter(max_iters);
        let homotopy_optimizer = homotopy_optimizer::HomotopyOptimizer::new(&mut panoc);
    }
}
