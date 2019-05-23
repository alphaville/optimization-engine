#![allow(unused_attributes, dead_code, unused_imports)] //TODO: WORK IN PROGRESS - Remove when done!

mod homotopy_optimizer;
mod homotopy_problem;

pub use homotopy_problem::HomotopyProblem;

/* ---------------------------------------------------------------------------- */
/*          TESTS                                                               */
/* ---------------------------------------------------------------------------- */
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
        let tolerance = 1e-14;
        let np: usize = 3;
        let problem_size = NonZeroUsize::new(2).unwrap();
        let lbfgs_memory_size = NonZeroUsize::new(10).unwrap();

        /* parametric cost */
        let f = |u: &[f64], p: &[f64], cost: &mut f64| -> Result<(), Error> {
            *cost = mocks::rosenbrock_cost(p[0], p[1], u);
            Ok(())
        };

        /* parametric gradient */
        let df = |u: &[f64], p: &[f64], grad: &mut [f64]| -> Result<(), Error> {
            mocks::rosenbrock_grad(p[0], p[1], u, grad);
            Ok(())
        };

        /* parametric constraints, c(u; p) */
        let cp = |u: &[f64], p: &[f64], constraints: &mut [f64]| -> Result<(), Error> {
            let t = crate::matrix_operations::norm2(u);
            constraints[0] = if t < 1. { 0. } else { p[0] * (t - 1.) };
            Ok(())
        };

        let bounds = Ball2::new(None, 2.0);

        let mut homotopy_problem = continuation::HomotopyProblem::new(bounds, df, f, cp, np);
        homotopy_problem.add_continuation(0, 1., 1000., 0);
        homotopy_problem.add_continuation(2, 1., std::f64::INFINITY, 0);

        let mut panoc_cache = PANOCCache::new(problem_size, tolerance, lbfgs_memory_size);

        let mut u_: [f64; 2] = [0.0, 0.0];
        {
            let p_ = [1., 2., 3.];
            let f_ = |u: &[f64], cost: &mut f64| -> Result<(), Error> { f(u, &p_, cost) };
            let df_ = |u: &[f64], grad: &mut [f64]| -> Result<(), Error> { df(u, &p_, grad) };

            let problem_ = Problem::new(homotopy_problem.constraints, df_, f_);
            let mut panoc = PANOCOptimizer::new(problem_, &mut panoc_cache);

            panoc.solve(&mut u_);
            println!("u = {:?}", u_);
        }

        Ok(())
    }

}
