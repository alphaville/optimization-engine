//! Continuation/Homotopy methods for parametric optimization problems
//!
//! Consider a parametric optimization problem of the general form
//!
//! ```txt
//! Minimize_u f(u; p)
//! subject to: u in U(p)
//! ```
//!
//! Suppose that for a particular value of `p`, this problem is ill
//! conditioned.
//!
//! It then makes sense to solve the problem for a sequence of values
//! `p_k`, which converge to the target value `p`, starting from an
//! initial value `p_0` and using each solution as a warm start for the
//! next optimization problem.
//!
//! The target value of `p` can be equal to infinity. In that case we
//! have the so-called "penalty method".
//!
//! The above parametric problem is accompanied by a function `c(u; p)`,
//! used as a termination criterion. In particular, when `c(u; p)`
//! drops below a desired tolerance, then the iterative procedure
//! is terminated.
//!

//TODO: WORK IN PROGRESS - Remove when done!
#![allow(unused_attributes, dead_code, unused_imports)]

mod homotopy_optimizer;
mod homotopy_problem;
mod homotopy_solver_status;

pub use homotopy_optimizer::HomotopyOptimizer;
pub use homotopy_problem::HomotopyProblem;
pub use homotopy_solver_status::HomotopySolverStatus;

/// Continuation mode for free parameters
#[derive(Debug, Clone, Copy)]
pub enum ContinuationMode {
    /// Arithmetic progression
    ///
    /// A free parameter with initial value `x: f64` and target value
    /// `y: f64` is updated by adding a constant parameter `s: f64`.
    /// Parameter `y` is allowed to be `std::f64::INFINITY`. If `y` is
    /// finite, the continuation will terminate in a finite number of
    /// steps.
    ///
    Arithmetic(f64),
    /// Convex combination
    ///
    /// Can only be applied when the initial and final values, `x` and
    /// `y`, are finite. The update is a convex combination of the current
    /// value and `y`
    Convex(f64),
    /// Is typically used when `y` is plus or minus infinity or zero.
    /// Then, the parameter is updated by multiplying by a given factor.
    Geometric(f64),
}

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
        let problem_size = NonZeroUsize::new(2).unwrap();
        let lbfgs_memory_size = NonZeroUsize::new(10).unwrap();
        let a = 1.0;

        /* parametric cost */
        let f = |u: &[f64], p: &[f64], cost: &mut f64| -> Result<(), Error> {
            *cost = mocks::rosenbrock_cost(a, p[1], u);
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

        let bounds = Ball2::new(None, 10.0);

        let mut panoc_cache = PANOCCache::new(problem_size, tolerance, lbfgs_memory_size);

        let mut homotopy_problem = continuation::HomotopyProblem::new(bounds, df, f, cp, 2);
        homotopy_problem.add_continuations(
            &[1, 2],
            &[1.; 2],
            &[1000.; 2],
            &[continuation::ContinuationMode::Geometric(5.0); 2],
        );

        let mut homotopy_optimizer =
            continuation::HomotopyOptimizer::new(&homotopy_problem, &mut panoc_cache);

        let mut u_: [f64; 2] = [1.0, 1.0];
        let p_: [f64; 3] = [1., 1., 1.];
        homotopy_optimizer.solve(&mut u_, &p_)?;
        Ok(())
    }

}