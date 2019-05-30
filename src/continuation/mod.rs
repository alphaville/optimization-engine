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
    use crate::continuation::tests;
    use crate::core::constraints::Ball2;
    use crate::core::panoc::*;
    use crate::core::*;
    use crate::{mocks, SolverError};
    use icasadi_test;
    use std::num::NonZeroUsize;

    const NU: usize = 5; // Number of decision variables
    const NP: usize = 2; // Number of parameters
    const NCP: usize = 3; // Number of additional penalty-type constraint-related parameters (y)
    const INITIAL_PENALTY_WEIGHTS: &[f64] = &[2.0; self::NCP]; // Initial values of the weights
    const XMIN: Option<&[f64]> = Some(&[-2.0, -1.0, -1.0, -1.0, -4.0]);
    const XMAX: Option<&[f64]> = Some(&[1.0, 3.0, 1.0, 1.0, 4.0]);

    /// Initialisation of the solver
    pub fn initialize_solver(inner_tolerance: f64) -> PANOCCache {
        let problem_size = NonZeroUsize::new(self::NU).unwrap();
        let lbfgs_memory_size = NonZeroUsize::new(10).unwrap();
        let panoc_cache = PANOCCache::new(problem_size, inner_tolerance, lbfgs_memory_size);

        panoc_cache
    }

    pub fn solve(
        p: &[f64],
        cache: &mut PANOCCache,
        u: &mut [f64],
        max_duration_micros: u64,
        constraints_tolerance: f64,
        penalty_weight_update_factor: f64,
        max_outer_iterations: usize,
        max_inner_iterations: usize,
    ) -> Result<continuation::HomotopySolverStatus, SolverError> {
        let mut q_augmented_params = [0.0; self::NP + self::NCP];
        q_augmented_params[0..self::NP].copy_from_slice(p);

        //TODO: better error management (in closures)
        /* cost function, f(u; q) */
        let cost_function = |u: &[f64], q: &[f64], cost: &mut f64| -> Result<(), SolverError> {
            icasadi_test::icasadi_cost(u, q, cost);
            Ok(())
        };

        /* parametric gradient, df(u, q) */
        let gradient_function =
            |u: &[f64], q: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
                icasadi_test::icasadi_grad(u, q, grad);
                Ok(())
            };

        /* penalty-type constraints: c(u; p) */
        let penalty_constr_function =
            |u: &[f64], q: &[f64], constraints: &mut [f64]| -> Result<(), SolverError> {
                icasadi_test::icasadi_constraints_as_penalty(u, q, constraints);
                Ok(())
            };

        // Constraints...
        let bounds = crate::constraints::Rectangle::new(self::XMIN, self::XMAX);

        // Define homotopy problem
        let mut homotopy_problem = continuation::HomotopyProblem::new(
            bounds,
            gradient_function,
            cost_function,
            penalty_constr_function,
            self::NCP,
        );

        // Define the initial weights, the update rule and the update factor
        let idx_y: Vec<usize> = (self::NP..self::NP + self::NCP).collect();
        homotopy_problem.add_continuations(
            &idx_y[..],
            self::INITIAL_PENALTY_WEIGHTS,
            &[0.; self::NCP],
            &[continuation::ContinuationMode::Geometric(penalty_weight_update_factor); self::NCP],
        );

        // construct a homotopy optimizer
        let mut homotopy_optimizer = continuation::HomotopyOptimizer::new(&homotopy_problem, cache)
            .with_constraint_tolerance(constraints_tolerance)
            .with_max_outer_iterations(max_outer_iterations)
            .with_max_inner_iterations(max_inner_iterations);

        // set the maximum execution duration
        homotopy_optimizer.with_max_duration(std::time::Duration::from_micros(max_duration_micros));

        // solve the problem and return its status
        // parameter `u` is updated with the solution
        homotopy_optimizer.solve(u, &q_augmented_params)
    }

    #[test]
    fn t_homotopy_basic() -> Result<(), SolverError> {
        let tolerance = 1e-14;
        let problem_size = NonZeroUsize::new(2).unwrap();
        let lbfgs_memory_size = NonZeroUsize::new(10).unwrap();
        let a = 1.0;

        /* parametric cost */
        let f = |u: &[f64], p: &[f64], cost: &mut f64| -> Result<(), SolverError> {
            *cost = mocks::rosenbrock_cost(a, p[1], u);
            Ok(())
        };

        /* parametric gradient */
        let df = |u: &[f64], p: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
            mocks::rosenbrock_grad(p[0], p[1], u, grad);
            Ok(())
        };

        /* parametric constraints, c(u; p) */
        let cp = |u: &[f64], p: &[f64], constraints: &mut [f64]| -> Result<(), SolverError> {
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

    #[test]
    fn t_homotopy_rosenbrock() {
        let mut u = [-1.0, -1.0, -1.0, -1.0, 0.0];
        let p = [1.0, 100.0];
        let mut cache = initialize_solver(1e-5);
        let status = solve(&p, &mut cache, &mut u, 100000000, 1e-3, 2.0, 30, 500);
        assert_eq!(status.unwrap().exit_status(), ExitStatus::Converged);
    }

    #[test]
    fn t_homotopy_rosenbrock_outta_time() {
        let mut u = [-1.0, -1.0, -1.0, -1.0, 0.0];
        let p = [1.0, 100.0];
        let mut cache = initialize_solver(1e-5);
        let status = solve(&p, &mut cache, &mut u, 10, 1e-3, 2.0, 30, 500);
        assert_eq!(
            status.unwrap().exit_status(),
            ExitStatus::NotConvergedOutOfTime
        );
    }

    #[test]
    fn t_homotopy_rosenbrock_outta_iterations() {
        let mut u = [-1.0, -1.0, -1.0, -1.0, 0.0];
        let p = [1.0, 100.0];
        let mut cache = initialize_solver(1e-5);
        let status = solve(&p, &mut cache, &mut u, 10000000, 1e-12, 2.0, 30, 500);
        assert_eq!(
            status.unwrap().exit_status(),
            ExitStatus::NotConvergedIterations
        );
    }

    #[test]
    fn t_homotopy_rosenbrock_outta_iterations_2() {
        let mut u = [-1.0, -1.0, -1.0, -1.0, 0.0];
        let p = [1.0, 100.0];
        let mut cache = initialize_solver(1e-12);
        let status = solve(&p, &mut cache, &mut u, 10000000, 1e-3, 2.0, 30, 500);
        assert_eq!(
            status.unwrap().exit_status(),
            ExitStatus::NotConvergedIterations
        );
    }
}
