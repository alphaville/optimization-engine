use crate::continuation;
use crate::core::constraints::Ball2;
use crate::core::panoc::*;
use crate::core::*;
use crate::{mocks, SolverError};
use icasadi_test;

const NU: usize = 5; // Number of decision variables
const NP: usize = 2; // Number of parameters
const NCP: usize = 3; // Number of additional penalty-type constraint-related parameters (y)
const INITIAL_PENALTY_WEIGHTS: &[f64] = &[2.0; self::NCP]; // Initial values of the weights
const XMIN: Option<&[f64]> = Some(&[-2.0, -1.0, -1.0, -1.0, -4.0]);
const XMAX: Option<&[f64]> = Some(&[1.0, 3.0, 1.0, 1.0, 4.0]);
const LBFGS_MEM: usize = 10;

/// Initialisation of the solver
pub fn initialize_solver(inner_tolerance: f64) -> PANOCCache {
    let panoc_cache = PANOCCache::new(NU, inner_tolerance, LBFGS_MEM);
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

    /* cost function, f(u; q) */
    let cost_function = |u: &[f64], q: &[f64], cost: &mut f64| -> Result<(), SolverError> {
        icasadi_test::icasadi_cost(u, q, cost);
        Ok(())
    };

    /* parametric gradient, df(u, q) */
    let gradient_function = |u: &[f64], q: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
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
    let problem_size = 2;
    let lbfgs_memory_size = 10;
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
fn t_homotopy_rosenbrock_convergent() {
    let constraint_tolerance = 1e-3;
    let max_outer_iterations = 30;
    let mut u = [-1.0, -1.0, -1.0, -1.0, 0.0];
    let p = [1.0, 100.0];
    let mut cache = initialize_solver(1e-5);
    let out = solve(
        &p,
        &mut cache,
        &mut u,
        100000000,
        constraint_tolerance,
        2.0,
        max_outer_iterations,
        500,
    );
    let status = out.unwrap();
    assert_eq!(status.exit_status(), ExitStatus::Converged);
    assert!(status.max_constraint_violation() <= constraint_tolerance);
    assert!(status.num_outer_iterations() <= max_outer_iterations);
}

#[test]
fn t_homotopy_rosenbrock_convergent2() {
    let mut u = [1.0, 5.0, 10.0, 100.0, 1000.0];
    let p = [1.0, 1000.0];
    let mut cache = initialize_solver(1e-6);
    let status = solve(&p, &mut cache, &mut u, 500000, 1e-5, 10.0, 30, 500);
    println!("status : {:#?}", &status);
    assert_eq!(status.unwrap().exit_status(), ExitStatus::Converged);
}

#[test]
fn t_homotopy_rosenbrock_convergent_in_loop() {
    let mut u = [1.0, 5.0, 10.0, 100.0, 1000.0];
    let inner_tolerance = 1e-7;
    let constraint_violation_tolerance = 1e-5;
    let mut cache = initialize_solver(inner_tolerance);

    for _i in 1..20 {
        let p = [1.0, 100. + 50. * (_i as f64)];
        let out = solve(
            &p,
            &mut cache,
            &mut u,
            500000,
            constraint_violation_tolerance,
            10.0,
            30,
            500,
        );
        let status = out.unwrap();
        assert_eq!(status.exit_status(), ExitStatus::Converged);
        assert!(status.last_problem_norm_fpr() < inner_tolerance);
        assert!(status.max_constraint_violation() < constraint_violation_tolerance);
    }
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
