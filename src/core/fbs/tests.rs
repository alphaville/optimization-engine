use super::super::*;
use super::*;
use crate::constraints;
use crate::core::fbs::fbs_engine::FBSEngine;
use std::num::NonZeroUsize;

const N_DIM: usize = 2;

#[cfg(test)]
use crate::mocks;

#[test]
fn t_solve_fbs_hard() {
    let bounds = constraints::NoConstraints::new();

    let problem = Problem::new(
        &bounds,
        mocks::hard_quadratic_gradient,
        mocks::hard_quadratic_cost,
    );
    let gamma = 0.0005;
    let tolerance = 1e-6;

    let mut fbs_cache = FBSCache::new(NonZeroUsize::new(3).unwrap(), gamma, tolerance);
    let mut u = [-12., -160., 55.];
    let mut optimizer = FBSOptimizer::new(problem, &mut fbs_cache).with_max_iter(100_000);
    let status = optimizer.solve(&mut u).unwrap();

    println!("|fpr| = {}", status.norm_fpr());
    println!("solution = {:?}", u);
    assert!(status.has_converged());
    assert!(status.norm_fpr() < tolerance);
}

#[test]
fn t_solve_fbs_hard_failure_nan() {
    let bounds = constraints::NoConstraints::new();

    let problem = Problem::new(
        &bounds,
        mocks::hard_quadratic_gradient,
        mocks::hard_quadratic_cost,
    );
    let gamma = 0.005;
    let tolerance = 1e-6;

    let mut fbs_cache = FBSCache::new(NonZeroUsize::new(3).unwrap(), gamma, tolerance);
    let mut u = [-12., -160., 55.];
    let mut optimizer = FBSOptimizer::new(problem, &mut fbs_cache).with_max_iter(10000);
    let status = optimizer.solve(&mut u);
    assert_eq!(Err(SolverError::NotFiniteComputation), status);
}

#[test]
fn t_fbs_step_no_constraints() {
    let no_constraints = constraints::NoConstraints::new();
    let problem = Problem::new(&no_constraints, mocks::my_gradient, mocks::my_cost);
    let gamma = 0.1;
    let tolerance = 1e-6;

    let mut fbs_cache = FBSCache::new(NonZeroUsize::new(N_DIM).unwrap(), gamma, tolerance);
    {
        let mut fbs_engine = FBSEngine::new(problem, &mut fbs_cache);
        let mut u = [1.0, 3.0];

        assert!(fbs_engine.step(&mut u).unwrap());
        unit_test_utils::assert_nearly_equal_array(&[0.5, 2.4], &u, 1e-10, 1e-14, "u");
    }
    unit_test_utils::assert_nearly_equal_array(
        &[1., 3.],
        &fbs_cache.work_u_previous,
        1e-10,
        1e-14,
        "fbs_cache.work_u_previous",
    );
}

#[test]
fn t_fbs_step_ball_constraints() {
    let no_constraints = constraints::Ball2::new(None, 0.1);
    let problem = Problem::new(&no_constraints, mocks::my_gradient, mocks::my_cost);
    let gamma = 0.1;
    let tolerance = 1e-6;

    let mut fbs_cache = FBSCache::new(NonZeroUsize::new(N_DIM).unwrap(), gamma, tolerance);
    let mut fbs_engine = FBSEngine::new(problem, &mut fbs_cache);

    let mut u = [1.0, 3.0];

    assert!(fbs_engine.step(&mut u).unwrap());
    unit_test_utils::assert_nearly_equal_array(
        &[0.020_395_425_411_200, 0.097_898_041_973_761],
        &u,
        1e-8,
        1e-14,
        "u",
    );
}

#[test]
fn t_solve_fbs() {
    let radius = 0.2;
    let box_constraints = constraints::Ball2::new(None, radius);
    let problem = Problem::new(&box_constraints, mocks::my_gradient, mocks::my_cost);
    let gamma = 0.1;
    let tolerance = 1e-6;

    let mut fbs_cache = FBSCache::new(NonZeroUsize::new(N_DIM).unwrap(), gamma, tolerance);
    let mut u = [0.0; N_DIM];
    let mut optimizer = FBSOptimizer::new(problem, &mut fbs_cache);

    let status = optimizer.solve(&mut u).unwrap();

    assert!(status.has_converged());
    assert!(status.norm_fpr() < tolerance);

    unit_test_utils::assert_nearly_equal_array(&mocks::SOLUTION_A, &u, 1e-4, 1e-5, "u");
}

#[test]
fn t_solve_fbs_many_times() {
    // Algorithm configuration
    let gamma = 0.1;
    let tolerance = 1e-6;

    // The cache is constructed ONCE. This step allocates memory.
    let mut fbs_cache = FBSCache::new(NonZeroUsize::new(N_DIM).unwrap(), gamma, tolerance);

    let mut u = [0.0; 2];

    for _i in 1..10 {
        // Every time NMPC is executed, the constraints may change
        let box_constraints = constraints::Ball2::new(None, 0.2);

        // The problem is surely update at every execution of NMPC
        let problem = Problem::new(&box_constraints, mocks::my_gradient, mocks::my_cost);

        // Here comes the new initial condition
        u[0] = 2.0 * _i as f64;
        u[1] = -_i as f64;

        // Create a new optimizer...
        let mut optimizer = FBSOptimizer::new(problem, &mut fbs_cache);

        let status = optimizer.solve(&mut u).unwrap();

        assert!(status.norm_fpr() < tolerance);
    }
}
