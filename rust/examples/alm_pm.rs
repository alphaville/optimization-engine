//! # Augmented Lagrangian and Penalty Method
//!
//! This example shows how to solve an optimization problem based on the [Rosenbrock function].
//!
//! [Rosenbrock function]: https://en.wikipedia.org/wiki/Rosenbrock_function

use optimization_engine::{
    alm::*,
    core::{constraints::*, panoc::*},
    matrix_operations, SolverError,
};

// Smooth cost function
pub fn f(u: &[f64], cost: &mut f64) -> Result<(), SolverError> {
    *cost = 0.5 * matrix_operations::norm2_squared(u) + matrix_operations::sum(u);
    Ok(())
}

pub fn df(u: &[f64], grad: &mut [f64]) -> Result<(), SolverError> {
    grad.iter_mut()
        .zip(u.iter())
        .for_each(|(grad_i, u_i)| *grad_i = u_i + 1.0);
    Ok(())
}

pub fn f1(u: &[f64], f1u: &mut [f64]) -> Result<(), SolverError> {
    assert!(u.len() == 3, "the length of u must be equal to 3");
    assert!(f1u.len() == 2, "the length of F1(u) must be equal to 2");
    f1u[0] = 2.0 * u[0] + u[2] + 0.5;
    f1u[1] = u[0] + 3.0 * u[1];
    Ok(())
}

pub fn f1_jacobian_product(_u: &[f64], d: &[f64], res: &mut [f64]) -> Result<(), SolverError> {
    assert!(d.len() == 2, "the length of d must be equal to 3");
    assert!(res.len() == 3, "the length of res must be equal to 3");
    res[0] = 2.0 * d[0] + d[1];
    res[1] = 3.0 * d[1];
    res[2] = d[0];
    Ok(())
}

fn main() {
    let tolerance = 1e-5;
    let nx = 3;
    let n1 = 2;
    let n2 = 0;
    let lbfgs_mem = 3;
    let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
    let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);

    let set_c = Ball2::new(None, 0.5);
    let bounds = Ball2::new(None, 10.0);
    let set_y = Ball2::new(None, 1e12);

    let factory = AlmFactory::new(
        f,
        df,
        Some(f1),
        Some(f1_jacobian_product),
        NO_MAPPING,
        NO_JACOBIAN_MAPPING,
        Some(set_c),
        n2,
    );

    let alm_problem = AlmProblem::new(
        bounds,
        Some(set_c),
        Some(set_y),
        |u: &[f64], xi: &[f64], cost: &mut f64| -> Result<(), SolverError> {
            factory.psi(u, xi, cost)
        },
        |u: &[f64], xi: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
            factory.d_psi(u, xi, grad)
        },
        Some(f1),
        NO_MAPPING,
        n1,
        n2,
    );

    let mut alm_optimizer = AlmOptimizer::new(&mut alm_cache, alm_problem)
        .with_delta_tolerance(1e-5)
        .with_max_outer_iterations(20)
        .with_epsilon_tolerance(1e-6)
        .with_initial_inner_tolerance(1e-2)
        .with_inner_tolerance_update_factor(0.5)
        .with_initial_penalty(100.0)
        .with_penalty_update_factor(1.05)
        .with_sufficient_decrease_coefficient(0.2)
        .with_initial_lagrange_multipliers(&vec![5.0; n1]);

    let mut u = vec![0.0; nx];
    let solver_result = alm_optimizer.solve(&mut u);
    let r = solver_result.unwrap();
    println!("\n\nSolver result : {:#.7?}\n", r);
    println!("Solution u = {:#.6?}", u);
}
