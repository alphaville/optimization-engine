//! # Constrained minimisation with penalty method
//!
//! In this example we solve the problem:
//!
//! $$
//! \begin{aligned}
//! \text{Minimise  }  f(u) = \tfrac{1}{2}\\|u\\|^2 + \sum_{i=1}^{n}u_i
//! \\\\
//! F_2(u) = \\|u\\|^2 - 1 = 0
//! \end{aligned}
//! $$

use optimization_engine::{
    alm::*,
    core::{constraints::*, panoc::*},
    matrix_operations, SolverError,
};

/// Smooth cost function
///
/// This is a quadratic function given by
/// $$
/// \begin{aligned}
/// f(u) = \tfrac{1}{2}\\|u\\|^2 + \sum_{i=1}^{n}u_i
/// \end{aligned}
/// $$
pub fn f(u: &[f64], cost: &mut f64) -> Result<(), SolverError> {
    *cost = 0.5 * matrix_operations::norm2_squared(u) + matrix_operations::sum(u);
    Ok(())
}

/// Gradient of the cost function
///
/// $$\begin{aligned}
/// f(u) = \tfrac{1}{2}\\|u\\|^2 + \sum_{i=1}^{n}u_i
/// \Rightarrow
/// \nabla f(u) = u + 1_n
/// \end{aligned}
/// $$
pub fn df(u: &[f64], grad: &mut [f64]) -> Result<(), SolverError> {
    grad.iter_mut()
        .zip(u.iter())
        .for_each(|(grad_i, u_i)| *grad_i = u_i + 1.0);
    Ok(())
}

/// Function $F_2 = \\|u\\|^2 - 1$
///
///
pub fn f2(u: &[f64], res: &mut [f64]) -> Result<(), SolverError> {
    res[0] = matrix_operations::norm2_squared(u) - 1.;
    Ok(())
}

/// Function $JF_2(u)^\top d$
pub fn jf2t(u: &[f64], d: &[f64], res: &mut [f64]) -> Result<(), crate::SolverError> {
    res.iter_mut()
        .zip(u.iter())
        .for_each(|(res_i, u_i)| *res_i = u_i * d[0]);
    Ok(())
}

fn main() {
    let tolerance = 1e-4;
    let nx = 3;
    let n1 = 0;
    let n2 = 1;
    let lbfgs_mem = 5;
    let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
    let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);

    let bounds = NoConstraints::new();

    let factory = AlmFactory::new(
        f,
        df,
        NO_MAPPING,
        NO_JACOBIAN_MAPPING,
        Some(f2),
        Some(jf2t),
        NO_SET,
        n2,
    );

    let alm_problem = AlmProblem::new(
        bounds,
        NO_SET,
        NO_SET,
        |u: &[f64], xi: &[f64], cost: &mut f64| -> Result<(), SolverError> {
            factory.psi(u, xi, cost)
        },
        |u: &[f64], xi: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
            factory.d_psi(u, xi, grad)
        },
        NO_MAPPING,
        Some(f2),
        n1,
        n2,
    );

    let mut alm_optimizer = AlmOptimizer::new(&mut alm_cache, alm_problem)
        .with_delta_tolerance(1e-6)
        .with_epsilon_tolerance(1e-5)
        .with_max_outer_iterations(20)
        .with_max_inner_iterations(1000)
        .with_initial_penalty(5000.0)
        .with_penalty_update_factor(2.2);

    let mut u = vec![0.1; nx];
    let solver_result = alm_optimizer.solve(&mut u);
    let r = solver_result.unwrap();
    println!("\n\nSolver result : {:#.7?}\n", r);
    println!("Solution u = {:#.6?}", u);
}
