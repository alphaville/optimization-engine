//! # PANOC Example 1
//!
//! This example shows how to minimize the [Rosenbrock function] subject to constraints.
//!
//! [Rosenbrock function]: https://en.wikipedia.org/wiki/Rosenbrock_function

use optimization_engine::{constraints::*, panoc::*, *};

fn rosenbrock_cost(a: f64, b: f64, u: &[f64]) -> f64 {
    (a - u[0]).powi(2) + b * (u[1] - u[0].powi(2)).powi(2)
}

fn rosenbrock_grad(a: f64, b: f64, u: &[f64], grad: &mut [f64]) {
    grad[0] = 2.0 * u[0] - 2.0 * a - 4.0 * b * u[0] * (-u[0].powi(2) + u[1]);
    grad[1] = b * (-2.0 * u[0].powi(2) + 2.0 * u[1]);
}

fn main() {
    /* USER PARAMETERS */
    let tolerance = 1e-14;
    let a = 1.0;
    let b = 200.0;
    let problem_size = 2;
    let lbfgs_memory_size = 10;
    let max_iters = 80;
    let mut u = [-1.5, 0.9];
    let radius = 1.0;

    // define the cost function and its gradient
    let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
        if a < 0.0 || b < 0.0 {
            Err(SolverError::Cost)
        } else {
            rosenbrock_grad(a, b, u, grad);
            Ok(())
        }
    };

    let f = |u: &[f64], c: &mut f64| -> Result<(), SolverError> {
        if a < 0.0 || b < 0.0 {
            Err(SolverError::Cost)
        } else {
            *c = rosenbrock_cost(a, b, u);
            Ok(())
        }
    };

    // define the constraints
    let bounds = Ball2::new(None, radius);

    /* PROBLEM STATEMENT */
    let problem = Problem::new(&bounds, df, f);
    let mut panoc_cache = PANOCCache::new(problem_size, tolerance, lbfgs_memory_size);
    let mut panoc = PANOCOptimizer::new(problem, &mut panoc_cache).with_max_iter(max_iters);

    // Invoke the solver
    let status = panoc.solve(&mut u);

    println!("Panoc status: {:#?}", status);
    println!("Panoc solution: {:#?}", u);
}
