//! # PANOC Example 1
//!
//! This example shows how to solve an optimization problem based on the [Rosenbrock function].
//!
//! [Rosenbrock function]: https://en.wikipedia.org/wiki/Rosenbrock_function

use optimization_engine::{constraints::*, panoc::*, *};
use std::num::NonZeroUsize;

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
    let problem_size = NonZeroUsize::new(2).unwrap();
    let lbfgs_memory_size = NonZeroUsize::new(10).unwrap();
    let max_iters = 80;
    let mut u = [-1.5, 0.9];
    let radius = 1.0;

    // define the cost function and its gradient
    let df = |u: &[f64], grad: &mut [f64]| -> Result<(), Error> {
        rosenbrock_grad(a, b, u, grad);
        Ok(())
    };

    let f = |u: &[f64], c: &mut f64| -> Result<(), Error> {
        *c = rosenbrock_cost(a, b, u);
        Ok(())
    };

    // define the constraints
    let bounds = Ball2::new_at_origin_with_radius(radius);

    /* PROBLEM STATEMENT */
    let problem = Problem::new(bounds, df, f);
    let mut panoc_cache = PANOCCache::new(problem_size, tolerance, lbfgs_memory_size);
    let mut panoc = PANOCOptimizer::new(problem, &mut panoc_cache).with_max_iter(max_iters);

    // Invoke the solver
    let status = panoc.solve(&mut u);

    println!("Panoc status: {:#?}", status);
    println!("Panoc solution: {:#?}", u);
}
