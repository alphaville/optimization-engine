//! # PANOC Example 2
//!
//! This example shows how to minimize the [Rosenbrock function] subject to constraints,
//! in a loop using a common cache object.
//!
//! [Rosenbrock function]: https://en.wikipedia.org/wiki/Rosenbrock_function
//!
use optimization_engine::{panoc::*, *};

fn rosenbrock_cost(a: f64, b: f64, u: &[f64]) -> f64 {
    (a - u[0]).powi(2) + b * (u[1] - u[0].powi(2)).powi(2)
}

fn rosenbrock_grad(a: f64, b: f64, u: &[f64], grad: &mut [f64]) {
    grad[0] = 2.0 * u[0] - 2.0 * a - 4.0 * b * u[0] * (-u[0].powi(2) + u[1]);
    grad[1] = b * (-2.0 * u[0].powi(2) + 2.0 * u[1]);
}

fn main() {
    let tolerance = 1e-6;
    let mut a_param = 1.0;
    let mut b_param = 100.0;
    let n_dim_u = 2;
    let lbfgs_memory = 10;
    let max_iters = 100;
    let mut u = [-1.5, 0.9];
    let mut radius = 1.0;

    // the cache is created only ONCE
    let mut panoc_cache = PANOCCache::new(n_dim_u, tolerance, lbfgs_memory);

    let mut idx = 0;
    while idx < 100 {
        // update the values of `a`, `b` and `radius`
        b_param *= 1.01;
        a_param -= 1e-3;
        radius += 0.001;

        // update the function definitions (`f` and `df`)
        let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
            rosenbrock_grad(a_param, b_param, u, grad);
            Ok(())
        };
        let f = |u: &[f64], c: &mut f64| -> Result<(), SolverError> {
            *c = rosenbrock_cost(a_param, b_param, u);
            Ok(())
        };

        // define the bounds at every iteration
        let bounds = constraints::Ball2::new(None, radius);

        // the problem definition is updated at every iteration
        let problem = Problem::new(&bounds, df, f);

        // updated instance of the solver
        let mut panoc = PANOCOptimizer::new(problem, &mut panoc_cache).with_max_iter(max_iters);

        let status = panoc.solve(&mut u).unwrap();

        idx += 1;

        // print useful information
        println!(
            "parameters: (a={:.4}, b={:.4}, r={:.4}), iters = {}",
            a_param,
            b_param,
            radius,
            status.iterations()
        );
        println!("u = {:#.6?}", u);
    }
}
