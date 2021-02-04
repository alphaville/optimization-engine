/// Example for testing performance in problems PANOC was not made for.

use optimization_engine::{constraints::*, panoc::*, *};

fn main() {
    let mut panoc_cache = PANOCCache::new(1, 1e-5, 10);
    let u = &mut [1e4];

    // Define the cost function and its gradient.
    let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
        println!("df, u = {}", u[0]);
        grad[0] = u[0].signum();
        // grad[0] = u[0] / (1. + sq(u[0])).sqrt(); // Soft version

        Ok(())
    };

    let f = |u: &[f64], c: &mut f64| -> Result<(), SolverError> {
        println!("f, u = {}", u[0]);
        *c = u[0].abs();
        // *c = (1. + sq(u[0])).sqrt(); // Soft version
        Ok(())
    };

    let bounds = NoConstraints::new();

    // Problem statement.
    let problem = Problem::new(&bounds, df, f);

    let mut panoc = PANOCOptimizer::new(problem, &mut panoc_cache).with_max_iter(20);

    // Invoke the solver.
    let status = panoc.solve(u);

    println!("Solver status: {:#?}", status);
    println!("u: {:?}", u);
}
