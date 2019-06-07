use optimization_engine::core::constraints::Rectangle;
use optimization_engine::{panoc::*, *};
use optimization_engine::continuation::ContinuationMode::*;

fn main() {
    /* cost function, f(u; q) */
    let cost_function = |u: &[f64], q: &[f64], cost: &mut f64| -> Result<(), SolverError> {
        *cost = (q[0] - u[0]).powi(2)
            + q[1] * (u[1] - u[0].powi(2)).powi(2)
            + q[2] * (1.5 * u[1] - u[0]).powi(2);
        Ok(())
    };

    /* parametric gradient, df(u, q) */
    let gradient_function = |u: &[f64], q: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
        let t = 1.5 * u[0] - u[1];
        let s = u[1] - u[0].powi(2);
        grad[0] = 2.0 * (u[0] - q[0]) - 4.0 * q[1] * s + 3.0 * q[2] * t;
        grad[1] = 2.0 * q[1] * s - 2.0 * q[2] * t;
        Ok(())
    };

    /* penalty-type constraints: c(u; p) */
    let penalty_constr_function =
        |u: &[f64], _q: &[f64], constr: &mut [f64]| -> Result<(), SolverError> {
            constr[0] = 1.5 * u[0] - u[1];
            Ok(())
        };

    // Constraints...
    let xmin = &[-1.0, -1.0];
    let xmax = &[1.0, 1.0];
    let bounds = Rectangle::new(Some(xmin), Some(xmax));

    let num_constraints = 1;
    let homotopy_problem = continuation::HomotopyProblem::new(
        bounds,
        gradient_function,
        cost_function,
        penalty_constr_function,
        num_constraints,
    );

    let panoc_cache = PANOCCache::new(2, 1e-5, 5);
    let mut homotopy_cache = continuation::HomotopyCache::new(panoc_cache);

    homotopy_cache.add_continuation(
        2,
        100.,
        std::f64::INFINITY,
        Geometric(50.0),
    );

    let mut homotopy_optimizer =
        continuation::HomotopyOptimizer::new(homotopy_problem, &mut homotopy_cache)
            .with_constraint_tolerance(1e-5);
    let mut u: [f64; 2] = [0.2, 1.5];
    let p: [f64; 3] = [1., 100., 1.];
    let status = homotopy_optimizer.solve(&mut u, &p);

    println!("{:#?}", status);
    println!("{:#?}", u);
}
