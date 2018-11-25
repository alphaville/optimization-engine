//! # panoc
//!
//! `panoc` is a library for solving nonlinear optimization problems, utilizing the PANOC solver.
//! The solver only needs the cost function and gradient to be available, and the recommended way
//! to generate these functions are via [CasADi](https://web.casadi.org/) generated functions.
//!
//!
//! # Examples
//!
//! ```
//! fn main() {
//! }
//! ```
//!
//! # Errors
//!
//!
//! # Panics
//!
//!

mod contraints;
mod lbfgs;
mod lipschitz;
mod optimizer;
mod proximal_gradient_descent;

#[derive(Debug)]
pub struct PANOC {
    lip_est: LipschitzEstimator,
    buffers: Buffers,
}

#[derive(Debug)]
pub struct Buffers {
    current_df: Vec<f64>,
    new_location_df: Vec<f64>,
    pure_prox_location_df: Vec<f64>,
}

#[cfg(test)]
mod tests {
    use crate::*;

    // #[test]
    // #[should_panic]
    // fn lbfgs_panic_problem_size() {
    //     let _ = LBFGS::new(0, 5);
    // }

    // #[test]
    // #[should_panic]
    // fn lbfgs_panic_buffer_size() {
    //     let _ = LBFGS::new(5, 0);
    // }

    // #[test]
    // fn lbfgs_test() {
    //     let l = LBFGS::new(3, 5);
    //     println!("LBFGS instance: {:#?}", l);
    // }
}
