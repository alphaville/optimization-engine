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

extern crate num;

mod contraints;
mod lbfgs;
mod lipschitz;
mod matrix_operations;
mod optimizer;
mod proximal_gradient_descent;
mod buffer;

#[derive(Debug)]
pub struct PANOC {
    lip_est: lipschitz::Estimator,
    buffers: Buffers,
}

#[derive(Debug)]
pub struct Buffers {
    current_df: Vec<f64>,
    new_location_df: Vec<f64>,
    pure_prox_location_df: Vec<f64>,
}

#[cfg(test)]
mod tests {}
