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

pub mod constraints;
pub mod lipschitz_estimator;
pub mod matrix_operations;
pub mod optimizer;

#[derive(Debug)]
pub struct PANOC {
    buffers: Buffers,
}

#[derive(Debug)]
pub struct Buffers {
    current_df: Vec<f64>,
    new_location_df: Vec<f64>,
    pure_prox_location_df: Vec<f64>,
}

/* ---------------------------------------------------------------------------- */
/*          TESTS                                                               */
/* ---------------------------------------------------------------------------- */
#[cfg(test)]
mod tests {}
