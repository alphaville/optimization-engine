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

extern crate lipschitz_estimator;
extern crate num;

pub mod constraints;
pub mod matrix_operations;
mod optimizer;
pub mod proximal_gradient_descent;

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

#[cfg(test)]
mod tests {

    fn modify_u(u: &mut [f64]) {
        u.iter_mut().for_each(|x| *x += 1.0);
    }

    #[test]
    fn testme() {
        let mut u = vec![0.0_f64; 10];
        modify_u(&mut u);
        modify_u(&mut u);
    }
}
