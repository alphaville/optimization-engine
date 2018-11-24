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

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
