//! # lbfgs
//!
//! The `L-BFGS` used by the optimization algorithm.
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

#[derive(Debug)]
pub struct LBFGS {
    hessian_estimate: f64,
    /// The number of vectors in s and y that are currently in use
    active_size: usize,
    /// s holds the vectors of state difference s_k = x_{k+1} - x_k
    s: Vec<Vec<f64>>,
    /// y holds the vectors of gradient difference y_k = df(x_{k+1}) - df(x_k)
    y: Vec<Vec<f64>>,
    alpha: Vec<f64>,
    rho: Vec<f64>,
    direction: Vec<f64>,
    gradient_current_location: Vec<f64>,
    gradient_new_location: Vec<f64>,
}

impl LBFGS {
    pub fn new(problem_size: usize, buffer_size: usize) -> LBFGS {
        assert!(problem_size > 0);
        assert!(buffer_size > 0);

        LBFGS {
            hessian_estimate: 0.0,
            active_size: 0,
            s: vec![vec![0.0; problem_size]; buffer_size + 1],
            y: vec![vec![0.0; problem_size]; buffer_size + 1],
            alpha: vec![0.0; buffer_size],
            rho: vec![0.0; buffer_size],
            direction: vec![0.0; problem_size],
            gradient_current_location: vec![0.0; problem_size],
            gradient_new_location: vec![0.0; problem_size],
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::*;

    #[test]
    #[should_panic]
    fn lbfgs_panic_problem_size() {
        let _ = lbfgs::LBFGS::new(0, 5);
    }

    #[test]
    #[should_panic]
    fn lbfgs_panic_buffer_size() {
        let _ = lbfgs::LBFGS::new(5, 0);
    }

    #[test]
    fn lbfgs_test() {
        let l = lbfgs::LBFGS::new(3, 5);
        println!("LBFGS instance: {:#?}", l);
    }
}
