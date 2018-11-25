//! # lipschitz
//!
//! The `lipschitz` estimator used by the optimization algorithm.
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
pub struct Estimator {
    current_position_delta: Vec<f64>,
    df_current_position_delta: Vec<f64>,
}

impl Estimator {
    pub fn new(problem_size: usize) -> Estimator {
        assert!(problem_size > 0);

        Estimator {
            current_position_delta: vec![0.0; problem_size],
            df_current_position_delta: vec![0.0; problem_size],
        }
    }

    pub fn estimate() -> f64 {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use crate::*;

    // #[test]
    // fn testing() {
    // }
}
