//! # matrix_operations
//!
//! Matrix operations used by the optimization algorithm.
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

/// Calculate the inner product of two vectors
#[inline(always)]
pub fn inner_product(a: &[f64], b: &[f64]) -> f64 {
    assert!(a.len() == b.len());

    a.iter().zip(b.iter()).map(|(x, y)| x * y).sum()
}

/// Calculate the 2-norm of a vector
#[inline(always)]
pub fn norm1(a: &[f64]) -> f64 {
    a.iter().map(|x| x.abs()).sum()
}

/// Calculate the 2-norm of a vector
#[inline(always)]
pub fn norm2(a: &[f64]) -> f64 {
    let norm: f64 = a.iter().map(|x| x * x).sum();
    norm.sqrt()
}

/// Calculate the infinity-norm of a vector
#[inline(always)]
pub fn norm_inf(a: &[f64]) -> f64 {
    a.iter()
        .fold(0.0, |current_max, x| x.abs().max(current_max))
}

#[cfg(test)]
mod tests {
    use crate::*;

    #[test]
    fn inner_product_test() {
        assert_eq!(
            matrix_operations::inner_product(&vec![1.0, 2.0, 3.0], &vec![1.0, 2.0, 3.0]),
            14.0
        );
    }

    #[test]
    #[should_panic]
    fn inner_product_test_panic() {
        matrix_operations::inner_product(&vec![2.0, 3.0], &vec![1.0, 2.0, 3.0]);
    }

    #[test]
    fn norm1_test() {
        assert_eq!(matrix_operations::norm1(&vec![1.0, -2.0, -3.0]), 6.0);
    }

    #[test]
    fn norm2_test() {
        assert_eq!(matrix_operations::norm2(&vec![3.0, 4.0]), 5.0);
    }

    #[test]
    fn norm_inf_test() {
        assert_eq!(matrix_operations::norm_inf(&vec![1.0, -2.0, -3.0]), 3.0);
        assert_eq!(
            matrix_operations::norm_inf(&vec![1.0, -8.0, -3.0, 0.0]),
            8.0
        );
    }
}
