//! # matrix_operations
//!
//! Simple algebraic operations used by Optimization Engine
//!
//! # Examples
//!
//! ```
//! use optimization_engine::matrix_operations::*;
//!
//! let a = [1.0, 2.0, 3.0];
//! let b = [4.0, 5.0, 6.0];
//!
//! // inner product
//! let a_dot_b = inner_product(&a, &b);
//! assert!(a_dot_b == 32.);
//!
//! // Euclidean norm and squared norm
//! let norm_a = norm2(&a);
//! let norm_sq_a = norm2_squared(&a);
//! assert!(norm_sq_a == 14.);
//!
//! // Squared Euclidean norm of difference of vectors
//! let norm_sq_a_minus_b = norm2_squared_diff(&a, &b);
//! assert!(norm_sq_a_minus_b == 27.);
//!
//! // Sum of elements of vector
//! let sum_a = sum(&a);
//! assert!(sum_a == 6.);
//!
//! // Check whether all elements of a vector are finite
//! assert!(is_finite(&a));
//!
//! // Infinity norm
//! let norm_inf_b = norm_inf(&b);
//! assert!(norm_inf_b == 6.);
//! ```
//!

use num::{Float, Zero};
use std::iter::Sum;
use std::ops::Mul;

/// Calculate the inner product of two vectors
#[inline(always)]
pub fn inner_product<T>(a: &[T], b: &[T]) -> T
where
    T: Float + Sum<T> + Mul<T, Output = T>,
{
    assert!(a.len() == b.len());

    a.iter().zip(b.iter()).map(|(x, y)| (*x) * (*y)).sum()
}

/// Calculate the 1-norm of a vector
#[inline(always)]
pub fn norm1<T>(a: &[T]) -> T
where
    T: Float + Sum<T>,
{
    a.iter().map(|x| x.abs()).sum()
}

/// Calculate the 2-norm of a vector
#[inline(always)]
pub fn norm2<T>(a: &[T]) -> T
where
    T: Float + Sum<T> + Mul<T, Output = T>,
{
    let norm: T = norm2_squared(a);
    norm.sqrt()
}

/// Calculate the squared 2-norm of the difference of two vectors
#[inline(always)]
pub fn norm2_squared_diff<T>(a: &[T], b: &[T]) -> T
where
    T: Float + Sum<T> + Mul<T, Output = T> + std::ops::AddAssign,
{
    a.iter().zip(b.iter()).fold(T::zero(), |mut sum, (&x, &y)| {
        sum += (x - y).powi(2);
        sum
    })
}

/// Calculate the 2-norm of a vector
#[inline(always)]
pub fn norm2_squared<T>(a: &[T]) -> T
where
    T: Float + Sum<T> + Mul<T, Output = T>,
{
    let norm: T = a.iter().map(|x| (*x) * (*x)).sum();
    norm
}

/// Calculate the sum of all elements of a vector
#[inline(always)]
pub fn sum<T>(a: &[T]) -> T
where
    T: Float + Sum<T> + Mul<T, Output = T>,
{
    let norm: T = a.iter().copied().sum();
    norm
}

/// Calculates the infinity-norm of a vector
#[inline(always)]
pub fn norm_inf<T>(a: &[T]) -> T
where
    T: Float + Zero,
{
    a.iter()
        .fold(T::zero(), |current_max, x| x.abs().max(current_max))
}

/// Computes the infinity norm of the difference of two vectors
#[inline(always)]
pub fn norm_inf_diff<T>(a: &[T], b: &[T]) -> T
where
    T: Float + Zero,
{
    assert_eq!(a.len(), b.len());
    a.iter()
        .zip(b.iter())
        .fold(T::zero(), |current_max, (x, y)| {
            (*x - *y).abs().max(current_max)
        })
}

/// Checks whether all elements of a vector are finite
///
/// ## Returns
///
/// Returns `true` if all elements are finite and `false` if any
/// of the elements are either NaN or Infinity
#[inline(always)]
pub fn is_finite<T>(a: &[T]) -> bool
where
    T: Float,
{
    !a.iter().any(|&xi| !xi.is_finite())
}

/* ---------------------------------------------------------------------------- */
/*          TESTS                                                               */
/* ---------------------------------------------------------------------------- */
#[cfg(test)]
mod tests {
    use crate::*;

    #[test]
    fn t_inner_product_test() {
        unit_test_utils::assert_nearly_equal(
            14.0,
            matrix_operations::inner_product(&[1.0, 2.0, 3.0], &[1.0, 2.0, 3.0]),
            1e-10,
            1e-16,
            "inner product",
        );
    }

    #[test]
    #[should_panic]
    fn t_inner_product_test_panic() {
        matrix_operations::inner_product(&[2.0, 3.0], &[1.0, 2.0, 3.0]);
    }

    #[test]
    fn t_norm1_test() {
        unit_test_utils::assert_nearly_equal(
            6.0,
            matrix_operations::norm1(&[1.0, -2.0, -3.0]),
            1e-10,
            1e-16,
            "norm1",
        );
    }

    #[test]
    fn t_norm2_test() {
        unit_test_utils::assert_nearly_equal(
            5.0,
            matrix_operations::norm2(&[3.0, 4.0]),
            1e-10,
            1e-16,
            "norm2",
        );
    }

    #[test]
    fn t_norm_inf_test() {
        unit_test_utils::assert_nearly_equal(
            3.0,
            matrix_operations::norm_inf(&[1.0, -2.0, -3.0]),
            1e-10,
            1e-16,
            "norm infinity of vector",
        );
        unit_test_utils::assert_nearly_equal(
            8.0,
            matrix_operations::norm_inf(&[1.0, -8.0, -3.0, 0.0]),
            1e-10,
            1e-16,
            "infinity norm",
        );
    }

    #[test]
    fn t_norm_inf_diff() {
        let x = [1.0, 2.0, 1.0];
        let y = [-4.0, 0.0, 3.0];
        let norm_diff = matrix_operations::norm_inf_diff(&x, &y);
        unit_test_utils::assert_nearly_equal(5.0f64, norm_diff, 1e-10, 1e-9, "norm of difference");
        unit_test_utils::assert_nearly_equal(
            0.0f64,
            matrix_operations::norm_inf_diff(&x, &x),
            1e-10,
            1e-16,
            "difference of same vector",
        );
        unit_test_utils::assert_nearly_equal(
            0.0f64,
            matrix_operations::norm_inf_diff(&[], &[]),
            1e-10,
            1e-16,
            "difference of empty vectors",
        );
    }

    #[test]
    #[should_panic]
    fn t_norm_inf_diff_panic() {
        let x = [1.0, 2.0, 3.0];
        let y = [0.0, 3.0];
        let _ = matrix_operations::norm_inf_diff(&x, &y);
    }

    #[test]
    fn t_norm2_squared_diff_test() {
        let x = [2.0, 5.0, 7.0, -1.0];
        let y = [4.0, 1.0, 0.0, 10.0];
        let norm2sq = matrix_operations::norm2_squared_diff(&x, &y);
        unit_test_utils::assert_nearly_equal(190., norm2sq, 1e-10, 1e-12, "norm sq diff");
    }
}
