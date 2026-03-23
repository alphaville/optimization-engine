//! Cholesky factorization and linear solves for symmetric positive-definite matrices.
//!
//! This module provides a [`CholeskyFactorizer`] type for computing and storing the
//! Cholesky factorization of a square matrix
//!
//! $$A = L L^\intercal$$
//!
//! where $L$ is lower triangular.
//!
//! The matrix is provided as a flat slice in **row-major** order. Internally, the
//! computed Cholesky factor `L` is also stored in row-major order in a dense
//! `Vec<T>`. Only the lower-triangular part is meaningful after factorization.
//!
//! Once a factorization has been computed with [`CholeskyFactorizer::factorize`],
//! the stored factor can be reused to solve linear systems of the form
//!
//! $$A x = b$$
//!
//! via [`CholeskyFactorizer::solve`].
//!
//! # Requirements
//!
//! The input matrix must:
//!
//! - be square of dimension `n × n`
//! - be stored in row-major order
//! - be symmetric positive definite
//!
//! If the matrix is not positive definite, factorization fails with
//! [`CholeskyError::NotPositiveDefinite`].
//!
//! # Errors
//!
//! The module uses [`CholeskyError`] to report the following conditions:
//!
//! - [`CholeskyError::DimensionMismatch`] when the input matrix or right-hand side
//!   has an incompatible size
//! - [`CholeskyError::NotPositiveDefinite`] when factorization fails because the
//!   matrix is not positive definite
//! - [`CholeskyError::NotFactorized`] when attempting to solve a system before a
//!   valid factorization has been computed
//!
//! # Example
//!
//! ```rust
//! use optimization_engine::CholeskyFactorizer;
//!
//! let a = vec![
//!     4.0_f64, 1.0,
//!     1.0,     3.0,
//! ];
//!
//! let b = vec![1.0_f64, 2.0];
//!
//! let mut factorizer = CholeskyFactorizer::new(2);
//! factorizer.factorize(&a).unwrap();
//!
//! let x = factorizer.solve(&b).unwrap();
//!
//! assert!((x[0] - 0.0909090909).abs() < 1e-10);
//! assert!((x[1] - 0.6363636364).abs() < 1e-10);
//! ```
//!
//! # Notes
//!
//! - The implementation is generic over `T: Float`.
//! - Storage is preallocated when constructing the factorizer with
//!   [`CholeskyFactorizer::new`].
//! - The factorizer keeps the computed factor internally so that multiple right-hand
//!   sides can be solved efficiently after a single factorization.

use num::Float;

#[derive(Debug, Clone)]
/// Cholesky factoriser
pub struct CholeskyFactorizer<T> {
    n: usize,
    cholesky_factor: Vec<T>, // row-major storage of the lower-triangular factor L
    is_factorized: bool,     // whether factorization has been completed
}

#[derive(Debug, Clone, PartialEq, Eq)]
/// Cholesky errors
pub enum CholeskyError {
    /// Not positive definite matrix
    NotPositiveDefinite,
    /// RHS dimension mismatch
    DimensionMismatch,
    /// Attempting to solve without having factorized
    NotFactorized,
}

impl<T: Float> CholeskyFactorizer<T> {
    /// Create a factorizer with storage preallocated for an n x n matrix.
    pub fn new(n: usize) -> Self {
        Self {
            n,
            cholesky_factor: vec![T::zero(); n * n],
            is_factorized: false,
        }
    }

    /// Compute the Cholesky factorization $A = L L^\intercal$
    /// from a square matrix in row-major order.
    ///
    /// The input matrix must have dimension equal to the one used in `new(n)`.
    pub fn factorize(&mut self, a: &[T]) -> Result<(), CholeskyError> {
        self.is_factorized = false;
        if a.len() != self.n * self.n {
            return Err(CholeskyError::DimensionMismatch);
        }
        self.cholesky_factor.fill(T::zero());
        let n = self.n;

        for i in 0..n {
            let row_i = i * n;

            for j in 0..=i {
                let row_j = j * n;
                let mut sum = a[row_i + j];

                for k in 0..j {
                    sum = sum - self.cholesky_factor[row_i + k] * self.cholesky_factor[row_j + k];
                }

                if i == j {
                    if sum <= T::zero() {
                        return Err(CholeskyError::NotPositiveDefinite);
                    }
                    self.cholesky_factor[row_i + i] = sum.sqrt();
                } else {
                    self.cholesky_factor[row_i + j] = sum / self.cholesky_factor[row_j + j];
                }
            }
        }
        self.is_factorized = true;
        Ok(())
    }

    #[inline]
    #[must_use]
    /// dimension
    pub fn dimension(&self) -> usize {
        self.n
    }

    #[inline]
    #[must_use]
    /// Cholesky factor, L
    pub fn cholesky_factor(&self) -> &[T] {
        &self.cholesky_factor
    }

    /// Solves the linear system $A x = b$ using the stored Cholesky factorization
    /// $A = L L^\intercal$.
    ///
    /// This method assumes that [`Self::factorize`] has already been called successfully,
    /// so that the lower-triangular Cholesky factor `L` is available internally.
    ///
    /// The solution is computed in two steps:
    ///
    /// 1. Forward substitution to solve $L y = b$
    /// 2. Back substitution to solve $L^\intercal x = y$
    ///
    /// # Arguments
    ///
    /// * `b` - Right-hand-side vector of length `n`
    ///
    /// # Returns
    ///
    /// Returns the solution vector `x` such that $A x = b$.
    ///
    /// # Errors
    ///
    /// Returns an error in the following cases:
    ///
    /// * `CholeskyError::NotFactorized` if no valid Cholesky factorization is currently
    ///   stored in the factorizer
    /// * `CholeskyError::DimensionMismatch` if `b.len() != n`, where `n` is the matrix dimension
    ///
    /// # Notes
    ///
    /// * The matrix `A` itself is not accessed directly; only its Cholesky factor `L` is used.
    /// * The Cholesky factor is stored internally in row-major order.
    /// * This method allocates temporary vectors for the intermediate solution `y` and the
    ///   final solution `x`.
    pub fn solve(&self, b: &[T]) -> Result<Vec<T>, CholeskyError> {
        if !self.is_factorized {
            return Err(CholeskyError::NotFactorized);
        }
        if b.len() != self.n {
            return Err(CholeskyError::DimensionMismatch);
        }

        let n = self.n;

        // Forward substitution: L y = b
        let mut y = vec![T::zero(); n];
        for i in 0..n {
            let row_i = i * n;
            let mut sum = b[i];
            for (&lij, &yj) in self.cholesky_factor[row_i..row_i + i]
                .iter()
                .zip(y[..i].iter())
            {
                sum = sum - lij * yj;
            }
            y[i] = sum / self.cholesky_factor[row_i + i];
        }

        // Back substitution: L^T x = y
        let mut x = vec![T::zero(); n];
        for i in (0..n).rev() {
            let mut sum = y[i];
            for (row_j, &xj) in self
                .cholesky_factor
                .chunks_exact(n)
                .skip(i + 1)
                .zip(x.iter().skip(i + 1))
            {
                sum = sum - row_j[i] * xj;
            }
            x[i] = sum / self.cholesky_factor[i * n + i];
        }

        Ok(x)
    }
}

/* ---------------------------------------------------------------------------- */
/*          TESTS                                                               */
/* ---------------------------------------------------------------------------- */
#[cfg(test)]
mod tests {
    use crate::*;

    #[test]
    fn t_cholesky_basic() {
        let a = vec![4.0_f64, 12.0, -16.0, 12.0, 37.0, -43.0, -16.0, -43.0, 98.0];
        let mut factorizer = CholeskyFactorizer::new(3);
        let _ = factorizer.factorize(&a);
        assert!(3 == factorizer.dimension(), "wrong dimension");
        let expected_l = [2.0, 0.0, 0.0, 6.0, 1.0, 0.0, -8.0, 5.0, 3.0];
        unit_test_utils::nearly_equal_array(&expected_l, factorizer.cholesky_factor(), 1e-10, 1e-12);
    }

    #[test]
    fn t_cholesky_solve_linear_system() {
        let a = vec![4.0_f64, 12.0, -16.0, 12.0, 37.0, -43.0, -16.0, -43.0, 98.0];
        let mut factorizer = CholeskyFactorizer::new(3);
        let _ = factorizer.factorize(&a);
        let rhs = vec![-5.0_f64, 2.0, -3.0];
        let x = factorizer.solve(&rhs).unwrap();
        let expected_sol = [-280.25_f64, 77., -12.];
        unit_test_utils::nearly_equal_array(&expected_sol, &x, 1e-10, 1e-12);
    }

    #[test]
    fn t_cholesky_f32() {
        let a = vec![4.0_f32, 12.0, -16.0, 12.0, 37.0, -43.0, -16.0, -43.0, 98.0];
        let mut factorizer = CholeskyFactorizer::new(3);
        factorizer.factorize(&a).unwrap();

        let expected_l = [2.0_f32, 0.0, 0.0, 6.0, 1.0, 0.0, -8.0, 5.0, 3.0];
        unit_test_utils::nearly_equal_array(&expected_l, factorizer.cholesky_factor(), 1e-5, 1e-6);

        let rhs = vec![-5.0_f32, 2.0, -3.0];
        let x = factorizer.solve(&rhs).unwrap();
        let expected_sol = [-280.25_f32, 77.0, -12.0];
        unit_test_utils::nearly_equal_array(&expected_sol, &x, 1e-4, 1e-5);
    }

    #[test]
    fn t_cholesky_not_square_matrix() {
        let a = vec![1.0_f64, 2., 7., 5., 9.];
        let mut factorizer = CholeskyFactorizer::new(3);
        let result = factorizer.factorize(&a);
        assert_eq!(result, Err(CholeskyError::DimensionMismatch));
    }

    #[test]
    fn t_cholesky_solve_wrong_dimension_rhs() {
        let a = vec![4.0_f64, 12.0, -16.0, 12.0, 37.0, -43.0, -16.0, -43.0, 98.0];
        let mut factorizer = CholeskyFactorizer::new(3);
        let _ = factorizer.factorize(&a);
        let rhs = vec![-5.0_f64, 2.0];
        let result = factorizer.solve(&rhs);
        assert_eq!(result, Err(CholeskyError::DimensionMismatch));
    }

    #[test]
    fn t_cholesky_solve_not_factorized_1() {
        let factorizer = CholeskyFactorizer::new(3);
        let rhs = vec![-5.0_f64, 2.0];
        let result = factorizer.solve(&rhs);
        assert_eq!(result, Err(CholeskyError::NotFactorized));
    }

    #[test]
    fn t_cholesky_solve_not_factorized_2() {
        let a = vec![1.0_f64, 1.0, 1.0, 1.0];
        let mut factorizer = CholeskyFactorizer::new(2);
        let factorization_result = factorizer.factorize(&a);
        assert_eq!(
            factorization_result,
            Err(CholeskyError::NotPositiveDefinite)
        );
        let rhs = vec![-5.0_f64, 2.0];
        let result = factorizer.solve(&rhs);
        assert_eq!(result, Err(CholeskyError::NotFactorized));
    }
}
