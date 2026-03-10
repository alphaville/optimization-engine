//!
//! Computes Cholesky factorization
//!
//!

use num::Float;

#[derive(Debug, Clone)]
/// Cholesky factoriser
pub struct CholeskyFactorizer<T> {
    n: usize,
    cholesky_factor: Vec<T>, // row-major storage of the lower-triangular factor L
}

#[derive(Debug, Clone, PartialEq)]
/// Cholesky errors
pub enum CholeskyError {
    /// Not positive definite matrix
    NotPositiveDefinite,
    /// RHS dimension mismatch
    DimensionMismatch,
}

impl<T: Float> CholeskyFactorizer<T> {
    /// Create a factorizer with storage preallocated for an n x n matrix.
    pub fn new(n: usize) -> Self {
        Self {
            n,
            cholesky_factor: vec![T::zero(); n * n],
        }
    }

    /// Compute the Cholesky factorisation A = L L^T
    /// from a square matrix in row-major order.
    ///
    /// The input matrix must have dimension equal to the one used in `new(n)`.
    pub fn factorize(&mut self, a: &[T]) -> Result<(), CholeskyError> {
        if a.len() != self.n * self.n {
            println!("[CholeskyError] n = {}, but len(a) = {}", self.n, a.len());
            return Err(CholeskyError::DimensionMismatch);
        }
        self.cholesky_factor.fill(T::zero());
        let n = self.n;
        for i in 0..n {
            for j in 0..=i {
                let mut sum = a[i * n + j];
                for k in 0..j {
                    sum = sum - self.cholesky_factor[i * n + k] * self.cholesky_factor[j * n + k];
                }
                if i == j {
                    if sum <= T::zero() {
                        return Err(CholeskyError::NotPositiveDefinite);
                    }
                    self.cholesky_factor[i * n + i] = sum.sqrt();
                } else {
                    self.cholesky_factor[i * n + j] = sum / self.cholesky_factor[j * n + j];
                }
            }
        }
        Ok(())
    }

    #[inline]
    /// dimension
    pub fn dimension(&self) -> usize {
        self.n
    }

    #[inline]
    /// Cholesky factor, L
    pub fn cholesky_factor(&self) -> &[T] {
        &self.cholesky_factor
    }

    /// Solves A x = b using the stored Cholesky factorization A = L L^T.
    pub fn solve(&self, b: &[T]) -> Result<Vec<T>, CholeskyError> {
        if b.len() != self.n {
            return Err(CholeskyError::DimensionMismatch);
        }

        let n = self.n;

        // Forward substitution: L y = b
        let mut y = vec![T::zero(); n];
        for i in 0..n {
            let mut sum = b[i];
            for j in 0..i {
                sum = sum - self.cholesky_factor[i * n + j] * y[j];
            }
            y[i] = sum / self.cholesky_factor[i * n + i];
        }

        // Back substitution: L^T x = y
        let mut x = vec![T::zero(); n];
        for i in (0..n).rev() {
            let mut sum = y[i];
            for j in (i + 1)..n {
                sum = sum - self.cholesky_factor[j * n + i] * x[j];
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
        unit_test_utils::nearly_equal_array(&expected_l, &factorizer.cholesky_factor(), 1e-10, 1e-12);
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
}
