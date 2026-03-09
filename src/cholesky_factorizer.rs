//!
//! Computes Cholesky factorization
//!
//!

use num::Float;


#[derive(Debug, Clone)]
/// Cholesky factoriser
pub struct CholeskyFactoriser<T> {
    n: usize,
    l: Vec<T>, // row-major storage of the lower-triangular factor L
}

#[derive(Debug, Clone, PartialEq)]
/// Cholesky errors
pub enum CholeskyError {
    /// Not square matrix
    NotSquare,
    /// Not positive definite matrix
    NotPositiveDefinite,
    /// RHS dimension mismatch
    DimensionMismatch,
}


impl<T: Float> CholeskyFactoriser<T> {
    /// Create a factoriser with storage preallocated for an n x n matrix.
    pub fn new(n: usize) -> Self {
        Self { n, l: vec![T::zero(); n * n] }
    }

    /// Compute the Cholesky factorisation A = L L^T
    /// from a square matrix in row-major order.
    ///
    /// The input matrix must have dimension equal to the one used in `new(n)`.
    pub fn factorize(&mut self, a: &[T]) -> Result<(), CholeskyError> {
        if a.len() != self.n * self.n {
            println!("n = {}, but len(a) = {}", self.n, a.len());
            return Err(CholeskyError::DimensionMismatch);
        }
        self.l.fill(T::zero());
        let n = self.n;
        for i in 0..n {
            for j in 0..=i {
                let mut sum = a[i * n + j];
                for k in 0..j {
                    sum = sum - self.l[i * n + k] * self.l[j * n + k];
                }
                if i == j {
                    if !(sum > T::zero()) {
                        return Err(CholeskyError::NotPositiveDefinite);
                    }
                    self.l[i * n + i] = sum.sqrt();
                } else {
                    self.l[i * n + j] = sum / self.l[j * n + j];
                }
            }
        }
        Ok(())
    }

    #[inline]

    /// dim
    pub fn dimension(&self) -> usize {
        self.n
    }

    #[inline]
    /// l
    pub fn l(&self) -> &[T] {
        &self.l
    }

    /// Solves A x = b using the stored Cholesky factorisation A = L L^T.
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
                sum = sum - self.l[i * n + j] * y[j];
            }
            y[i] = sum / self.l[i * n + i];
        }

        // Back substitution: L^T x = y
        let mut x = vec![T::zero(); n];
        for i in (0..n).rev() {
            let mut sum = y[i];
            for j in (i + 1)..n {
                sum = sum - self.l[j * n + i] * x[j];
            }
            x[i] = sum / self.l[i * n + i];
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
        let a = vec![
             4.0_f64,  12.0, -16.0,
            12.0,      37.0, -43.0,
           -16.0,     -43.0,  98.0,
        ];
        let mut factoriser = CholeskyFactoriser::new(3);
        factoriser.factorize(&a).unwrap();
        println!("n = {}", factoriser.dimension());
        println!("L = {:?}", factoriser.l());

        let b = vec![1.0_f64, 2.0, 1.0];
        let x = factoriser.solve(&b).unwrap();
        println!("x = {:?}", x);
    }

}
