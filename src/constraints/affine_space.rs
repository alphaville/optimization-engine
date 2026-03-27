use super::Constraint;
use crate::matrix_operations;
use crate::{CholeskyError, CholeskyFactorizer};

use ndarray::{ArrayView1, ArrayView2, LinalgScalar};
use num::Float;

#[derive(Clone)]
/// An affine space here is defined as the set of solutions of a linear equation, $Ax = b$,
/// that is, $E=\\{x\in\mathbb{R}^n: Ax = b\\}$, which is an affine space. It is assumed that
/// the matrix $AA^\intercal$ is full-rank.
pub struct AffineSpace<T = f64> {
    a_mat: Vec<T>,
    b_vec: Vec<T>,
    factorizer: CholeskyFactorizer<T>,
    n_rows: usize,
    n_cols: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Errors that can arise when constructing an [`AffineSpace`].
pub enum AffineSpaceError {
    /// The vector `b` is empty.
    EmptyB,
    /// The dimensions of `A` and `b` are incompatible.
    IncompatibleDimensions,
    /// The matrix `AA^T` is not positive definite, which typically means
    /// that `A` does not have full row rank.
    NotFullRowRank,
}

impl<T> AffineSpace<T>
where
    T: Float + LinalgScalar + 'static,
{
    /// Construct a new affine space given the matrix $A\in\mathbb{R}^{m\times n}$ and
    /// the vector $b\in\mathbb{R}^m$
    ///
    /// ## Arguments
    ///
    /// - `a`: matrix $A$, row-wise data
    /// - `b`: vector $b$
    ///
    /// ## Returns
    /// New Affine Space structure
    ///
    /// ## Panics
    ///
    /// Panics if:
    ///
    /// - `b` is empty,
    /// - `A` and `b` have incompatible dimensions,
    /// - `A` does not have full row rank.
    ///
    /// Use [`AffineSpace::try_new`] if you want to handle these conditions
    /// without panicking.
    ///
    pub fn new(a: Vec<T>, b: Vec<T>) -> Self {
        Self::try_new(a, b).expect("invalid affine space data")
    }

    /// Construct a new affine space given the matrix $A\in\mathbb{R}^{m\times n}$
    /// and the vector $b\in\mathbb{R}^m$.
    ///
    /// ## Arguments
    ///
    /// - `a`: matrix $A$, row-wise data
    /// - `b`: vector $b$
    ///
    /// ## Returns
    ///
    /// Returns a new [`AffineSpace`] on success, or an [`AffineSpaceError`] if
    /// the provided data are invalid.
    pub fn try_new(a: Vec<T>, b: Vec<T>) -> Result<Self, AffineSpaceError> {
        let n_rows = b.len();
        let n_elements_a = a.len();
        if n_rows == 0 {
            return Err(AffineSpaceError::EmptyB);
        }
        if !n_elements_a.is_multiple_of(n_rows) {
            return Err(AffineSpaceError::IncompatibleDimensions);
        }
        let n_cols = n_elements_a / n_rows;
        let aat = matrix_operations::mul_a_at(&a, n_rows, n_cols)
            .map_err(|_| AffineSpaceError::IncompatibleDimensions)?;
        let mut factorizer = CholeskyFactorizer::new(n_rows);
        factorizer.factorize(&aat).map_err(|err| match err {
            CholeskyError::NotPositiveDefinite => AffineSpaceError::NotFullRowRank,
            CholeskyError::DimensionMismatch | CholeskyError::NotFactorized => {
                AffineSpaceError::IncompatibleDimensions
            }
        })?;
        Ok(AffineSpace {
            a_mat: a,
            b_vec: b,
            factorizer,
            n_rows,
            n_cols,
        })
    }
}

impl<T> Constraint<T> for AffineSpace<T>
where
    T: Float + LinalgScalar + 'static,
{
    /// Projection onto the set $E = \\{x: Ax = b\\}$, which is computed by
    /// $$P_E(x) = x - A^\intercal z(x),$$
    /// where $z$ is the solution of the linear system
    /// $$(AA^\intercal)z = Ax - b,$$
    /// which has a unique solution provided $A$ has full row rank. The linear system
    /// is solved by computing the Cholesky factorization of $AA^\intercal$, which is
    /// done using `CholeskyFactorizer`
    ///
    /// ## Arguments
    ///
    /// - `x`: The given vector $x$ is updated with the projection on the set
    ///
    /// ## Example
    ///
    /// Consider the set $X = \\{x \in \mathbb{R}^4 :Ax = b\\}$, with $A\in\mathbb{R}^{3\times 4}$
    /// being the matrix
    /// $$A = \begin{bmatrix}0.5 & 0.1& 0.2& -0.3\\\\ -0.6& 0.3& 0 & 0.5 \\\\ 1.0& 0.1& -1& -0.4\end{bmatrix},$$
    /// and $b$ being the vector
    /// $$b = \begin{bmatrix}1 \\\\ 2 \\\\ -0.5\end{bmatrix}.$$
    ///
    /// ```rust
    /// use optimization_engine::constraints::*;
    ///
    /// let a = vec![0.5, 0.1, 0.2, -0.3, -0.6, 0.3, 0., 0.5, 1.0, 0.1, -1.0, -0.4,];
    /// let b = vec![1., 2., -0.5];
    /// let affine_set = AffineSpace::new(a, b);
    /// let mut x = [1., -2., -0.3, 0.5];
    /// affine_set.project(&mut x);
    /// ```
    ///
    /// The result is stored in `x` and it can be verified that $Ax = b$.
    fn project(&self, x: &mut [T]) {
        let n = self.n_cols;
        assert!(x.len() == n, "x has wrong dimension");

        // Step 1: Compute e = Ax - b
        let a = ArrayView2::from_shape((self.n_rows, self.n_cols), &self.a_mat)
            .expect("invalid A shape");
        let x_view = ArrayView1::from(&x[..]);
        let b = ArrayView1::from(&self.b_vec[..]);
        let e = a.dot(&x_view) - b;
        let e_slice: &[T] = e.as_slice().unwrap();

        // Step 2: Solve AA' z = e and compute z
        let z = self.factorizer.solve(e_slice).unwrap();

        // Step 3: Compute x = x - A'z
        let at_z = a.t().dot(&ArrayView1::from(&z[..]));
        for (xi, corr) in x.iter_mut().zip(at_z.iter()) {
            *xi = *xi - *corr;
        }
    }

    /// Affine sets are convex.
    fn is_convex(&self) -> bool {
        true
    }
}
