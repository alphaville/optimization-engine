use super::Constraint;

use oxiblas_lapack::cholesky::{Cholesky, CholeskyError};
use oxiblas_matrix::Mat;
use oxiblas_blas::level3::gemm;

type OpenMat<T> = Mat<T>;
type OpenVec<T> = Vec<T>;

#[derive(Clone)]
/// An affine space here is defined as the set of solutions of a linear equation, $Ax = b$,
/// that is, $E=\\{x\in\mathbb{R}^n: Ax = b\\}$, which is an affine space. It is assumed that
/// the matrix $AA^\intercal$ is full-rank.
pub struct AffineSpace {
    a_mat: OpenMat<f64>,
    b_vec: OpenVec<f64>,
    l: OpenMat<f64>,
    n_rows: usize,
    n_cols: usize,
}
impl AffineSpace {
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
    pub fn new(a: Vec<f64>, b: Vec<f64>) -> Self {
        let n_rows = b.len();
        let n_elements_a = a.len();

        assert!(n_rows > 0, "b must not be empty");
        assert!(
            n_elements_a % n_rows == 0,
            "A and b have incompatible dimensions"
        );

        let n_cols = n_elements_a / n_rows;

        // Build A from row-major input
        let mut a_mat = Mat::<f64>::zeros(n_rows, n_cols);
        for i in 0..n_rows {
            for j in 0..n_cols {
                a_mat[(i, j)] = a[i * n_cols + j];
            }
        }

        let b_vec = b;

        // Compute A * A^T
        let a_t = a_mat.transpose();
        let mut a_times_a_t = Mat::<f64>::zeros(n_rows, n_rows);
        gemm(1.0, a_mat.as_ref(), a_t.as_ref(), 0.0, a_times_a_t.as_mut());

        // Cholesky of A A^T = L L^T
        let chol = Cholesky::compute(a_times_a_t.as_ref()).unwrap();
        let l = chol.l_factor();

        AffineSpace {
            a_mat,
            b_vec,
            l,
            n_rows,
            n_cols,
        }
    }
}