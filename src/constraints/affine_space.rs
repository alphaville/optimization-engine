use super::Constraint;
use crate::matrix_operations;
use crate::CholeskyFactorizer;

use ndarray::{ArrayView1, ArrayView2};

#[derive(Clone)]
/// An affine space here is defined as the set of solutions of a linear equation, $Ax = b$,
/// that is, $E=\\{x\in\mathbb{R}^n: Ax = b\\}$, which is an affine space. It is assumed that
/// the matrix $AA^\intercal$ is full-rank.
pub struct AffineSpace {
    a_mat: Vec<f64>,
    b_vec: Vec<f64>,
    factorizer: CholeskyFactorizer<f64>,
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
            n_elements_a.is_multiple_of(n_rows),
            "A and b have incompatible dimensions"
        );
        let n_cols = n_elements_a / n_rows;
        let aat = matrix_operations::mul_a_at(&a, n_rows, n_cols).unwrap();
        let mut factorizer = CholeskyFactorizer::new(n_rows);
        factorizer.factorize(&aat).unwrap();
        AffineSpace {
            a_mat: a,
            b_vec: b,
            factorizer: factorizer,
            n_rows,
            n_cols,
        }
    }
}

impl Constraint for AffineSpace {
    /// Projection onto the set $E = \\{x: Ax = b\\}$, which is computed by
    /// $$P_E(x) = x - A^\intercal z(x),$$
    /// where $z$ is the solution of the linear system
    /// $$(AA^\intercal)z = Ax - b,$$
    /// which has a unique solution provided $A$ has full row rank. The linear system
    /// is solved by computing the Cholesky factorization of $AA^\intercal$, which is
    /// done using ...
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
    fn project(&self, x: &mut [f64]) {
        let n = self.n_cols;
        assert!(x.len() == n, "x has wrong dimension");

        // Step 1: Compute e = Ax - b
        let a = ArrayView2::from_shape((self.n_rows, self.n_cols), &self.a_mat)
            .expect("invalid A shape");
        let x_view = ArrayView1::from(&x[..]);
        let b = ArrayView1::from(&self.b_vec[..]);
        let e = a.dot(&x_view) - b;
        let e_slice: &[f64] = e.as_slice().unwrap();

        // Step 2: Solve AA' z = e and compute z
        let z = self.factorizer.solve(e_slice).unwrap();

        // Step 3: Compute x = x - A'z
        let at_z = a.t().dot(&ArrayView1::from(&z[..]));
        for (xi, corr) in x.iter_mut().zip(at_z.iter()) {
            *xi -= *corr;
        }
    }

    /// Affine sets are convex.
    fn is_convex(&self) -> bool {
        true
    }
}
