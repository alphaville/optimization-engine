use super::Constraint;

extern crate modcholesky;
extern crate ndarray;

use modcholesky::ModCholeskySE99;
use ndarray::{Array1, Array2, ArrayBase, Dim, OwnedRepr};

type OpenMat<T> = ArrayBase<OwnedRepr<T>, Dim<[usize; 2]>>;
type OpenVec<T> = ArrayBase<OwnedRepr<T>, Dim<[usize; 1]>>;

#[derive(Clone)]
/// An affine space here is defined as the set of solutions of a linear equation, $Ax = b$,
/// that is, $E=\\{x\in\mathbb{R}^n: Ax = b\\}$, which is an affine space. It is assumed that
/// the matrix $AA^\intercal$ is full-rank.
pub struct AffineSpace {
    a_mat: OpenMat<f64>,
    b_vec: OpenVec<f64>,
    l: OpenMat<f64>,
    p: OpenVec<usize>,
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
        // Infer dimensions of A and b
        let n_rows = b.len();
        let n_elements_a = a.len();
        assert!(
            n_elements_a % n_rows == 0,
            "A and b have incompatible dimensions"
        );
        let n_cols = n_elements_a / n_rows;
        // Cast A and b as ndarray structures
        let a_mat = Array2::from_shape_vec((n_rows, n_cols), a).unwrap();
        let b_vec = Array1::from_shape_vec((n_rows,), b).unwrap();
        // Compute a permuted Cholesky factorisation of AA'; in particular, we are looking for a
        // minimum-norm matrix E, a permulation matrix P and a lower-trianular L, such that
        //  P(AA' + E)P' = LL'
        // and E should be 0 if A is full rank.
        let a_times_a_t = a_mat.dot(&a_mat.t());
        let res = a_times_a_t.mod_cholesky_se99();
        let l = res.l;
        let p = res.p;

        // Construct and return new AffineSpace structure
        AffineSpace {
            a_mat,
            b_vec,
            l,
            p,
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
    /// is solved by computing the Cholesky factorisation of $AA^\intercal$, which is
    /// done using [`modcholesky`](https://crates.io/crates/modcholesky).
    ///
    /// ## Arguments
    ///
    /// - `x`: The given vector $x$ is updated with the projection on the set
    ///
    fn project(&self, x: &mut [f64]) {
        let m = self.n_rows;
        let n = self.n_cols;
        let chol = &self.l;
        let perm = &self.p;

        assert!(x.len() == n, "x has wrong dimension");
        let x_vec = x.to_vec();
        let x_arr = Array1::from_shape_vec((n,), x_vec).unwrap();
        let ax = self.a_mat.dot(&x_arr);
        let err = ax - &self.b_vec;

        // Step 1: Solve Ly = b(P)
        // TODO: Make `y` into an attribute; however, to do this, we need to change
        // &self to &mut self, which will require a mild refactoring
        let mut y = vec![0.; m];
        for i in 0..m {
            let mut sum = 0.;
            for j in 0..i {
                sum += chol[(i, j)] * y[j];
            }
            y[i] = (err[perm[i]] - sum) / chol[(i, i)];
        }

        // Step 2: Solve L'z(P) = y
        let mut z = vec![0.; m];
        for i in 1..=m {
            z[perm[m - i]] = y[m - i];
            for j in 1..i {
                z[perm[m - i]] -= chol[(m - j, m - i)] * z[perm[m - j]];
            }
            z[perm[m - i]] /= chol[(m - i, m - i)];
        }

        // Step 3: Determine A' * z
        let z_arr = Array1::from_shape_vec((self.n_rows,), z).unwrap();
        let w = self.a_mat.t().dot(&z_arr);

        // Step 4: x <-- x - A'(AA')\(Ax-b)
        x.iter_mut().zip(w.iter()).for_each(|(xi, wi)| *xi -= wi);
    }

    /// Affine sets are convex.
    fn is_convex(&self) -> bool {
        true
    }
}
