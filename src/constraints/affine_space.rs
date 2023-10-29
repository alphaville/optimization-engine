use super::Constraint;
use modcholesky::ModCholeskySE99;

type OpenMat<T> = ndarray::ArrayBase<ndarray::OwnedRepr<T>, ndarray::Dim<[usize; 2]>>;
type OpenVec<T> = ndarray::ArrayBase<ndarray::OwnedRepr<T>, ndarray::Dim<[usize; 1]>>;

#[derive(Clone)]
/// An affine space here is defined as the set of solutions of a linear equation, $Ax = b$,
/// that is, $\{x\in\mathbb{R}^n: Ax = b\}$, which is an affine space. It is assumed that
/// the matrix $AA^\intercal$ is full-rank.
pub struct AffineSpace {
    a_mat: OpenMat<f64>,
    b_vec: OpenVec<f64>,
    a_times_a_t: OpenMat<f64>,
    l: OpenMat<f64>,
    p: OpenVec<usize>,
    e: OpenVec<f64>,
}

impl AffineSpace {
    /// Construct a new affine space given the matrix $A\in\mathbb{R}^{m\times n}$ and
    /// the vector $b\in\mathbb{R}^m$
    pub fn new(a_data: Vec<f64>, b_data: Vec<f64>) -> Self {
        // Infer dimensions of A and b
        let n_rows = b_data.len();
        let n_elements_a = a_data.len();
        assert!(
            n_elements_a % n_rows == 0,
            "A and b have incompatible dimensions"
        );
        let n_cols = n_elements_a / n_rows;
        // Cast A and b as ndarray structures
        let a_mat = ndarray::Array2::from_shape_vec((n_rows, n_cols), a_data).unwrap();
        let b_vec = ndarray::Array1::from_shape_vec((n_rows,), b_data).unwrap();
        // Compute a permuted Cholesky factorisation of AA'; in particular, we are looking for a
        // minimum-norm matrix E, a permulation matrix P and a lower-trianular L, such that
        //  P(AA' + E)P' = LL'
        // and E should be 0 if A is full rank.
        let a_times_a_t = a_mat.dot(&a_mat.t());
        let res = a_times_a_t.mod_cholesky_se99();
        let l = res.l;
        let p = res.p;
        let e = res.e;
        // Construct and return new AffineSpace structure
        AffineSpace {
            a_mat,
            b_vec,
            a_times_a_t,
            l,
            p,
            e,
        }
    }
}

impl Constraint for AffineSpace {
    fn project(&self, x: &mut [f64]) {
        let x_vec = x.to_vec();
        let x_arr = ndarray::Array1::from_shape_vec((x_vec.len(),), x_vec).unwrap();
        let ax = self.a_mat.dot(&x_arr);
        let err = ax - &self.b_vec;
        println!("err = {:?}", err);
    }

    fn is_convex(&self) -> bool {
        true
    }
}
