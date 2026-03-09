use super::Constraint;
use crate::CholeskyFactoriser;
use crate::matrix_operations;

#[derive(Clone)]
/// An affine space here is defined as the set of solutions of a linear equation, $Ax = b$,
/// that is, $E=\\{x\in\mathbb{R}^n: Ax = b\\}$, which is an affine space. It is assumed that
/// the matrix $AA^\intercal$ is full-rank.
pub struct AffineSpace {
    a_mat: Vec<f64>,
    b_vec: Vec<f64>,
    l: Vec<f64>,
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
        println!("n_rows = {}", n_rows);
        let n_elements_a = a.len();
        assert!(n_rows > 0, "b must not be empty");
        assert!(
            n_elements_a % n_rows == 0,
            "A and b have incompatible dimensions"
        );
        let n_cols = n_elements_a / n_rows;
        let aat = matrix_operations::mul_a_at(&a, n_rows, n_cols).unwrap();
        println!("A = {:?}", a);
        println!("AA' = {:?}", aat);
        let mut factoriser = CholeskyFactoriser::new(3);
        factoriser.factorize(&aat).unwrap();
        let l = factoriser.l();
        println!("L = {:?}", l);
        AffineSpace {
            a_mat: a,
            b_vec: b,
            l: l.to_vec(),
            n_rows,
            n_cols,
        }
    }
}