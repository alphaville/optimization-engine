use super::Constraint;
use crate::matrix_operations;

///
/// A second-order cone (SOC)
///
/// A set of the form `{ (x, t) : ||x|| <= a*t}`, where `a` is a positive
/// scalar
pub struct SecondOrderCone {
    alpha: f64,
}

impl SecondOrderCone {
    /// Construct a new rectangle with given `xmin` and `xmax`
    pub fn new(alpha: f64) -> SecondOrderCone {
        assert!(alpha > 0.0); // alpha must be positive
        SecondOrderCone { alpha }
    }
}

impl Constraint for SecondOrderCone {
    fn project(&self, x: &mut [f64]) {
        // x = (z, r)
        let n = x.len();
        let z = &x[..n - 1];
        let r = x[n - 1];
        let norm_z = matrix_operations::norm2(z);
        if self.alpha * norm_z <= -r {
            x.iter_mut().for_each(|v| *v = 0.0);
        } else if norm_z > self.alpha * r {
            let beta = (self.alpha * norm_z + r) / (self.alpha.powi(2) + 1.0);
            x[..n - 1].iter_mut().for_each(|v| *v *= self.alpha * beta);
            x[n - 1] = beta;
        }
    }
}
