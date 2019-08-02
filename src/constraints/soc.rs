use super::Constraint;
use crate::matrix_operations;

///
/// A second-order cone (SOC)
///
/// A set of the form `{ (x, t) : ||x|| <= a*t}`, where `a` is a positive
/// scalar.
///
/// Projections on the second-order cone are computed as in H.H. Bauschke's
/// 1996 doctoral dissertation: Projection Algorithms and Monotone Operators
/// (p. 40, Theorem 3.3.6).
pub struct SecondOrderCone {
    alpha: f64,
}

impl SecondOrderCone {
    /// Construct a new instance of SecondOrderCone with parameter `alpha`
    ///
    /// ### Panics
    ///
    /// The method panics if the given parameter `alpha` is nonpositive.
    pub fn new(alpha: f64) -> SecondOrderCone {
        assert!(alpha > 0.0); // alpha must be positive
        SecondOrderCone { alpha }
    }
}

impl Constraint for SecondOrderCone {
    /// Project on the second-order cone (updates the given vector/slice)
    ///
    /// ### Panics
    ///
    /// The methods panics is the length of `x` is less than 2.
    fn project(&self, x: &mut [f64]) {
        // x = (z, r)
        let n = x.len();
        assert!(n >= 2, "x must be of dimension at least 2");
        let z = &x[..n - 1];
        let r = x[n - 1];
        let norm_z = matrix_operations::norm2(z);
        if self.alpha * norm_z <= -r {
            x.iter_mut().for_each(|v| *v = 0.0);
        } else if norm_z > self.alpha * r {
            let beta = (self.alpha * norm_z + r) / (self.alpha.powi(2) + 1.0);
            x[..n - 1]
                .iter_mut()
                .for_each(|v| *v *= self.alpha * beta / norm_z);
            x[n - 1] = beta;
        }
    }
}
