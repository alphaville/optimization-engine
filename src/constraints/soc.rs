use super::Constraint;
use crate::matrix_operations;
use num::Float;
use std::iter::Sum;

#[derive(Clone, Copy)]
///
/// A second-order cone (SOC)
///
/// A set of the form
///
/// $$
/// C_{\alpha} = \\{x=(y, t) \in \mathbb{R}^{n+1}: t\in\mathbb{R}, \Vert{}y\Vert \leq \alpha{}t\\},
/// $$
///
/// where $\alpha$ is a positive scalar.
///
/// Projections on the second-order cone are computed as in H.H. Bauschke's
/// 1996 doctoral dissertation: Projection Algorithms and Monotone Operators
/// (p. 40, Theorem 3.3.6).
///
pub struct SecondOrderCone<T = f64> {
    alpha: T,
}

impl<T: Float> SecondOrderCone<T> {
    /// Construct a new instance of `SecondOrderCone` with parameter `alpha`.
    ///
    /// A second-order cone with parameter `alpha` is the set
    /// $C_\alpha = \\{x=(y, t) \in \mathbb{R}^{n+1}: t\in\mathbb{R}, \Vert{}y\Vert \leq \alpha t\\}$,
    /// where $\alpha$ is a positive parameter, and projections are computed
    /// according to Theorem 3.3.6 in H.H. Bauschke's 1996 doctoral dissertation:
    /// [Projection Algorithms and Monotone Operators](http://summit.sfu.ca/system/files/iritems1/7015/b18025766.pdf)
    /// (page 40).
    ///
    /// # Arguments
    ///
    /// - `alpha`: parameter $\alpha$
    ///
    /// # Panics
    ///
    /// The method panics if the given parameter `alpha` is nonpositive.
    ///
    /// # Example
    ///
    /// ```
    /// use optimization_engine::constraints::{Constraint, SecondOrderCone};
    ///
    /// let cone = SecondOrderCone::new(1.0);
    /// let mut x = [2.0, 0.0, 0.5];
    /// cone.project(&mut x);
    /// ```
    pub fn new(alpha: T) -> SecondOrderCone<T> {
        assert!(alpha > T::zero()); // alpha must be positive
        SecondOrderCone { alpha }
    }
}

impl<T> Constraint<T> for SecondOrderCone<T>
where
    T: Float + Sum<T>,
{
    /// Project onto the second-order cone.
    ///
    /// # Arguments
    ///
    /// - `x`: (in) vector to be projected onto the current second-order cone,
    ///   (out) its projection onto that cone
    ///
    /// # Panics
    ///
    /// This method panics if the length of `x` is less than 2.
    ///
    fn project(&self, x: &mut [T]) {
        // x = (z, r)
        let n = x.len();
        assert!(n >= 2, "x must be of dimension at least 2");
        let z = &x[..n - 1];
        let r = x[n - 1];
        let norm_z = matrix_operations::norm2(z);
        if self.alpha * norm_z <= -r {
            x.iter_mut().for_each(|v| *v = T::zero());
        } else if norm_z > self.alpha * r {
            let beta = (self.alpha * norm_z + r) / (self.alpha.powi(2) + T::one());
            x[..n - 1]
                .iter_mut()
                .for_each(|v| *v = *v * self.alpha * beta / norm_z);
            x[n - 1] = beta;
        }
    }

    fn is_convex(&self) -> bool {
        true
    }
}
