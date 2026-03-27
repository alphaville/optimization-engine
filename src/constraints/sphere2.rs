use crate::numeric::cast;

use super::Constraint;
use num::Float;
use std::iter::Sum;

fn norm2_squared_diff<T: Float>(a: &[T], b: &[T]) -> T {
    assert_eq!(a.len(), b.len());
    a.iter()
        .zip(b.iter())
        .fold(T::zero(), |sum, (&x, &y)| sum + (x - y) * (x - y))
}

#[derive(Copy, Clone)]
/// A Euclidean sphere, that is, a set given by $S_2^r = \\{x \in \mathbb{R}^n {}:{} \Vert{}x{}\Vert = r\\}$
/// or a Euclidean sphere centered at a point $x_c$, that is, $S_2^{x_c, r} = \\{x \in \mathbb{R}^n {}:{} \Vert{}x-x_c{}\Vert = r\\}$
pub struct Sphere2<'a, T = f64> {
    center: Option<&'a [T]>,
    radius: T,
}

impl<'a, T: Float> Sphere2<'a, T> {
    /// Construct a new Euclidean sphere with given center and radius
    /// If no `center` is given, then it is assumed to be in the origin
    ///
    /// # Example
    ///
    /// ```
    /// use optimization_engine::constraints::{Constraint, Sphere2};
    ///
    /// let sphere = Sphere2::new(None, 1.0);
    /// let mut x = [3.0, 4.0];
    /// sphere.project(&mut x);
    /// ```
    pub fn new(center: Option<&'a [T]>, radius: T) -> Self {
        assert!(radius > T::zero());
        Sphere2 { center, radius }
    }
}

impl<'a, T> Constraint<T> for Sphere2<'a, T>
where
    T: Float + Sum<T>,
{
    /// Projection onto the sphere, $S_{r, c}$ with radius $r$ and center $c$.
    /// If $x\neq c$, the projection is uniquely defined by
    ///
    /// $$
    /// P_{S_{r, c}}(x) = c + r\frac{x-c}{\Vert{}x-c\Vert_2},
    /// $$
    ///
    /// but for $x=c$, the projection is multi-valued. In particular, let
    /// $y = P_{S_{r, c}}(c)$. Then $y_1 = c_1 + r$ and $y_i = c_i$ for
    /// $i=2,\ldots, n$.
    ///
    /// ## Arguments
    ///
    /// - `x`: The given vector $x$ is updated with the projection on the set
    ///
    /// ## Panics
    ///
    /// Panics if `x` is empty or, when a center is provided, if `x` and
    /// `center` have incompatible dimensions.
    ///
    fn project(&self, x: &mut [T]) {
        let epsilon = cast::<T>(1e-12);
        assert!(!x.is_empty(), "x must be nonempty");
        if let Some(center) = &self.center {
            assert_eq!(
                x.len(),
                center.len(),
                "x and center have incompatible dimensions"
            );
            let norm_difference = norm2_squared_diff(x, center).sqrt();
            if norm_difference <= epsilon {
                x.copy_from_slice(center);
                x[0] = x[0] + self.radius;
                return;
            }
            x.iter_mut().zip(center.iter()).for_each(|(x, c)| {
                *x = *c + self.radius * (*x - *c) / norm_difference;
            });
        } else {
            let norm_x = crate::matrix_operations::norm2(x);
            if norm_x <= epsilon {
                x[0] = x[0] + self.radius;
                return;
            }
            let norm_over_radius = self.radius / norm_x;
            x.iter_mut().for_each(|x_| *x_ = *x_ * norm_over_radius);
        }
    }

    /// Returns false (the sphere is not a convex set)
    ///
    fn is_convex(&self) -> bool {
        false
    }
}
