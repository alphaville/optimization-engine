use crate::{matrix_operations, numeric::cast};

use super::Constraint;
use num::Float;
use roots::FloatType;
use std::iter::Sum;

#[derive(Copy, Clone, Default)]
/// The epigraph of the squared Euclidean norm, that is,
/// $$
/// X = \\{x = (z, t) \in \mathbb{R}^{n}\times \mathbb{R} : \Vert z\Vert_2^2 \leq t \\}.
/// $$
///
/// A point is represented by a slice `x` whose last entry is the scalar
/// component `t`, while the preceding entries form the vector component `z`.
pub struct EpigraphSquaredNorm {}

impl EpigraphSquaredNorm {
    /// Create a new instance of the epigraph of the squared norm.
    ///
    /// Note that you do not need to specify the dimension.
    #[must_use]
    pub fn new() -> Self {
        EpigraphSquaredNorm {}
    }
}

impl<T> Constraint<T> for EpigraphSquaredNorm
where
    T: Float + FloatType + Sum<T>,
{
    /// Project on the epigraph of the squared Euclidean norm.
    ///
    /// Let the input be represented as $(z,t)$, where `z` is the vector formed
    /// by the first `x.len() - 1` entries of `x`, and `t` is the last entry.
    /// This method computes the Euclidean projection of $(z,t)$ onto
    ///
    /// $$
    /// \mathrm{epi}\Vert \cdot\Vert_2^2 = \\{(u,s) \in \mathbb{R}^n \times \mathbb{R} : \Vert u\Vert_2^2 \leq s \\}.
    /// $$
    ///
    /// If the point is already feasible, that is, if $\Vert z\Vert_2^2 \leq t,$
    /// then the input is left unchanged, otherwise, the projection is computed using
    /// the methodology described
    /// [here](https://mathematix.wordpress.com/2017/05/02/projection-on-the-epigraph-of-the-squared-euclidean-norm/).
    ///
    /// ## Arguments
    ///
    /// - `x`: The given vector `x` is updated with the projection on the set
    ///
    /// ## Panics
    ///
    /// Panics if:
    ///
    /// - `x.len() < 2`,
    /// - no admissible real root is found,
    /// - the Newton derivative becomes too small,
    /// - the final scaling factor is numerically singular.
    ///
    /// ## Example
    ///
    /// ```rust
    /// use optimization_engine::constraints::*;
    ///
    /// let epi = EpigraphSquaredNorm::new();
    ///
    /// // Here, z = [1., 2., 3.] and t = 4.
    /// let mut x = [1., 2., 3., 4.];
    ///
    /// epi.project(&mut x);
    /// ```
    fn project(&self, x: &mut [T]) {
        assert!(
            x.len() >= 2,
            "EpigraphSquaredNorm::project requires x.len() >= 2"
        );

        let nx = x.len() - 1;
        let z = &x[..nx];
        let t = x[nx];
        let norm_z_sq = matrix_operations::norm2_squared(z);

        // Already feasible
        if norm_z_sq <= t {
            return;
        }

        // Cubic:
        // 4 r^3 + 4 theta r^2 + theta^2 r - ||z||^2 = 0
        let theta = cast::<T>(1.0) - cast::<T>(2.0) * t;
        let a3 = cast::<T>(4.0);
        let a2 = cast::<T>(4.0) * theta;
        let a1 = theta * theta;
        let a0 = -norm_z_sq;

        let cubic_poly_roots = roots::find_roots_cubic(a3, a2, a1, a0);

        let root_tol = cast::<T>(1e-6);
        let mut right_root: Option<T> = None;

        // Pick the first admissible real root
        for &ri in cubic_poly_roots.as_ref().iter() {
            let denom = cast::<T>(1.0) + cast::<T>(2.0) * (ri - t);

            // We need a valid scaling and consistency with ||z_proj||^2 = ri
            if denom > cast::<T>(0.0) {
                let candidate_norm_sq = norm_z_sq / (denom * denom);
                if num::Float::abs(candidate_norm_sq - ri) <= root_tol {
                    right_root = Some(ri);
                    break;
                }
            }
        }

        let mut zsol =
            right_root.expect("EpigraphSquaredNorm::project: no admissible real root found");

        // Newton refinement
        let newton_max_iters: usize = 5;
        let newton_eps = cast::<T>(1e-14);

        for _ in 0..newton_max_iters {
            let zsol_sq = zsol * zsol;
            let zsol_cb = zsol_sq * zsol;

            let p_z = a3 * zsol_cb + a2 * zsol_sq + a1 * zsol + a0;
            if num::Float::abs(p_z) <= newton_eps {
                break;
            }

            let dp_z = cast::<T>(3.0) * a3 * zsol_sq + cast::<T>(2.0) * a2 * zsol + a1;
            assert!(
                num::Float::abs(dp_z) > cast::<T>(1e-15),
                "EpigraphSquaredNorm::project: Newton derivative too small"
            );

            zsol = zsol - p_z / dp_z;
        }

        let right_root = zsol;
        let scaling = cast::<T>(1.0) + cast::<T>(2.0) * (right_root - t);

        assert!(
            num::Float::abs(scaling) > cast::<T>(1e-15),
            "EpigraphSquaredNorm::project: scaling factor too small"
        );

        // Projection
        for xi in x.iter_mut().take(nx) {
            *xi = *xi / scaling;
        }
        x[nx] = right_root;
    }

    /// This is a convex set, so this function returns `true`.
    fn is_convex(&self) -> bool {
        true
    }
}
