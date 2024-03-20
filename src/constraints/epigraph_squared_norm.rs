use crate::matrix_operations;

use super::Constraint;

#[derive(Copy, Clone, Default)]
/// The epigraph of the squared Eucliden norm is a set of the form
/// $X = \\{x = (z, t) \in \mathbb{R}^{n}\times \mathbb{R} {}:{} \\|z\\|^2 \leq t \\}.$
pub struct EpigraphSquaredNorm {}

impl EpigraphSquaredNorm {
    /// Create a new instance of the epigraph of the squared norm.
    ///
    /// Note that you do not need to specify the dimension.
    pub fn new() -> Self {
        EpigraphSquaredNorm {}
    }
}

impl Constraint for EpigraphSquaredNorm {
    ///Project on the epigraph of the squared Euclidean norm.
    ///
    /// The projection is computed as detailed
    /// [here](https://mathematix.wordpress.com/2017/05/02/projection-on-the-epigraph-of-the-squared-euclidean-norm/).
    ///
    /// ## Arguments
    /// - `x`: The given vector $x$ is updated with the projection on the set
    ///
    /// ## Example
    ///
    /// ```rust
    /// use optimization_engine::constraints::*;
    ///
    /// let epi = EpigraphSquaredNorm::new();
    /// let mut x = [1., 2., 3., 4.];    
    /// epi.project(&mut x);
    /// ```
    fn project(&self, x: &mut [f64]) {
        let nx = x.len() - 1;
        assert!(nx > 0, "x must have a length of at least 2");
        let z: &[f64] = &x[..nx];
        let t: f64 = x[nx];
        let norm_z_sq = matrix_operations::norm2_squared(z);
        if norm_z_sq <= t {
            return;
        }

        let theta = 1. - 2. * t;
        let a3 = 4.;
        let a2 = 4. * theta;
        let a1 = theta * theta;
        let a0 = -norm_z_sq;

        let cubic_poly_roots = roots::find_roots_cubic(a3, a2, a1, a0);
        let mut right_root = f64::NAN;
        let mut scaling = f64::NAN;

        // Find right root
        cubic_poly_roots.as_ref().iter().for_each(|ri| {
            if *ri > 0. {
                let denom = 1. + 2. * (*ri - t);
                if ((norm_z_sq / (denom * denom)) - *ri).abs() < 1e-6 {
                    right_root = *ri;
                    scaling = denom;
                }
            }
        });

        // Refinement of root with Newton-Raphson
        let mut refinement_error = 1.;
        let newton_max_iters: usize = 5;
        let newton_eps = 1e-14;
        let mut zsol = right_root;
        let mut iter = 0;
        while refinement_error > newton_eps && iter < newton_max_iters {
            let zsol_sq = zsol * zsol;
            let zsol_cb = zsol_sq * zsol;
            let p_z = a3 * zsol_cb + a2 * zsol_sq + a1 * zsol + a0;
            let dp_z = 3. * a3 * zsol_sq + 2. * a2 * zsol + a1;
            zsol -= p_z / dp_z;
            refinement_error = p_z.abs();
            iter += 1;
        }
        right_root = zsol;

        // Projection
        for xi in x.iter_mut().take(nx) {
            *xi /= scaling;
        }
        x[nx] = right_root;
    }

    /// This is a convex set, so this function returns `True`
    fn is_convex(&self) -> bool {
        true
    }
}
