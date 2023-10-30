use crate::matrix_operations;

use super::Constraint;

#[derive(Copy, Clone)]
/// A
pub struct EpigraphSquaredNorm {}

impl EpigraphSquaredNorm {
    /// A
    pub fn new() -> Self {
        EpigraphSquaredNorm {}
    }
}

impl Constraint for EpigraphSquaredNorm {
    fn project(&self, x: &mut [f64]) {
        let nx = x.len() - 1;
        let z: &[f64] = &x[..nx];
        let t: f64 = x[nx];
        let norm_z_sq = matrix_operations::norm2_squared(&z);
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
                return;
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
            zsol = zsol - p_z / dp_z;
            refinement_error = p_z.abs();
            iter += 1;
        }
        right_root = zsol;

        // Projection
        for i in 0..nx {
            x[i] /= scaling;
        }
        x[nx] = right_root;
    }

    fn is_convex(&self) -> bool {
        true
    }
}
