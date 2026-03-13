use super::Constraint;

#[derive(Copy, Clone)]
/// Epigraph of the squared Euclidean norm, that is,
/// $$\mathrm{epi}\Vert \cdot \Vert^2 = \\{(x, z)\in\mathbb{R}^{n+1}: \Vert x \Vert^2 \leq z \\}$$
///
/// A point $u = (x, z)$ is represented by a slice `u` of length at least 2:
///
/// - `u[..u.len()-1]` is the vector part `x`,
/// - `u[u.len()-1]` is the scalar part `z`.
pub struct EpiSqNorm {
    /// Tolerance used to accept a candidate root.
    tol_root: f64,

    /// Tolerance used in Newton refinement.
    tol_newton: f64,

    /// Maximum number of Newton iterations.
    max_iter: usize,
}

impl EpiSqNorm {
    /// Constructor
    ///
    /// Creates a new instance of `EpiSqNorm`
    pub fn new() -> Self {
        Self {
            tol_root: 1e-6,
            tol_newton: 1e-10,
            max_iter: 20,
        }
    }

    /// Set `tol_root` parameter
    pub fn with_tol_root(mut self, tol_root: f64) -> Self {
        self.tol_root = tol_root;
        self
    }

    /// Set `tol_newton` parameter
    pub fn with_tol_newton(mut self, tol_newton: f64) -> Self {
        self.tol_newton = tol_newton;
        self
    }

    /// Set `tol_newton` parameter
    pub fn with_max_iter(mut self, max_iter: usize) -> Self {
        self.max_iter = max_iter;
        self
    }

    #[inline]
    fn norm2_squared(x: &[f64]) -> f64 {
        x.iter().map(|xi| xi * xi).sum()
    }

    /// Returns the real roots of $a r^3 + b r^2 + c r + d = 0$.
    fn solve_cubic_real_roots(a: f64, b: f64, c: f64, d: f64) -> Vec<f64> {
        assert!(a != 0.0);

        let p = (3.0 * a * c - b * b) / (3.0 * a * a);
        let q = (2.0 * b.powi(3) - 9.0 * a * b * c + 27.0 * a * a * d) / (27.0 * a * a * a);
        let shift = -b / (3.0 * a);
        let delta = q * q / 4.0 + p * p * p / 27.0;

        if delta > 0.0 {
            let sqrt_delta = delta.sqrt();
            let u = (-q / 2.0 + sqrt_delta).cbrt();
            let v = (-q / 2.0 - sqrt_delta).cbrt();
            vec![u + v + shift]
        } else if delta.abs() <= 1e-14 {
            let u = (-q / 2.0).cbrt();
            vec![2.0 * u + shift, -u + shift]
        } else {
            let rho = (-p * p * p / 27.0).sqrt();
            let phi = (-q / (2.0 * rho)).acos();
            let m = 2.0 * (-p / 3.0).sqrt();

            let r1 = m * (phi / 3.0).cos() + shift;
            let r2 = m * ((phi + 2.0 * std::f64::consts::PI) / 3.0).cos() + shift;
            let r3 = m * ((phi + 4.0 * std::f64::consts::PI) / 3.0).cos() + shift;

            vec![r1, r2, r3]
        }
    }

    /// Computes candidate roots of
    /// $$4 r^3 + 4 θ r^2 + θ^2 r - \Vert x \Vert^2 = 0.$$
    fn cubic_roots(theta: f64, x_norm_sq: f64) -> Vec<f64> {
        let b = 4.0 * theta;
        let c = theta * theta;
        let d = -x_norm_sq;

        let discr = 72.0 * b * c * d - 4.0 * b.powi(3) * d + b * b * c * c
            - 16.0 * c.powi(3)
            - 432.0 * d * d;
        let discr0 = b * b - 12.0 * c;

        if discr.abs() <= 1e-14 {
            if discr0.abs() <= 1e-14 {
                return vec![-b / 12.0];
            } else {
                let single = (16.0 * b * c - 144.0 * d - b.powi(3)) / (4.0 * discr0);
                let double = (36.0 * d - b * c) / (2.0 * discr0);
                return vec![single, double];
            }
        }
        Self::solve_cubic_real_roots(4.0, b, c, d)
    }

    /// Newton refinement for
    /// 4 z^3 + 4 θ z^2 + θ^2 z - ||x||² = 0.
    fn newton_solver(x_norm_sq: f64, theta: f64, z0: f64, max_iter: usize, tolx: f64) -> f64 {
        let mut zsol = z0;
        let mut zprev = z0 - 1.0;

        for _ in 0..max_iter {
            let numerator =
                4.0 * zsol.powi(3) + 4.0 * theta * zsol.powi(2) + theta * theta * zsol - x_norm_sq;
            let denominator = 12.0 * zsol.powi(2) + 8.0 * theta * zsol + theta * theta;

            let next = zsol - numerator / denominator;
            let err = (next - zprev).abs();

            zprev = zsol;
            zsol = next;

            if err < tolx {
                return zsol;
            }
        }

        zsol
    }
}

impl Constraint for EpiSqNorm {
    /// Project a point onto the epigraph of the squared Euclidean norm.
    ///
    /// A point $(y, t)$ is represented by a slice `x` of length at least `2`:
    ///
    /// - `x[..x.len()-1]` is the vector part $y$,
    /// - `x[x.len()-1]` is the scalar part $t$.
    ///
    /// The set is
    /// $$C = \\{ (u, s) \in \mathbb{R}^n \times \mathbb{R} : \|u\|_2^2 \le s \\}.$$
    /// This method computes the Euclidean projection of $(y, t)$ onto $C$.
    /// If the input is already feasible, that is, if $\Vert y \Vert_2^2 \leq t$,
    /// then the input is left unchanged.
    ///
    /// # Method
    ///
    /// For an infeasible point $(y, t)$, the projection $(u, s)$ satisfies the
    /// optimality conditions $y - u = 2 (s - t) u$, and $s = \Vert u\Vert_2^2$.
    /// Solving for $u$, we have
    /// $$u = \frac{y}{1 + 2(s - t)}.$$
    /// Substituting this into $s = \Vert u \Vert_2^2$ yields the scalar equation
    /// $$s \left(1 + 2(s - t)\right)^2 = \Vert y \Vert_2^2.$$
    /// Defining $\theta = 1 - 2t$, this becomes the cubic polynomial
    /// $$4s^3 + 4\theta s^2 + \theta^2 s - \|y\|_2^2 = 0.$$
    ///
    /// The implementation follows this procedure:
    ///
    /// 1. Check whether the point is already in the epigraph.
    /// 2. Form the cubic equation in the projected scalar variable $s$.
    /// 3. Compute its real candidate roots.
    /// 4. Select a root that is consistent with
    ///
    /// $$
    /// u = \frac{y}{1 + 2(s - t)}
    /// $$
    ///
    /// and
    ///
    /// $$
    /// s \approx \|u\|_2^2.
    /// $$
    ///
    /// 5. Refine the selected root with a few Newton iterations applied to
    ///
    /// $$
    /// f(s) = 4s^3 + 4\theta s^2 + \theta^2 s - \Vert y \Vert_2^2.
    /// $$
    ///
    /// 6. Recover the projected vector component from
    ///
    /// $$
    /// u = \frac{y}{1 + 2(s - t)}.
    /// $$
    ///
    /// # Notes
    ///
    /// - The projection is taken with respect to the standard Euclidean norm on
    ///   the product space.
    /// - The last entry of `x` is interpreted as the epigraph variable.
    /// - The set is convex, so the projection is uniquely defined.
    ///
    /// # Panics
    ///
    /// Panics if:
    ///
    /// - `x.len() < 2`,
    /// - no admissible real root is found for the cubic equation.
    fn project(&self, x: &mut [f64]) {
        assert!(
            x.len() >= 2,
            "EpiSqNorm projection requires a slice of length at least 2"
        );

        let n = x.len();
        let z = x[n - 1];
        let x_vec = &x[..n - 1];
        let x_norm_sq = Self::norm2_squared(x_vec);

        // Already feasible
        if x_norm_sq <= z {
            return;
        }

        let theta = 1.0 - 2.0 * z;
        let roots = Self::cubic_roots(theta, x_norm_sq);

        let mut z_proj = None;

        for &r in roots.iter() {
            let denom = 1.0 + 2.0 * (r - z);
            if denom.abs() <= 1e-15 {
                continue;
            }

            let candidate_norm_sq = x_vec.iter().map(|xi| (xi / denom).powi(2)).sum::<f64>();

            if (candidate_norm_sq - r).abs() <= self.tol_root {
                z_proj = Some(r);
                break;
            }
        }

        let mut z_sol = z_proj.expect("No admissible real root found in EpiSqNorm::project");

        z_sol = Self::newton_solver(x_norm_sq, theta, z_sol, self.max_iter, self.tol_newton);

        let denom = 1.0 + 2.0 * (z_sol - z);

        x[..n - 1].iter_mut().for_each(|xi| *xi /= denom);
        x[n - 1] = z_sol;
    }

    /// The epigraph of the squared Euclidean norm is convex,
    /// so this returns `true`.
    fn is_convex(&self) -> bool {
        true
    }
}
