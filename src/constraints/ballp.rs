use super::Constraint;

#[derive(Copy, Clone)]
/// An $\\ell_p$ ball, that is,
/// $$B_p^r = \\{ x \\in R^n : \Vert x \Vert_p \\leq r \\}$$
/// or a translated ball
/// $$B_p^{x_c, r} = \\{ x \\in \mathbb{R}^n : \Vert x - x_c \Vert_p  \\leq r \\},$$
/// with $1 < p < \infty$.
///
/// # Projection problem
///
/// Given a vector $x$, projection onto the ball means solving
///
/// $$\Pi_{B_p^r}(x) = \mathrm{argmin}_{z \in B_p^r} \frac{1}{2}\Vert z - x \Vert_2^2.$$
///
/// If $x$ already belongs to the ball, the projection is $x$ itself.
/// Otherwise, the projection lies on the boundary of the ball.
///
/// # Numerical method
///
/// For general $1 < p < \infty$, the projection does not admit a simple
/// closed-form expression. This implementation computes it numerically using
/// the optimality conditions of the constrained problem.
///
/// For a ball centered at the origin, the projection is characterized by
///
/// $$z_i = \operatorname{sign}(x_i)\,u_i(\lambda),$$
///
/// where each scalar $u_i(\lambda) \geq 0$ solves
///
/// $$u_i + \lambda p\,u_i^{p-1} = |x_i|,$$
///
/// and the Lagrange multiplier $\lambda \geq 0$ is chosen so that the projected
/// vector satisfies the feasibility condition $\|z\|_p = r.$
///
/// The implementation uses:
///
/// - an *outer bisection loop* to determine the multiplier $\lambda$,
/// - an *inner safeguarded Newton method* to solve the scalar nonlinear
///   equation for each coordinate.
///
/// The Newton iteration is combined with bracketing and a bisection fallback,
/// which improves robustness.
///
/// # Translated balls
///
/// If the ball has a center $x_c$, projection is performed by translating the
/// input vector to the origin, projecting onto the origin-centered ball, and
/// translating the result back:
///
/// $$\Pi_{B_p^{x_c, r}}(x) = x_c + \Pi_{B_p^r}(x - x_c).$$
///
/// # Convexity
///
/// Since this module assumes `p > 1.0`, every [`BallP`] is convex, and therefore
/// [`Constraint::project`] computes a unique Euclidean projection.
///
/// # Examples
///
/// Project onto the unit \(\ell_p\)-ball centered at the origin:
///
/// ```
/// use optimization_engine::constraints::{BallP, Constraint};
///
/// let ball = BallP::new(None, 1.0, 1.5, 1e-10, 100);
/// let mut x = vec![3.0, -1.0, 2.0];
/// ball.project(&mut x);
/// ```
///
/// Project onto a translated \(\ell_p\)-ball:
///
/// ```
/// use optimization_engine::constraints::{BallP, Constraint};
///
/// let center = vec![1.0, 1.0, 1.0];
/// let ball = BallP::new(Some(&center), 2.0, 3.0, 1e-10, 100);
/// let mut x = vec![4.0, -1.0, 2.0];
/// ball.project(&mut x);
/// ```
///
/// # Notes
///
/// - The projection is with respect to the *Euclidean norm*
/// - The implementation is intended for general finite $p > 1.0$. If you need
/// to project on a $\Vert{}\cdot{}\Vert_1$-ball or an $\Vert{}\cdot{}
/// \Vert_\infty$-ball, use the implementations in [`Ball1`](crate::constraints::Ball1) and [`BallInf`](crate::constraints::BallInf).
/// - Do not use this struct to project on a Euclidean ball; the implementation
/// in [`Ball2`](crate::constraints::Ball2) is more efficient
/// - The quality and speed of the computation depend on the chosen numerical
///   tolerance and iteration limit.
pub struct BallP<'a> {
    /// Optional center of the ball.
    ///
    /// If `None`, the ball is centered at the origin.
    /// If `Some(center)`, the ball is centered at `center`.
    center: Option<&'a [f64]>,

    /// Radius of the ball.
    ///
    /// Must be strictly positive.
    radius: f64,

    /// Exponent of the norm.
    ///
    /// Must satisfy `p > 1.0` and be finite.
    p: f64,

    /// Numerical tolerance used by the outer bisection on the Lagrange
    /// multiplier and by the inner Newton solver.
    tolerance: f64,

    /// Maximum number of iterations used by the outer bisection and
    /// the inner Newton solver.
    max_iter: usize,
}

impl<'a> BallP<'a> {
    /// Construct a new l_p ball with given center, radius, and exponent.
    ///
    /// - `center`: if `None`, the ball is centered at the origin
    /// - `radius`: radius of the ball
    /// - `p`: norm exponent, must satisfy `p > 1.0` and be finite
    /// - `tolerance`: tolerance for the numerical solvers
    /// - `max_iter`: maximum number of iterations for the numerical solvers
    pub fn new(
        center: Option<&'a [f64]>,
        radius: f64,
        p: f64,
        tolerance: f64,
        max_iter: usize,
    ) -> Self {
        assert!(radius > 0.0);
        assert!(p > 1.0 && p.is_finite());
        assert!(tolerance > 0.0);
        assert!(max_iter > 0);

        BallP {
            center,
            radius,
            p,
            tolerance,
            max_iter,
        }
    }

    #[inline]
    /// Computes the $p$-norm of a given vector
    ///
    /// The $p$-norm of a vector $x\in \mathbb{R}^n$ is given by
    /// $$\Vert x \Vert_p = \left(\sum_{i=1}^{n} |x_i|^p\right)^{1/p},$$
    /// for $p > 1$.
    fn lp_norm(&self, x: &[f64]) -> f64 {
        x.iter()
            .map(|xi| xi.abs().powf(self.p))
            .sum::<f64>()
            .powf(1.0 / self.p)
    }

    #[inline]
    fn project_origin(&self, x: &mut [f64]) {
        self.project_lp_ball_general(x);
    }

    fn project_lp_ball_general(&self, x: &mut [f64]) {
        let p = self.p;
        let r = self.radius;
        let tol = self.tolerance;
        let max_iter = self.max_iter;

        let current_norm = self.lp_norm(x);
        if current_norm <= r {
            return;
        }

        let abs_x: Vec<f64> = x.iter().map(|xi| xi.abs()).collect();
        let target = r.powf(p);

        let radius_error = |lambda: f64| -> f64 {
            abs_x
                .iter()
                .map(|&a| {
                    let u = Self::solve_coordinate_newton(a, lambda, p, tol, max_iter);
                    u.powf(p)
                })
                .sum::<f64>()
                - target
        };

        let mut lambda_lo = 0.0_f64;
        let mut lambda_hi = 1.0_f64;

        while radius_error(lambda_hi) > 0.0 {
            lambda_hi *= 2.0;
            if lambda_hi > 1e20 {
                panic!("Failed to bracket the Lagrange multiplier");
            }
        }

        for _ in 0..max_iter {
            let lambda_mid = 0.5 * (lambda_lo + lambda_hi);
            let err = radius_error(lambda_mid);

            if err.abs() <= tol {
                lambda_lo = lambda_mid;
                lambda_hi = lambda_mid;
                break;
            }

            if err > 0.0 {
                lambda_lo = lambda_mid;
            } else {
                lambda_hi = lambda_mid;
            }
        }

        let lambda_star = 0.5 * (lambda_lo + lambda_hi);

        x.iter_mut().zip(abs_x.iter()).for_each(|(xi, &a)| {
            let u = Self::solve_coordinate_newton(a, lambda_star, p, tol, max_iter);
            *xi = xi.signum() * u;
        });
    }

    /// Solve for u >= 0 the equation u + lambda * p * u^(p-1) = a
    /// using safeguarded Newton iterations.
    ///
    /// The solution always belongs to [0, a], so Newton is combined with
    /// bracketing and a bisection fallback.
    fn solve_coordinate_newton(a: f64, lambda: f64, p: f64, tol: f64, max_iter: usize) -> f64 {
        if a == 0.0 {
            return 0.0;
        }

        if lambda == 0.0 {
            return a;
        }

        let mut lo = 0.0_f64;
        let mut hi = a;

        // Heuristic initial guess:
        // exact when p = 2, and usually in the right scale for general p.
        let mut u = (a / (1.0 + lambda * p)).clamp(lo, hi);

        for _ in 0..max_iter {
            let upm1 = u.powf(p - 1.0);
            let f = u + lambda * p * upm1 - a;

            if f.abs() <= tol {
                return u;
            }

            if f > 0.0 {
                hi = u;
            } else {
                lo = u;
            }

            let df = 1.0 + lambda * p * (p - 1.0) * u.powf(p - 2.0);
            let mut candidate = u - f / df;

            if !candidate.is_finite() || candidate <= lo || candidate >= hi {
                candidate = 0.5 * (lo + hi);
            }

            if (candidate - u).abs() <= tol * (1.0 + u.abs()) {
                return candidate;
            }

            u = candidate;
        }

        0.5 * (lo + hi)
    }
}

impl<'a> Constraint for BallP<'a> {
    fn project(&self, x: &mut [f64]) {
        if let Some(center) = &self.center {
            assert_eq!(x.len(), center.len());

            let mut shifted = vec![0.0; x.len()];
            shifted
                .iter_mut()
                .zip(x.iter().zip(center.iter()))
                .for_each(|(s, (xi, ci))| *s = *xi - *ci);

            self.project_origin(&mut shifted);

            x.iter_mut()
                .zip(shifted.iter().zip(center.iter()))
                .for_each(|(xi, (si, ci))| *xi = *ci + *si);
        } else {
            self.project_origin(x);
        }
    }

    fn is_convex(&self) -> bool {
        true
    }
}
