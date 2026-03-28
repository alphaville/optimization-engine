use crate::numeric::cast;

use super::Constraint;
use crate::{FunctionCallResult, SolverError};
use num::Float;

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
/// ball.project(&mut x).unwrap();
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
/// ball.project(&mut x).unwrap();
/// ```
///
/// # Notes
///
/// - The projection is with respect to the *Euclidean norm*
/// - The implementation is intended for general finite $p > 1.0$. If you need
///   to project on a $\Vert{}\cdot{}\Vert_1$-ball or an $\Vert{}\cdot{}\Vert_\infty$-ball,
///   use the implementations in [`Ball1`](crate::constraints::Ball1)
///   and [`BallInf`](crate::constraints::BallInf).
/// - Do not use this struct to project on a Euclidean ball; the implementation
///   in [`Ball2`](crate::constraints::Ball2) is more efficient
/// - The quality and speed of the computation depend on the chosen numerical
///   tolerance and iteration limit.
pub struct BallP<'a, T = f64> {
    /// Optional center of the ball.
    ///
    /// If `None`, the ball is centered at the origin.
    /// If `Some(center)`, the ball is centered at `center`.
    center: Option<&'a [T]>,

    /// Radius of the ball.
    ///
    /// Must be strictly positive.
    radius: T,

    /// Exponent of the norm.
    ///
    /// Must satisfy `p > 1.0` and be finite.
    p: T,

    /// Numerical tolerance used by the outer bisection on the Lagrange
    /// multiplier and by the inner Newton solver.
    tolerance: T,

    /// Maximum number of iterations used by the outer bisection and
    /// the inner Newton solver.
    max_iter: usize,
}

impl<'a, T: Float> BallP<'a, T> {
    /// Construct a new l_p ball with given center, radius, and exponent.
    ///
    /// - `center`: if `None`, the ball is centered at the origin
    /// - `radius`: radius of the ball
    /// - `p`: norm exponent, must satisfy `p > 1.0` and be finite
    /// - `tolerance`: tolerance for the numerical solvers
    /// - `max_iter`: maximum number of iterations for the numerical solvers
    pub fn new(center: Option<&'a [T]>, radius: T, p: T, tolerance: T, max_iter: usize) -> Self {
        assert!(radius > T::zero());
        assert!(p > T::one() && p.is_finite());
        assert!(tolerance > T::zero());
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
    fn lp_norm(&self, x: &[T]) -> T {
        x.iter()
            .map(|xi| xi.abs().powf(self.p))
            .fold(T::zero(), |sum, xi| sum + xi)
            .powf(T::one() / self.p)
    }

    fn project_lp_ball(&self, x: &mut [T]) -> FunctionCallResult {
        let p = self.p;
        let r = self.radius;
        let tol = self.tolerance;
        let max_iter = self.max_iter;

        let current_norm = self.lp_norm(x);
        if current_norm <= r {
            return Ok(());
        }

        let abs_x: Vec<T> = x.iter().map(|xi| xi.abs()).collect();
        let target = r.powf(p);

        let radius_error = |lambda: T| -> T {
            abs_x
                .iter()
                .map(|&a| {
                    let u = Self::solve_coordinate_newton(a, lambda, p, tol, max_iter);
                    u.powf(p)
                })
                .fold(T::zero(), |sum, ui| sum + ui)
                - target
        };

        let mut lambda_lo = T::zero();
        let mut lambda_hi = T::one();

        while radius_error(lambda_hi) > T::zero() {
            lambda_hi = lambda_hi * cast::<T>(2.0);
            if lambda_hi > cast::<T>(1e20) {
                return Err(SolverError::ProjectionFailed(
                    "failed to bracket the Lagrange multiplier",
                ));
            }
        }

        for _ in 0..max_iter {
            let lambda_mid = cast::<T>(0.5) * (lambda_lo + lambda_hi);
            let err = radius_error(lambda_mid);

            if err.abs() <= tol {
                lambda_lo = lambda_mid;
                lambda_hi = lambda_mid;
                break;
            }

            if err > T::zero() {
                lambda_lo = lambda_mid;
            } else {
                lambda_hi = lambda_mid;
            }
        }

        let lambda_star = cast::<T>(0.5) * (lambda_lo + lambda_hi);

        x.iter_mut().zip(abs_x.iter()).for_each(|(xi, &a)| {
            let u = Self::solve_coordinate_newton(a, lambda_star, p, tol, max_iter);
            *xi = xi.signum() * u;
        });
        Ok(())
    }

    /// Solve for u >= 0 the equation u + lambda * p * u^(p-1) = a
    /// using safeguarded Newton iterations.
    ///
    /// The solution always belongs to [0, a], so Newton is combined with
    /// bracketing and a bisection fallback.
    fn solve_coordinate_newton(a: T, lambda: T, p: T, tol: T, max_iter: usize) -> T {
        if a == T::zero() {
            return T::zero();
        }

        if lambda == T::zero() {
            return a;
        }

        let mut lo = T::zero();
        let mut hi = a;

        // Heuristic initial guess:
        // exact when p = 2, and usually in the right scale for general p.
        let mut u = (a / (T::one() + lambda * p)).clamp(lo, hi);

        for _ in 0..max_iter {
            let upm1 = u.powf(p - T::one());
            let f = u + lambda * p * upm1 - a;

            if f.abs() <= tol {
                return u;
            }

            if f > T::zero() {
                hi = u;
            } else {
                lo = u;
            }

            let df = T::one() + lambda * p * (p - T::one()) * u.powf(p - cast::<T>(2.0));
            let mut candidate = u - f / df;

            if !candidate.is_finite() || candidate <= lo || candidate >= hi {
                candidate = cast::<T>(0.5) * (lo + hi);
            }

            if (candidate - u).abs() <= tol * (T::one() + u.abs()) {
                return candidate;
            }

            u = candidate;
        }

        cast::<T>(0.5) * (lo + hi)
    }
}

impl<'a, T: Float> Constraint<T> for BallP<'a, T> {
    fn project(&self, x: &mut [T]) -> FunctionCallResult {
        if let Some(center) = &self.center {
            assert_eq!(
                x.len(),
                center.len(),
                "x and xc have incompatible dimensions"
            );

            let mut shifted = vec![T::zero(); x.len()];
            shifted
                .iter_mut()
                .zip(x.iter().zip(center.iter()))
                .for_each(|(s, (xi, ci))| *s = *xi - *ci);

            self.project_lp_ball(&mut shifted)?;

            x.iter_mut()
                .zip(shifted.iter().zip(center.iter()))
                .for_each(|(xi, (si, ci))| *xi = *ci + *si);
        } else {
            self.project_lp_ball(x)?;
        }
        Ok(())
    }

    fn is_convex(&self) -> bool {
        true
    }
}
