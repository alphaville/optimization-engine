use crate::numeric::cast;

use super::Constraint;
use num::Float;

#[derive(Copy, Clone)]
/// A simplex with level $\alpha$ is a set of the form
/// $\Delta_\alpha^n = \\{x \in \mathbb{R}^n {}:{} x \geq 0, \sum_i x_i = \alpha\\}$,
/// where $\alpha$ is a positive constant.
pub struct Simplex<T = f64> {
    /// Simplex level
    alpha: T,
}

impl<T: Float> Simplex<T> {
    /// Construct a new simplex with given (positive) $\alpha$. The user does not need
    /// to specify the dimension of the simplex.
    pub fn new(alpha: T) -> Self {
        assert!(alpha > T::zero(), "alpha is nonpositive");
        Simplex { alpha }
    }
}

impl<T: Float> Constraint<T> for Simplex<T> {
    /// Project onto $\Delta_\alpha^n$ using Condat's fast projection algorithm.
    ///
    /// See: Laurent Condat. Fast Projection onto the Simplex and the $\ell_1$ Ball.
    /// <em>Mathematical Programming, Series A,</em> Springer, 2016, 158 (1), pp.575-585.
    /// ⟨<a href="https://dx.doi.org/10.1007/s10107-015-0946-6">10.1007/s10107-015-0946-6</a>⟩.
    fn project(&self, x: &mut [T]) {
        assert!(!x.is_empty(), "x must be nonempty");
        let a = &self.alpha;

        // ---- step 1
        let mut v = Vec::<T>::with_capacity(x.len()); // vector containing x[0]
        v.push(x[0]);
        let mut v_size_old: i64 = -1; // 64 bit signed int
        let mut v_tilde: Vec<T> = Vec::new();
        let mut rho: T = x[0] - *a;

        // ---- step 2
        x.iter().skip(1).for_each(|x_n| {
            if *x_n > rho {
                rho = rho + (*x_n - rho) / cast(v.len() + 1);
                if rho > *x_n - *a {
                    v.push(*x_n);
                } else {
                    v_tilde.extend(&v);
                    v = vec![*x_n];
                    rho = *x_n - *a;
                }
            }
        });

        // ---- step 3
        if !v_tilde.is_empty() {
            v_tilde.iter().for_each(|v_t_n| {
                if *v_t_n > rho {
                    v.push(*v_t_n);
                    rho = rho + (*v_t_n - rho) / cast(v.len());
                }
            });
        }

        // ---- step 4
        let mut keep_running = true;
        while keep_running {
            let mut hit_list: Vec<usize> = Vec::with_capacity(x.len());
            let mut current_len_v = v.len() as i64;
            v.iter().enumerate().for_each(|(n, v_n)| {
                if *v_n <= rho {
                    hit_list.push(n);
                    current_len_v -= 1;
                    rho = rho + (rho - *v_n) / cast(current_len_v);
                }
            });
            hit_list.iter().rev().for_each(|target| {
                // remove in reverse to keep indexing correct
                v.remove(*target);
            });
            keep_running = current_len_v != v_size_old;
            v_size_old = current_len_v;
        }

        // ---- step 6
        let zero = T::zero();
        x.iter_mut().for_each(|x_n| *x_n = zero.max(*x_n - rho));
    }

    fn is_convex(&self) -> bool {
        true
    }
}
