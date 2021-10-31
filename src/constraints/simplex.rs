use super::Constraint;

#[derive(Copy, Clone)]
/// A simplex with level $\alpha$ is a set of the form
/// $\Delta_\alpha^n = \\{x \in \mathbb{R}^n {}:{} x \geq 0, \sum_i x_i = \alpha\\}$,
/// where $\alpha$ is a positive constant.
pub struct Simplex {
    /// Simplex level
    alpha: f64,
}

impl Simplex {
    /// Construct a new simplex with given (positive) $\alpha$. The user does not need
    /// to specify the dimension of the simplex.
    pub fn new(alpha: f64) -> Self {
        assert!(alpha > 0.0, "alpha is nonpositive");
        Simplex { alpha }
    }
}

impl Constraint for Simplex {
    /// Project onto $\Delta_\alpha^n$ using Condat's fast projection algorithm.
    ///
    /// See: Laurent Condat. Fast Projection onto the Simplex and the $\ell_1$ Ball.
    /// <em>Mathematical Programming, Series A,</em> Springer, 2016, 158 (1), pp.575-585.
    /// ⟨<a href="https://dx.doi.org/10.1007/s10107-015-0946-6">10.1007/s10107-015-0946-6</a>⟩.
    fn project(&self, x: &mut [f64]) {
        let a = &self.alpha;

        // ---- step 1
        let mut v = Vec::<f64>::with_capacity(x.len()); // vector containing x[0]
        v.push(x[0]);
        let mut v_size_old: i64 = -1; // 64 bit signed int
        let mut v_tilde: Vec<f64> = Vec::new(); // empty vector of f64
        let mut rho: f64 = x[0] - a; // 64 bit float

        // ---- step 2
        x.iter().skip(1).for_each(|x_n| {
            if *x_n > rho {
                rho += (*x_n - rho) / ((v.len() + 1) as f64);
                if rho > *x_n - a {
                    v.push(*x_n);
                } else {
                    v_tilde.extend(&v);
                    v = vec![*x_n];
                    rho = *x_n - a;
                }
            }
        });

        // ---- step 3
        if !v_tilde.is_empty() {
            v_tilde.iter().for_each(|v_t_n| {
                if *v_t_n > rho {
                    v.push(*v_t_n);
                    rho += (*v_t_n - rho) / (v.len() as f64);
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
                    rho += (rho - *v_n) / (current_len_v as f64);
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
        let zero: f64 = 0.0;
        x.iter_mut().for_each(|x_n| *x_n = zero.max(*x_n - rho));
    }

    fn is_convex(&self) -> bool {
        true
    }
}
