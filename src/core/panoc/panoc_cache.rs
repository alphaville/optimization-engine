use lbfgs::LbfgsPrecision;
use num::Float;
use std::iter::Sum;

fn default_sy_epsilon<T: Float>() -> T {
    T::from(1e-10).expect("1e-10 must be representable")
}

fn default_cbfgs_epsilon<T: Float>() -> T {
    T::from(1e-8).expect("1e-8 must be representable")
}

fn default_cbfgs_alpha<T: Float>() -> T {
    T::one()
}

/// Cache for PANOC
///
/// This struct carries all the information needed at every step of the algorithm.
///
/// An instance of `PANOCCache` needs to be allocated once and a (mutable) reference to it should
/// be passed to instances of [PANOCOPtimizer](struct.PANOCOptimizer.html)
///
/// Subsequently, a `PANOCEngine` is used to construct an instance of `PANOCAlgorithm`
///
#[derive(Debug)]
pub struct PANOCCache<T = f64>
where
    T: Float + LbfgsPrecision + Sum<T>,
{
    pub(crate) lbfgs: lbfgs::Lbfgs<T>,
    pub(crate) gradient_u: Vec<T>,
    /// Stores the gradient of the cost at the previous iteration. This is
    /// an optional field because it is used (and needs to be allocated)
    /// only if we need to check the AKKT-specific termination conditions
    pub(crate) gradient_u_previous: Option<Vec<T>>,
    pub(crate) u_half_step: Vec<T>,
    /// Keeps track of best point so far
    pub(crate) best_u_half_step: Vec<T>,
    pub(crate) gradient_step: Vec<T>,
    pub(crate) direction_lbfgs: Vec<T>,
    pub(crate) u_plus: Vec<T>,
    pub(crate) rhs_ls: T,
    pub(crate) lhs_ls: T,
    pub(crate) gamma_fpr: Vec<T>,
    pub(crate) gamma: T,
    pub(crate) tolerance: T,
    pub(crate) norm_gamma_fpr: T,
    /// Keeps track of best FPR so far
    pub(crate) best_norm_gamma_fpr: T,
    pub(crate) gradient_u_norm_sq: T,
    pub(crate) gradient_step_u_half_step_diff_norm_sq: T,
    pub(crate) tau: T,
    pub(crate) lipschitz_constant: T,
    pub(crate) sigma: T,
    pub(crate) cost_value: T,
    pub(crate) iteration: usize,
    pub(crate) akkt_tolerance: Option<T>,
}

impl<T> PANOCCache<T>
where
    T: Float + LbfgsPrecision + Sum<T>,
{
    /// Construct a new instance of `PANOCCache`
    ///
    /// ## Arguments
    ///
    /// - `problem_size` dimension of the decision variables of the optimization problem
    /// - `tolerance` specified tolerance
    /// - `lbfgs_memory_size` memory of the LBFGS buffer
    ///
    /// ## Panics
    ///
    /// The method will panic if
    ///
    /// - the specified `tolerance` is not positive
    /// - memory allocation fails (memory capacity overflow)
    ///
    /// ## Memory allocation
    ///
    /// This constructor allocated memory using `vec!`.
    ///
    /// It allocates a total of `8*problem_size + 2*lbfgs_memory_size*problem_size + 2*lbfgs_memory_size + 11` floats of type `T`
    ///
    pub fn new(problem_size: usize, tolerance: T, lbfgs_memory_size: usize) -> PANOCCache<T> {
        assert!(tolerance > T::zero(), "tolerance must be positive");

        PANOCCache {
            gradient_u: vec![T::zero(); problem_size],
            gradient_u_previous: None,
            u_half_step: vec![T::zero(); problem_size],
            best_u_half_step: vec![T::zero(); problem_size],
            gamma_fpr: vec![T::zero(); problem_size],
            direction_lbfgs: vec![T::zero(); problem_size],
            gradient_step: vec![T::zero(); problem_size],
            u_plus: vec![T::zero(); problem_size],
            gamma: T::zero(),
            tolerance,
            norm_gamma_fpr: T::infinity(),
            best_norm_gamma_fpr: T::infinity(),
            gradient_u_norm_sq: T::zero(),
            gradient_step_u_half_step_diff_norm_sq: T::zero(),
            lbfgs: lbfgs::Lbfgs::new(problem_size, lbfgs_memory_size)
                .with_cbfgs_alpha(default_cbfgs_alpha())
                .with_cbfgs_epsilon(default_cbfgs_epsilon())
                .with_sy_epsilon(default_sy_epsilon()),
            lhs_ls: T::zero(),
            rhs_ls: T::zero(),
            tau: T::one(),
            lipschitz_constant: T::zero(),
            sigma: T::zero(),
            cost_value: T::zero(),
            iteration: 0,
            akkt_tolerance: None,
        }
    }

    /// Sets the AKKT-specific tolerance and activates the corresponding
    /// termination criterion
    ///
    /// ## Arguments
    ///
    /// - `akkt_tolerance`: Tolerance for the AKKT-specific termination condition
    ///
    /// ## Panics
    ///
    /// The method panics if `akkt_tolerance` is nonpositive
    ///
    pub fn set_akkt_tolerance(&mut self, akkt_tolerance: T) {
        assert!(
            akkt_tolerance > T::zero(),
            "akkt_tolerance must be positive"
        );
        self.akkt_tolerance = Some(akkt_tolerance);
        self.gradient_u_previous = Some(vec![T::zero(); self.gradient_step.len()]);
    }

    /// Copies the value of the current cost gradient to `gradient_u_previous`,
    /// which stores the previous gradient vector
    ///
    pub fn cache_previous_gradient(&mut self) {
        if self.iteration >= 1 {
            if let Some(df_previous) = &mut self.gradient_u_previous {
                df_previous.copy_from_slice(&self.gradient_u);
            }
        }
    }

    /// Computes the AKKT residual which is defined as `||gamma*(fpr + df - df_previous)||`
    fn akkt_residual(&self) -> T {
        let mut r = T::zero();
        if let Some(df_previous) = &self.gradient_u_previous {
            // Notation: gamma_fpr_i is the i-th element of gamma_fpr = gamma * fpr,
            // df_i is the i-th element of the gradient of the cost function at the
            // updated iterate (x+) and dfp_i is the i-th element of the gradient at the
            // current iterate (x)
            r = self
                .gamma_fpr
                .iter()
                .zip(self.gradient_u.iter())
                .zip(df_previous.iter())
                .fold(T::zero(), |sum, ((&gamma_fpr_i, &df_i), &dfp_i)| {
                    sum + (gamma_fpr_i + self.gamma * (df_i - dfp_i)).powi(2)
                })
                .sqrt();
        }
        r
    }

    /// Returns true iff the norm of gamma*FPR is below the desired tolerance
    fn fpr_exit_condition(&self) -> bool {
        self.norm_gamma_fpr < self.tolerance
    }

    /// Checks whether the AKKT-specific termination condition is satisfied
    fn akkt_exit_condition(&self) -> bool {
        let mut exit_condition = true;
        if let Some(akkt_tol) = self.akkt_tolerance {
            let res = self.akkt_residual();
            exit_condition = res < akkt_tol;
        }
        exit_condition
    }

    /// Returns `true` iff all termination conditions are satisfied
    ///
    /// It checks whether:
    ///  - the FPR condition, `gamma*||fpr|| < epsilon` ,
    ///  - (if activated) the AKKT condition `||gamma*fpr + (df - df_prev)|| < eps_akkt`
    ///    are satisfied.
    pub fn exit_condition(&self) -> bool {
        self.fpr_exit_condition() && self.akkt_exit_condition()
    }

    /// Resets the cache to its initial virgin state.
    ///
    /// In particular,
    ///
    /// - Resets/empties the LBFGS buffer
    /// - Sets tau = 1.0
    /// - Sets the iteration count to 0
    /// - Sets the internal variables `lhs_ls`, `rhs_ls`,
    ///   `lipschitz_constant`, `sigma`, `cost_value`
    ///   and `gamma` to 0.0
    pub fn reset(&mut self) {
        self.lbfgs.reset();
        self.best_u_half_step.fill(T::zero());
        self.best_norm_gamma_fpr = T::infinity();
        self.norm_gamma_fpr = T::infinity();
        self.gradient_u_norm_sq = T::zero();
        self.gradient_step_u_half_step_diff_norm_sq = T::zero();
        self.lhs_ls = T::zero();
        self.rhs_ls = T::zero();
        self.tau = T::one();
        self.lipschitz_constant = T::zero();
        self.sigma = T::zero();
        self.cost_value = T::zero();
        self.iteration = 0;
        self.gamma = T::zero();
    }

    /// Store the current half step if it improves the best fixed-point residual so far.
    pub(crate) fn cache_best_half_step(&mut self) {
        if self.norm_gamma_fpr < self.best_norm_gamma_fpr {
            self.best_norm_gamma_fpr = self.norm_gamma_fpr;
            self.best_u_half_step.copy_from_slice(&self.u_half_step);
        }
    }

    /// Sets the CBFGS parameters `alpha` and `epsilon`
    ///
    /// Read more in: D.-H. Li and M. Fukushima, “On the global convergence of the BFGS
    /// method for nonconvex unconstrained optimization problems,” vol. 11,
    /// no. 4, pp. 1054–1064, jan 2001.
    ///
    /// ## Arguments
    ///
    /// - alpha
    /// - epsilon
    /// - sy_epsilon
    ///
    /// ## Panics
    ///
    /// The method panics if alpha or epsilon are nonpositive and if sy_epsilon
    /// is negative.
    ///
    #[must_use]
    pub fn with_cbfgs_parameters(mut self, alpha: T, epsilon: T, sy_epsilon: T) -> Self {
        self.lbfgs = self
            .lbfgs
            .with_cbfgs_alpha(alpha)
            .with_cbfgs_epsilon(epsilon)
            .with_sy_epsilon(sy_epsilon);
        self
    }
}

#[cfg(test)]
mod tests {
    use super::PANOCCache;

    #[test]
    fn t_cache_best_half_step() {
        let mut cache = PANOCCache::new(2, 1e-6, 3);

        cache.u_half_step.copy_from_slice(&[1.0, 2.0]);
        cache.norm_gamma_fpr = 3.0;
        cache.cache_best_half_step();

        assert_eq!(3.0, cache.best_norm_gamma_fpr);
        assert_eq!(&[1.0, 2.0], &cache.best_u_half_step[..]);

        cache.u_half_step.copy_from_slice(&[10.0, 20.0]);
        cache.norm_gamma_fpr = 5.0;
        cache.cache_best_half_step();

        assert_eq!(3.0, cache.best_norm_gamma_fpr);
        assert_eq!(&[1.0, 2.0], &cache.best_u_half_step[..]);

        cache.u_half_step.copy_from_slice(&[-1.0, -2.0]);
        cache.norm_gamma_fpr = 2.0;
        cache.cache_best_half_step();

        assert_eq!(2.0, cache.best_norm_gamma_fpr);
        assert_eq!(&[-1.0, -2.0], &cache.best_u_half_step[..]);
    }

    #[test]
    fn t_cache_best_half_step_f32() {
        let mut cache = PANOCCache::<f32>::new(2, 1e-6_f32, 3);

        cache.u_half_step.copy_from_slice(&[1.0_f32, 2.0]);
        cache.norm_gamma_fpr = 3.0_f32;
        cache.cache_best_half_step();

        assert_eq!(3.0_f32, cache.best_norm_gamma_fpr);
        assert_eq!(&[1.0_f32, 2.0], &cache.best_u_half_step[..]);

        cache.reset();
        assert!(cache.best_norm_gamma_fpr.is_infinite());
        assert!(cache.norm_gamma_fpr.is_infinite());
        assert_eq!(0.0_f32, cache.gamma);
        assert_eq!(1.0_f32, cache.tau);
    }
}
