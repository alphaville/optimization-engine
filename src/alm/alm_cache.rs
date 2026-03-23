use crate::panoc::PANOCCache;
use lbfgs::LbfgsPrecision;
use num::Float;
use std::iter::Sum;

fn default_initial_penalty<T: Float>() -> T {
    T::from(10.0).expect("10.0 must be representable")
}

/// Cache for `AlmOptimizer` (to be allocated once)
///
/// This is a cache structure that contains all the data that make
/// up the "state" of the ALM/PM algorithm, i.e., all those data that
/// the algorithm *updates*.
///
/// On the other hand, the problem data are provided in an instance
/// of `AlmProblem`
///
#[derive(Debug)]
pub struct AlmCache<T = f64>
where
    T: Float + LbfgsPrecision + Sum<T>,
{
    /// PANOC cache for inner problems
    pub(crate) panoc_cache: PANOCCache<T>,
    /// Lagrange multipliers (next)
    pub(crate) y_plus: Option<Vec<T>>,
    /// Vector $\xi^\nu = (c^\nu, y^\nu)$
    pub(crate) xi: Option<Vec<T>>,
    /// Infeasibility related to ALM-type constraints
    pub(crate) delta_y_norm: T,
    /// Delta y at iteration `nu+1`
    pub(crate) delta_y_norm_plus: T,
    /// Value $\Vert F_2(u^\nu) \Vert$
    pub(crate) f2_norm: T,
    /// Value $\Vert F_2(u^{\nu+1}) \Vert$
    pub(crate) f2_norm_plus: T,
    /// Auxiliary variable `w`
    pub(crate) w_alm_aux: Option<Vec<T>>,
    /// Infeasibility related to PM-type constraints, `w_pm = F2(u)`
    pub(crate) w_pm: Option<Vec<T>>,
    /// (Outer) iteration count
    pub(crate) iteration: usize,
    /// Counter for inner iterations
    pub(crate) inner_iteration_count: usize,
    /// Value of the norm of the fixed-point residual for the last
    /// solved inner problem
    pub(crate) last_inner_problem_norm_fpr: T,
    /// Available time left for ALM/PM computations (the value `None`
    /// corresponds to an unspecified available time, i.e., there are
    /// no bounds on the maximum time). The maximum time is specified,
    /// if at all, in `AlmOptimizer`
    pub(crate) available_time: Option<std::time::Duration>,
}

impl<T> AlmCache<T>
where
    T: Float + LbfgsPrecision + Sum<T>,
{
    /// Construct a new instance of `AlmCache`
    ///
    /// # Arguments
    ///
    /// - `panoc_cache`: an instance of `PANOCCache` that will be used by
    ///   the inner problem
    /// - `n1`, `n2`: range dimensions of mappings `F1` and `F2` respectively
    ///
    /// # Panics
    ///
    /// Does not panic
    ///
    pub fn new(panoc_cache: PANOCCache<T>, n1: usize, n2: usize) -> Self {
        AlmCache {
            panoc_cache,
            y_plus: if n1 > 0 {
                Some(vec![T::zero(); n1])
            } else {
                None
            },
            // Allocate memory for xi = (c, y) if either n1 or n2 is nonzero,
            // otherwise, xi is None
            xi: if n1 + n2 > 0 {
                let mut xi_init = vec![default_initial_penalty(); 1];
                xi_init.append(&mut vec![T::zero(); n1]);
                Some(xi_init)
            } else {
                None
            },
            // w_alm_aux should be allocated only if n1 > 0
            w_alm_aux: if n1 > 0 {
                Some(vec![T::zero(); n1])
            } else {
                None
            },
            // w_pm is needed only if n2 > 0
            w_pm: if n2 > 0 {
                Some(vec![T::zero(); n2])
            } else {
                None
            },
            iteration: 0,
            delta_y_norm: T::zero(),
            delta_y_norm_plus: T::infinity(),
            f2_norm: T::zero(),
            f2_norm_plus: T::infinity(),
            inner_iteration_count: 0,
            last_inner_problem_norm_fpr: -T::one(),
            available_time: None,
        }
    }

    /// Resets the cache to its virgin state, and resets the stored instance
    /// of `PANOCCache`
    ///
    pub fn reset(&mut self) {
        self.panoc_cache.reset();
        self.iteration = 0;
        self.f2_norm = T::zero();
        self.f2_norm_plus = T::zero();
        self.delta_y_norm = T::zero();
        self.delta_y_norm_plus = T::zero();
        self.inner_iteration_count = 0;
    }
}
