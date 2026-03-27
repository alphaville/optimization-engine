use crate::{numeric::cast, panoc::PANOCCache};
use lbfgs::LbfgsPrecision;
use num::Float;
use std::iter::Sum;

fn default_initial_penalty<T: Float>() -> T {
    cast::<T>(10.0)
}

/// Cache and mutable state for `AlmOptimizer`
///
/// `AlmCache` stores the data that the outer ALM/PM loop updates from one
/// iteration to the next, together with the [`PANOCCache`] used to solve the
/// inner problems.
///
/// The problem definition itself is stored separately in `AlmProblem`.
///
/// The scalar type `T` is generic and is typically `f64` or `f32`. The default
/// is `f64`.
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
    /// Constructs a new `AlmCache`
    ///
    /// # Arguments
    ///
    /// - `panoc_cache`: cache used by the inner PANOC solver
    /// - `n1`: dimension of the ALM mapping `F1`
    /// - `n2`: dimension of the PM mapping `F2`
    ///
    /// The scalar type `T` is inferred from `panoc_cache`. Depending on the
    /// values of `n1` and `n2`, this constructor allocates the auxiliary
    /// vectors needed by the ALM and PM updates:
    ///
    /// - `y_plus` and `w_alm_aux` are allocated only when `n1 > 0`
    /// - `w_pm` is allocated only when `n2 > 0`
    /// - `xi` is allocated when `n1 + n2 > 0` and is initialized as
    ///   `xi = (c^0, y^0)`, where `c^0` is the default initial penalty and
    ///   `y^0` is the zero vector in `R^{n1}`
    ///
    /// # Examples
    ///
    /// Using the default scalar type (`f64`):
    ///
    /// ```
    /// use optimization_engine::{alm::AlmCache, panoc::PANOCCache};
    ///
    /// let panoc_cache = PANOCCache::new(4, 1e-6, 8);
    /// let _alm_cache = AlmCache::new(panoc_cache, 2, 1);
    /// ```
    ///
    /// Using `f32` explicitly:
    ///
    /// ```
    /// use optimization_engine::{alm::AlmCache, panoc::PANOCCache};
    ///
    /// let panoc_cache = PANOCCache::new(4, 1e-5_f32, 8);
    /// let _alm_cache = AlmCache::<f32>::new(panoc_cache, 2, 1);
    /// ```
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

    /// Resets the cache to its initial iteration state
    ///
    /// This method:
    ///
    /// - resets the stored [`PANOCCache`]
    /// - clears the outer iteration counters
    /// - resets the stored infeasibility and fixed-point-residual related norms
    ///
    /// The allocated work vectors remain allocated so the cache can be reused
    /// without additional memory allocations.
    ///
    /// # Examples
    ///
    /// ```
    /// use optimization_engine::{alm::AlmCache, panoc::PANOCCache};
    ///
    /// let panoc_cache = PANOCCache::new(3, 1e-6, 5);
    /// let mut alm_cache = AlmCache::new(panoc_cache, 1, 1);
    ///
    /// alm_cache.reset();
    /// ```
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
