use std::num::NonZeroUsize;

/// Cache for PANOC
///
/// This struct carries all the information needed at every step of the algorithm.
///
/// An instance of `PANOCCache` needs to be allocated once and a (mutable) reference to it should
/// be passed to instances of [PANOCEngine](struct.PANOCEngine.html)
///
/// Subsequently, a `PANOCEngine` is used to construct an instance of `PANOCAlgorithm`
///
#[derive(Debug)]
pub struct PANOCCache {
    pub(crate) lbfgs: lbfgs::Lbfgs,
    pub(crate) gradient_u: Vec<f64>,
    pub(crate) u_half_step: Vec<f64>,
    pub(crate) gradient_step: Vec<f64>,
    pub(crate) direction_lbfgs: Vec<f64>,
    pub(crate) u_plus: Vec<f64>,
    pub(crate) rhs_ls: f64,
    pub(crate) lhs_ls: f64,
    pub(crate) gamma_fpr: Vec<f64>,
    pub(crate) gamma: f64,
    pub(crate) tolerance: f64,
    pub(crate) norm_gamma_fpr: f64,
    pub(crate) tau: f64,
    pub(crate) lipschitz_constant: f64,
    pub(crate) sigma: f64,
    pub(crate) cost_value: f64,
    pub(crate) iteration: usize,
}

impl PANOCCache {
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
    /// It allocates a total of `8*problem_size + 2*lbfgs_memory_size*problem_size + 2*lbfgs_memory_size + 11` floats (`f64`)
    ///
    pub fn new(
        problem_size: NonZeroUsize,
        tolerance: f64,
        lbfgs_memory_size: NonZeroUsize,
    ) -> PANOCCache {
        assert!(tolerance > 0., "tolerance must be positive");

        PANOCCache {
            gradient_u: vec![0.0; problem_size.get()],
            u_half_step: vec![0.0; problem_size.get()],
            gamma_fpr: vec![0.0; problem_size.get()],
            direction_lbfgs: vec![0.0; problem_size.get()],
            gradient_step: vec![0.0; problem_size.get()],
            u_plus: vec![0.0; problem_size.get()],
            gamma: 0.0,
            tolerance,
            norm_gamma_fpr: std::f64::INFINITY,
            // TODO: change the following lines...
            lbfgs: lbfgs::Lbfgs::new(problem_size, lbfgs_memory_size)
                .with_cbfgs_alpha(1.0)
                .with_cbfgs_epsilon(1e-8)
                .with_sy_epsilon(1e-10),
            lhs_ls: 0.0,
            rhs_ls: 0.0,
            tau: 1.0,
            lipschitz_constant: 0.0,
            sigma: 0.0,
            cost_value: 0.0,
            iteration: 0,
        }
    }
}
