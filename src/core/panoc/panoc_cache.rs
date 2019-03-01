use super::PANOCCache;
use std::num::NonZeroUsize;

impl PANOCCache {
    /// Construct a new instance of `PANOCCache`
    ///
    /// ## Arguments
    ///
    /// - `n` dimension of the decision variables of the optimization problem
    /// - `tolerance` specified tolerance
    /// - `lbfgs_mem` memory of the LBFGS buffer
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
    /// It allocates a total of `8*n + 2*lbfgs_mem*n + 2*lbfgs_mem + 11` floats (`f64`)
    ///
    pub fn new(n: NonZeroUsize, tolerance: f64, lbfgs_mem: NonZeroUsize) -> PANOCCache {
        assert!(tolerance > 0., "tolerance must be positive");
        PANOCCache {
            gradient_u: vec![0.0; n.get()],
            u_half_step: vec![0.0; n.get()],
            gamma_fpr: vec![0.0; n.get()],
            direction_lbfgs: vec![0.0; n.get()],
            gradient_step: vec![0.0; n.get()],
            u_plus: vec![0.0; n.get()],
            gamma: 0.0,
            tolerance: tolerance,
            norm_gamma_fpr: std::f64::INFINITY,
            // TODO: change the following lines...
            lbfgs: lbfgs::Lbfgs::new(n, lbfgs_mem)
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
