use super::PANOCCache;

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
    pub fn new(n: usize, tolerance: f64, lbfgs_mem: usize) -> PANOCCache {
        assert!(tolerance > 0., "tolerance must be positive");
        PANOCCache {
            gradient_u: vec![0.0; n],
            u_half_step: vec![0.0; n],
            fixed_point_residual: vec![0.0; n],
            direction_lbfgs: vec![0.0; n],
            gradient_step: vec![0.0; n],
            u_plus: vec![0.0; n],
            gamma: 0.0,
            tolerance: tolerance,
            norm_fpr: std::f64::INFINITY,
            lbfgs: lbfgs::Estimator::new(n, lbfgs_mem),
            lhs_ls: 0.0,
            rhs_ls: 0.0,
            tau: 1.0,
            lipschitz_constant: 0.0,
            sigma: 0.0,
            cost_value: 0.0,
        }
    }
}
