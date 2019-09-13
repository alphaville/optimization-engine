const DEFAULT_SY_EPSILON: f64 = 1e-10;
const DEFAULT_CBFGS_EPSILON: f64 = 1e-8;
const DEFAULT_CBFGS_ALPHA: f64 = 1.0;

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
pub struct PANOCCache {
    pub(crate) lbfgs: lbfgs::Lbfgs,
    pub(crate) gradient_u: Vec<f64>,
    /// Stores the gradient of the cost at the previous iteration. This is
    /// an optional field because it is used (and needs to be allocated)
    /// only if we need to check the AKKT-specific termination conditions
    pub(crate) gradient_u_previous: Option<Vec<f64>>,
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
    pub(crate) akkt_tolerance: Option<f64>,
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
    pub fn new(problem_size: usize, tolerance: f64, lbfgs_memory_size: usize) -> PANOCCache {
        assert!(tolerance > 0., "tolerance must be positive");

        PANOCCache {
            gradient_u: vec![0.0; problem_size],
            gradient_u_previous: None,
            u_half_step: vec![0.0; problem_size],
            gamma_fpr: vec![0.0; problem_size],
            direction_lbfgs: vec![0.0; problem_size],
            gradient_step: vec![0.0; problem_size],
            u_plus: vec![0.0; problem_size],
            gamma: 0.0,
            tolerance,
            norm_gamma_fpr: std::f64::INFINITY,
            lbfgs: lbfgs::Lbfgs::new(problem_size, lbfgs_memory_size)
                .with_cbfgs_alpha(DEFAULT_CBFGS_ALPHA)
                .with_cbfgs_epsilon(DEFAULT_CBFGS_EPSILON)
                .with_sy_epsilon(DEFAULT_SY_EPSILON),
            lhs_ls: 0.0,
            rhs_ls: 0.0,
            tau: 1.0,
            lipschitz_constant: 0.0,
            sigma: 0.0,
            cost_value: 0.0,
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
    pub fn set_akkt_tolerance(&mut self, akkt_tolerance: f64) {
        assert!(akkt_tolerance > 0.0, "akkt_tolerance must be positive");
        self.akkt_tolerance = Some(akkt_tolerance);
        self.gradient_u_previous = Some(vec![0.0; self.gradient_step.len()]);
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
    fn akkt_residual(&self) -> f64 {
        let mut r = 0.0;
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
                .fold(0.0, |mut sum, ((&gamma_fpr_i, &df_i), &dfp_i)| {
                    sum += (gamma_fpr_i + self.gamma * (df_i - dfp_i)).powi(2);
                    sum
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
    /// are satisfied.
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
        self.lhs_ls = 0.0;
        self.rhs_ls = 0.0;
        self.tau = 1.0;
        self.lipschitz_constant = 0.0;
        self.sigma = 0.0;
        self.cost_value = 0.0;
        self.iteration = 0;
        self.gamma = 0.0;
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
    pub fn with_cbfgs_parameters(mut self, alpha: f64, epsilon: f64, sy_epsilon: f64) -> Self {
        self.lbfgs = self
            .lbfgs
            .with_cbfgs_alpha(alpha)
            .with_cbfgs_epsilon(epsilon)
            .with_sy_epsilon(sy_epsilon);
        self
    }
}
