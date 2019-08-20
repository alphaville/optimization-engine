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
            // TODO: change the following lines (issue #5)...
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
            akkt_tolerance: None,
        }
    }

    ///
    pub fn set_akkt_tolerance(&mut self, akkt_tolerance: f64) {
        self.akkt_tolerance = Some(akkt_tolerance);
        self.gradient_u_previous = Some(vec![0.0; self.gradient_step.len()]);
    }

    pub fn swap_cached_gradients(&mut self) {
        if self.iteration >= 1 {
            if let Some(df_previous) = &mut self.gradient_u_previous {
                df_previous.copy_from_slice(&self.gradient_u);
            }
        }
    }

    pub fn akkt_residual(&self) -> f64 {
        let mut r = 0.0;
        if let Some(df_previous) = &self.gradient_u_previous {
            r = self
                .gamma_fpr
                .iter()
                .zip(self.gradient_u.iter())
                .zip(df_previous.iter())
                .fold(0.0, |mut sum, ((&fpr, &df), &dfp)| {
                    sum += (fpr + df - dfp).powi(2);
                    sum
                })
                .sqrt();
        }
        r
    }

    fn fpr_exit_condition(&self) -> bool {
        self.norm_gamma_fpr < self.tolerance
    }

    fn akkt_exit_condition(&self) -> bool {
        let mut exit_condition = true;
        if let Some(akkt_tol) = self.akkt_tolerance {
            let res = self.akkt_residual();
            exit_condition = res < akkt_tol;
        }
        exit_condition
    }

    pub fn exit_condition(&self) -> bool {
        self.fpr_exit_condition() && self.akkt_exit_condition()
    }

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
}
