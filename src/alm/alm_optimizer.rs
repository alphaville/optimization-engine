use crate::{
    alm::*,
    constraints,
    core::{panoc::PANOCOptimizer, ExitStatus, Optimizer, Problem, SolverStatus},
    matrix_operations, FunctionCallResult, SolverError,
};

const DEFAULT_MAX_OUTER_ITERATIONS: usize = 50;
const DEFAULT_MAX_INNER_ITERATIONS: usize = 5000;
const DEFAULT_EPSILON_TOLERANCE: f64 = 1e-6;
const DEFAULT_DELTA_TOLERANCE: f64 = 1e-4;
const DEFAULT_PENALTY_UPDATE_FACTOR: f64 = 5.0;
const DEFAULT_EPSILON_UPDATE_FACTOR: f64 = 0.1;
const DEFAULT_INFEAS_SUFFICIENT_DECREASE_FACTOR: f64 = 0.1;
const DEFAULT_INITIAL_TOLERANCE: f64 = 0.1;
const SMALL_EPSILON: f64 = std::f64::EPSILON;

/// Internal/private structure used by method AlmOptimizer.step
/// to return some minimal information about the inner problem
struct InnerProblemStatus {
    /// whether the outer solver should continue iterating
    /// This is `true` if and only if the an (epsilon,delta)-AKKT
    /// point has not been found so far
    outer_continue_iterating: bool,
    /// status of the inner optimization problem
    inner_problem_exit_status: ExitStatus,
}

impl InnerProblemStatus {
    fn new(outer_continue_iterating: bool, inner_problem_exit_status: ExitStatus) -> Self {
        InnerProblemStatus {
            outer_continue_iterating,
            inner_problem_exit_status,
        }
    }
}

/// Implements the ALM/PM method
///
/// `AlmOptimizer` solves the problem
///
///
/// $$
/// \begin{aligned}
/// \mathrm{Minimize}\  f(u)
/// \\\\
/// u \in U
/// \\\\
/// F_1(u) \in C
/// \\\\
/// F_2(u) = 0
/// \end{aligned}$$
///
/// where $u\in\mathbb{R}^{n_u}$, $f:\mathbb{R}^n\to\mathbb{R}$ is a $C^{1,1}$-smooth cost
/// function, $U$ is a (not necessarily convex) closed subset of $\mathbb{R}^{n_u}$
/// on which we can easily compute projections (e.g., a rectangle, a ball, a second-order cone,
/// a finite set, etc), $F_1:\mathbb{R}^{n_u}\to\mathbb{R}^{n_1}$ and $F_2:\mathbb{R}^{n_u}
/// \to\mathbb{R}^{n_2}$ are mappings with smooth partial derivatives, and $C\subseteq\mathbb{R}^{n_1}$
/// is a convex closed set on which we can easily compute projections.
///
///
/// # Numerical algorithm
///
///
/// Input:
///
/// - $\epsilon, \delta > 0$ (tolerance),
/// - $\nu_{\max}$ (maximum number of iterations),
/// - $c_0 > 0$ (initial penalty parameter),  
/// - $\epsilon_0 > \epsilon$ (initial tolerance),
/// - $\rho > 1$ (update factor for the penalty parameter)
/// - $\beta \in (0, 1)$ (decrease factor for the inner tolerance)
/// - $\theta \in (0, 1)$ (sufficient decrease coefficient)
/// - $u^0 \in \mathbb{R}^n$ (initial guess)
/// - $y^0 \in \mathbb{R}^{n_1}$ (initial guess for the Lagrange multipliers)
/// - $Y \subseteq C^*$ (compact set)
///
/// The ALM/PM algorithm performs the following iterations:
///
/// - $\bar{\epsilon} = \epsilon_0$, $y\gets{}y^0$, $u\gets{}u^0$, $t,z\gets{}0$
/// - For $\nu=0,\ldots, \nu_{\max}$
///     - $y \gets \Pi_Y(y)$
///     - $u \gets \arg\min_{u\in U} \psi(u, \xi)$, where $\psi(u, \xi)$ is a given function: this problem is
///         solved with tolerance $\bar\epsilon$
///         (see [`AlmFactory`](./struct.AlmFactory.html) regarding how this is constructed)
///     - $y^+ \gets y + c(F_1(u) - \Pi_C(F_1(u) + y/c))$
///     - Define $z^+ \gets \Vert y^+ - y \Vert$ and $t^+ = \Vert F_2(u) \Vert$
///     - If $z^+ \leq c\delta$, $t^+ \leq \delta$ and $\epsilon_\nu \leq \epsilon$, return $(u, y^+)$
///     - else if not ($\nu=0$ or ($z^+ \leq \theta z$ and $t^+ \leq \theta t$)), $c \gets \rho{}c$
///     - $\bar\epsilon \gets \max\\{\epsilon, \beta\bar{\epsilon}\\}$
///
///
/// # Theoretical solution guarantees  
/// The solver determines an $(\epsilon, \delta)$-approximate KKT point for the problem,
/// that is, a pair $(u^\star, y^\star)$ which satisfies
///
/// $$
/// \begin{aligned}
/// v {}\in{}& \partial_u L(u^\star, y^\star), \text{ with } \Vert v \Vert \leq \epsilon,
/// \\\\
/// w {}\in{}& \partial_y [-L](u^\star, y^\star), \text{ with } \Vert w \Vert \leq \delta,
/// \\\\
/// \Vert F_2(u^\star) \Vert {}\leq{}& \delta
/// \end{aligned}
/// $$
///
/// where $L:\mathbb{R}^{n_u}\times\mathbb{R}^{n_1}{}\to{}\mathbb{R}$ is the associated
/// Lagrangian function which is given by
///
/// $$
/// L(u, y) {}={} f(u) + y^\top F_1(u) + \delta_U(u) + \delta_{C^{\ast}}(y),
/// $$
///
/// for $u\in\mathbb{R}^{n_u}$, $y\in\mathbb{R}^{n_1}$, $C^{\ast}$ is the convex conjugate set
/// of $C$ and $\delta_{U}$, $\delta_{C^{\ast}}$ are the indicator functions of $U$ and $C^{\ast}$
/// respectively.
///
pub struct AlmOptimizer<
    'life,
    MappingAlm,
    MappingPm,
    ParametricGradientType,
    ParametricCostType,
    ConstraintsType,
    AlmSetC,
    LagrangeSetY,
> where
    MappingAlm: Fn(&[f64], &mut [f64]) -> FunctionCallResult,
    MappingPm: Fn(&[f64], &mut [f64]) -> FunctionCallResult,
    ParametricGradientType: Fn(&[f64], &[f64], &mut [f64]) -> FunctionCallResult,
    ParametricCostType: Fn(&[f64], &[f64], &mut f64) -> FunctionCallResult,
    ConstraintsType: constraints::Constraint,
    AlmSetC: constraints::Constraint,
    LagrangeSetY: constraints::Constraint,
{
    /// ALM cache (borrowed)
    alm_cache: &'life mut AlmCache,
    /// ALM problem definition (oracle)
    alm_problem: AlmProblem<
        MappingAlm,
        MappingPm,
        ParametricGradientType,
        ParametricCostType,
        ConstraintsType,
        AlmSetC,
        LagrangeSetY,
    >,
    /// Maximum number of outer iterations
    max_outer_iterations: usize,
    /// Maximum number of inner iterations
    max_inner_iterations: usize,
    /// Maximum duration
    max_duration: Option<std::time::Duration>,
    /// epsilon for inner AKKT condition
    epsilon_tolerance: f64,
    /// delta for outer AKKT condition
    delta_tolerance: f64,
    /// At every outer iteration, c is multiplied by this scalar
    penalty_update_factor: f64,
    /// The epsilon-tolerance is multiplied by this factor until
    /// it reaches its target value
    epsilon_update_factor: f64,
    /// If current_infeasibility <= sufficient_decrease_coeff * previous_infeasibility,
    /// then the penalty parameter is kept constant
    sufficient_decrease_coeff: f64,
    // Initial tolerance (for the inner problem)
    epsilon_inner_initial: f64,
}

impl<
        'life,
        MappingAlm,
        MappingPm,
        ParametricGradientType,
        ParametricCostType,
        ConstraintsType,
        AlmSetC,
        LagrangeSetY,
    >
    AlmOptimizer<
        'life,
        MappingAlm,
        MappingPm,
        ParametricGradientType,
        ParametricCostType,
        ConstraintsType,
        AlmSetC,
        LagrangeSetY,
    >
where
    MappingAlm: Fn(&[f64], &mut [f64]) -> FunctionCallResult,
    MappingPm: Fn(&[f64], &mut [f64]) -> FunctionCallResult,
    ParametricGradientType: Fn(&[f64], &[f64], &mut [f64]) -> FunctionCallResult,
    ParametricCostType: Fn(&[f64], &[f64], &mut f64) -> FunctionCallResult,
    ConstraintsType: constraints::Constraint,
    AlmSetC: constraints::Constraint,
    LagrangeSetY: constraints::Constraint,
{
    /* ---------------------------------------------------------------------------- */
    /*          CONSTRUCTOR                                                         */
    /* ---------------------------------------------------------------------------- */

    /// Create new instance of `AlmOptimizer`
    ///
    /// # Arguments
    ///
    /// - `alm_cache`: a reuseable instance of [`AlmCache`](./struct.AlmCache.html), which is borrowed by
    ///    `AlmOptimizer`
    /// - `alm_problem`: the problem specification (data for $\psi(u, \xi)$,
    ///    $\nabla_u \psi(u, \xi)$, $F_1(u)$ (if any), $F_2(u)$ (if any), and sets
    ///    $C$, $U$ and $Y$)
    ///
    ///
    /// # Example
    ///
    /// ```rust
    /// use optimization_engine::{alm::*, FunctionCallResult, core::{panoc::*, constraints}};
    ///
    /// let tolerance = 1e-8;
    /// let nx = 10;
    /// let n1 = 5;
    /// let n2 = 0;
    /// let lbfgs_mem = 3;
    /// let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
    /// let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);
    ///
    /// let psi =  |_u: &[f64], _param: &[f64], _cost: &mut f64| -> FunctionCallResult { Ok(()) };
    /// let d_psi =|_u: &[f64], _param: &[f64], _grad: &mut [f64]| -> FunctionCallResult { Ok(()) };
    /// let f1 = |_u: &[f64], _result: &mut [f64]| -> FunctionCallResult { Ok(()) };
    /// let set_c = constraints::Ball2::new(None, 1.50);
    ///
    /// // Construct an instance of AlmProblem without any PM-type data
    /// let bounds = constraints::Ball2::new(None, 10.0);
    /// let set_y = constraints::Ball2::new(None, 1.0);
    /// let alm_problem = AlmProblem::new(
    ///     bounds,
    ///     Some(set_c),
    ///     Some(set_y),
    ///     psi,
    ///     d_psi,
    ///     Some(f1),
    ///     NO_MAPPING,
    ///     n1,
    ///     n2,
    /// );
    ///
    /// let mut alm_optimizer = AlmOptimizer::new(&mut alm_cache, alm_problem)
    ///     .with_delta_tolerance(1e-4)
    ///     .with_max_outer_iterations(10);
    ///```     
    ///
    pub fn new(
        alm_cache: &'life mut AlmCache,
        alm_problem: AlmProblem<
            MappingAlm,
            MappingPm,
            ParametricGradientType,
            ParametricCostType,
            ConstraintsType,
            AlmSetC,
            LagrangeSetY,
        >,
    ) -> Self {
        // set the initial value of the inner tolerance; this step is
        // not necessary, however, because we set the initial tolerance
        // in #solve (see below)
        alm_cache
            .panoc_cache
            .set_akkt_tolerance(DEFAULT_INITIAL_TOLERANCE);
        AlmOptimizer {
            alm_cache,
            alm_problem,
            max_outer_iterations: DEFAULT_MAX_OUTER_ITERATIONS,
            max_inner_iterations: DEFAULT_MAX_INNER_ITERATIONS,
            max_duration: None,
            epsilon_tolerance: DEFAULT_EPSILON_TOLERANCE,
            delta_tolerance: DEFAULT_DELTA_TOLERANCE,
            penalty_update_factor: DEFAULT_PENALTY_UPDATE_FACTOR,
            epsilon_update_factor: DEFAULT_EPSILON_UPDATE_FACTOR,
            sufficient_decrease_coeff: DEFAULT_INFEAS_SUFFICIENT_DECREASE_FACTOR,
            epsilon_inner_initial: DEFAULT_INITIAL_TOLERANCE,
        }
    }

    /* ---------------------------------------------------------------------------- */
    /*          SETTER METHODS                                                      */
    /* ---------------------------------------------------------------------------- */

    /// Setter method for the maximum number of outer iterations
    ///
    /// # Arguments
    ///
    /// - `max_outer_iterations`: maximum number of outer iterations
    ///
    /// # Returns
    ///
    /// Returns the current mutable and updated instance of the provided object
    ///
    /// # Panics
    ///
    /// The method panics if the specified number of outer iterations is zero
    ///
    ///
    pub fn with_max_outer_iterations(mut self, max_outer_iterations: usize) -> Self {
        assert!(
            max_outer_iterations > 0,
            "max_outer_iterations must be positive"
        );
        self.max_outer_iterations = max_outer_iterations;
        self
    }

    /// Setter method for the maximum number of iterations for the inner problems
    /// which are solved with PANOC (see `PANOCOptimizer`).
    ///
    /// # Arguments
    ///
    /// - `max_inner_iterations`: maximum number of inner iterations
    ///
    /// # Returns
    ///
    /// Returns the current mutable and updated instance of the provided object
    ///
    /// # Panics
    ///
    /// The method panics if the specified number of inner iterations is zero
    ///
    ///
    pub fn with_max_inner_iterations(mut self, max_inner_iterations: usize) -> Self {
        assert!(
            max_inner_iterations > 0,
            "max_inner_iterations must be positive"
        );
        self.max_inner_iterations = max_inner_iterations;
        self
    }

    /// Setter methods for the maximum duration
    ///
    /// If the maximum duration is not set, there is no upper bound on the time
    /// allowed for the ALM/PM optimizer to run
    ///
    /// # Arguments
    ///
    /// - `max_duration`: maximum allowed execution duration
    ///
    /// # Returns
    ///
    /// Returns the current mutable and updated instance of the provided object
    ///
    pub fn with_max_duration(mut self, max_duration: std::time::Duration) -> Self {
        self.max_duration = Some(max_duration);
        self
    }

    /// Set the delta tolerance
    ///
    /// # Arguments
    ///
    /// - `delta_tolerance`: tolerance $\delta > 0$
    ///
    /// # Returns
    ///
    /// Returns the current mutable and updated instance of the provided object
    ///
    /// # Panics
    ///
    /// The method panics if the specified tolerance is not positive
    ///
    pub fn with_delta_tolerance(mut self, delta_tolerance: f64) -> Self {
        assert!(delta_tolerance > 0.0, "delta_tolerance must be positive");
        self.delta_tolerance = delta_tolerance;
        self
    }

    /// Set the epsilon tolerance
    ///
    /// # Arguments
    ///
    /// - `epsilon_tolerance`: tolerance $\epsilon > 0$
    ///
    /// # Returns
    ///
    /// Returns the current mutable and updated instance of the provided object
    ///
    /// # Panics
    ///
    /// The method panics if the specified tolerance is not positive
    ///
    pub fn with_epsilon_tolerance(mut self, epsilon_tolerance: f64) -> Self {
        assert!(
            epsilon_tolerance > 0.0,
            "epsilon_tolerance must be positive"
        );
        self.epsilon_tolerance = epsilon_tolerance;
        self
    }

    /// Setter method for the penalty update factor.
    ///
    /// At every iteration of the ALM/PM algorithm, the penalty parameter, $c_\nu$,
    /// may be updated by multiplying it by a constant factor. This can be specified
    /// with this setter method. The default value is `5.0`.
    ///
    /// # Arguments
    ///
    /// - `penalty_update_factor`: the penalty update factor
    ///
    /// # Returns
    ///
    /// Returns the current mutable and updated instance of the provided object
    ///
    /// # Panics
    ///
    /// The method panics if the update factor is not larger than `1.0 + f64::EPSILON`
    ///
    ///
    pub fn with_penalty_update_factor(mut self, penalty_update_factor: f64) -> Self {
        assert!(
            penalty_update_factor > 1.0 + SMALL_EPSILON,
            "`penalty_update_factor` must be larger than 1.0 + f64::EPSILON"
        );
        self.penalty_update_factor = penalty_update_factor;
        self
    }

    /// Setter method for the update factor for the epsilon tolerance
    ///
    /// The $\epsilon$-tolerance, which is the tolerance passed on to the inner problem,
    /// starts at an initial value $\epsilon_0$, and is updated at every (outer) iteration
    /// of the algorithm by being multiplied with this update factor, which must be in
    /// the interval $(0, 1)$.
    ///
    /// # Arguments
    ///
    /// - `inner_tolerance_update_factor`: the tolerance update factor
    ///
    /// # Returns
    ///
    /// Returns the current mutable and updated instance of the provided object
    ///
    /// # Panics
    ///
    /// The method panics if the specified tolerance update factor is not in the
    /// interval from `f64::EPSILON` to `1.0 - f64::EPSILON`.
    ///
    pub fn with_inner_tolerance_update_factor(
        mut self,
        inner_tolerance_update_factor: f64,
    ) -> Self {
        assert!(
            inner_tolerance_update_factor > SMALL_EPSILON
                && inner_tolerance_update_factor < 1.0 - SMALL_EPSILON,
            "the tolerance update factor needs to be in (f64::EPSILON, 1)"
        );
        self.epsilon_update_factor = inner_tolerance_update_factor;
        self
    }

    /// Setter method for the sufficient decrease coefficient
    ///
    /// The first inner problem is solved at an accuracy $\epsilon_0$. Subsequent
    /// problems are solved at an accuracy $\epsilon_{\nu+1} = \max\{\epsilon, \beta \epsilon_{\nu}\}$,
    /// where $\beta$ is the tolerance update factor and $\epsilon$ is the target
    /// tolerance for the inner problem.
    ///
    /// # Arguments
    ///
    /// - `initial_inner_tolerance`: the initial value of the inner tolerance, that is,
    ///    the value $\espilon_0$
    ///
    /// # Returns
    ///
    /// Returns the current mutable and updated instance of the provided object
    ///
    /// # Panics
    ///
    /// The method panics if the specified initial inner tolerance is less than the
    /// target tolerance. If you need to decrease the target tolerance, please use
    /// `with_inner_tolerance` to do so before invoking `with_initial_inner_tolerance`.
    ///
    ///
    pub fn with_initial_inner_tolerance(mut self, initial_inner_tolerance: f64) -> Self {
        assert!(
            initial_inner_tolerance >= self.epsilon_tolerance,
            "the initial tolerance should be no less than the target tolerance"
        );
        self.epsilon_inner_initial = initial_inner_tolerance;
        // for safety, we update the value of the tolerance in panoc_cache
        self.alm_cache
            .panoc_cache
            .set_akkt_tolerance(initial_inner_tolerance);
        self
    }

    /// Setter method for the sufficient decrease coefficient
    ///
    /// At every (outer) iteration, the ALM/PM may decide not to update the penalty
    /// parameter if the progress has been sufficiently good with respect to the
    /// previous iteration.
    ///
    /// # Arguments
    ///
    /// - `sufficient_decrease_coefficient`: the sufficient decrease coefficient
    ///
    /// # Returns
    ///
    /// Returns the current mutable and updated instance of the provided object
    ///
    /// # Panics
    ///
    /// The method panics if the specified sufficient decrease coefficient is not
    /// in the range `(f64::EPSILON, 1.0 - f64::EPSILON)`
    ///
    pub fn with_sufficient_decrease_coefficient(
        mut self,
        sufficient_decrease_coefficient: f64,
    ) -> Self {
        assert!(
            sufficient_decrease_coefficient < 1.0 - SMALL_EPSILON
                && sufficient_decrease_coefficient > SMALL_EPSILON,
            "sufficient_decrease_coefficient must be in (f64::EPSILON, 1.0 - f64::EPSILON)"
        );
        self.sufficient_decrease_coeff = sufficient_decrease_coefficient;
        self
    }

    /// Setter method for the initial vector of Lagrange multipliers, $y^0$
    ///
    /// # Arguments
    ///
    /// - `y_init`: initial vector of Lagrange multipliers (type: `&[f64]`) of
    ///             length equal to `n1`
    ///
    /// # Returns
    ///
    /// Returns the current mutable and updated instance of the provided object
    ///
    /// # Panics
    ///
    /// The method will panic if the length of `y_init` is not equal to `n1`
    ///
    pub fn with_initial_lagrange_multipliers(mut self, y_init: &[f64]) -> Self {
        let cache = &mut self.alm_cache;
        assert!(
            y_init.len() == self.alm_problem.n1,
            "y_init has wrong length (not equal to n1)"
        );
        // Function `copy_from_slice` would panic if given two arrays (slices)
        // of different lengths; however we catch this earlier in order to provide
        // a meaningful error message
        if let Some(xi_in_cache) = &mut cache.xi {
            xi_in_cache[1..].copy_from_slice(y_init);
        }
        self
    }

    /// Setter method for the initial penalty parameter
    ///
    /// # Arguments
    ///
    /// - `c0`: initial value of the penalty parameter
    ///
    /// # Returns
    ///
    /// Returns the current mutable and updated instance of the provided object
    ///
    /// # Panics
    ///
    /// The method panics if the specified initial penalty parameter is not
    /// larger than `f64::EPSILON`
    ///
    pub fn with_initial_penalty(self, c0: f64) -> Self {
        assert!(
            c0 > SMALL_EPSILON,
            "the initial penalty must be larger than f64::EPSILON"
        );
        if let Some(xi_in_cache) = &mut self.alm_cache.xi {
            xi_in_cache[0] = c0;
        }
        self
    }

    /* ---------------------------------------------------------------------------- */
    /*          PRIVATE METHODS                                                     */
    /* ---------------------------------------------------------------------------- */

    fn compute_alm_infeasibility(&mut self) -> FunctionCallResult {
        let alm_cache = &mut self.alm_cache; // ALM cache
        if let (Some(y_plus), Some(xi)) = (&alm_cache.y_plus, &alm_cache.xi) {
            // compute ||y_plus - y||
            let norm_diff_squared = matrix_operations::norm2_squared_diff(&y_plus, &xi[1..]);
            alm_cache.delta_y_norm_plus = norm_diff_squared.sqrt();
        }
        Ok(())
    }

    /// Computes PM infeasibility, that is, ||F2(u)||
    fn compute_pm_infeasibility(&mut self, u: &[f64]) -> FunctionCallResult {
        let problem = &self.alm_problem; // ALM problem
        let cache = &mut self.alm_cache; // ALM cache

        // If there is an F2 mapping: cache.w_pm <-- F2
        // Then compute the norm of w_pm and store it in cache.f2_norm_plus
        if let (Some(f2), Some(w_pm_vec)) = (&problem.mapping_f2, &mut cache.w_pm.as_mut()) {
            f2(u, w_pm_vec)?;
            cache.f2_norm_plus = matrix_operations::norm2(w_pm_vec);
        }
        Ok(())
    }

    /// Updates the Lagrange multipliers using
    ///
    /// `y_plus <-- y + c*[F1(u_plus) - Proj_C(F1(u_plus) + y/c)]`
    ///
    fn update_lagrange_multipliers(&mut self, u: &[f64]) -> FunctionCallResult {
        let problem = &self.alm_problem; // ALM problem
        let cache = &mut self.alm_cache; // ALM cache

        // y_plus <-- y + c*[F1(u_plus) - Proj_C(F1(u_plus) + y/c)]
        // This is implemented as follows:
        //
        // #1. w_alm_aux := F1(u), where u = solution of inner problem
        // #2. y_plus := w_alm_aux + y/c
        // #3. y_plus := Proj_C(y_plus)
        // #4. y_plus := y + c(w_alm_aux - y_plus)

        // Before we start: this should not be executed if n1 = 0
        if problem.n1 == 0 {
            return Ok(()); // nothing to do (no ALM), return
        }

        if let (Some(f1), Some(w_alm_aux), Some(y_plus), Some(xi), Some(alm_set_c)) = (
            &problem.mapping_f1,
            &mut cache.w_alm_aux,
            &mut cache.y_plus,
            &mut cache.xi,
            &problem.alm_set_c,
        ) {
            // Step #1: w_alm_aux := F1(u)
            (f1)(u, w_alm_aux)?;

            // Step #2: y_plus := w_alm_aux + y/c
            let y = &xi[1..];
            let c = xi[0];
            y_plus
                .iter_mut()
                .zip(y.iter())
                .zip(w_alm_aux.iter())
                .for_each(|((y_plus_i, y_i), w_alm_aux_i)| *y_plus_i = w_alm_aux_i + y_i / c);

            // Step #3: y_plus := Proj_C(y_plus)
            alm_set_c.project(y_plus);

            // Step #4
            y_plus
                .iter_mut()
                .zip(y.iter())
                .zip(w_alm_aux.iter())
                .for_each(|((y_plus_i, y_i), w_alm_aux_i)| {
                    // y_plus := y  + c * (w_alm_aux   - y_plus)
                    *y_plus_i = y_i + c * (w_alm_aux_i - *y_plus_i)
                });
        }

        Ok(())
    }

    /// Project y on set Y
    fn project_on_set_y(&mut self) {
        let problem = &self.alm_problem;
        if let Some(y_set) = &problem.alm_set_y {
            // NOTE: as_mut() converts from &mut Option<T> to Option<&mut T>
            // * cache.y is                Option<Vec<f64>>
            // * cache.y.as_mut is         Option<&mut Vec<f64>>
            // *  which can be treated as  Option<&mut [f64]>
            // * y_vec is                  &mut [f64]
            if let Some(xi_vec) = self.alm_cache.xi.as_mut() {
                y_set.project(&mut xi_vec[1..]);
            }
        }
    }

    /// Solve inner problem
    ///
    /// # Arguments
    ///
    /// - `u`: (on entry) current iterate, `u^nu`, (on exit) next iterate,
    ///   `u^{nu+1}` which is an epsilon-approximate solution of the inner problem
    /// - `xi`: vector `xi = (c, y)`
    ///
    /// # Returns
    ///
    /// Returns an instance of `Result<SolverStatus, SolverError>`, where `SolverStatus`
    /// is the solver status of the inner problem and `SolverError` is a potential
    /// error in solving the inner problem.
    ///
    ///
    fn solve_inner_problem(&mut self, u: &mut [f64]) -> Result<SolverStatus, SolverError> {
        let alm_problem = &self.alm_problem; // Problem
        let alm_cache = &mut self.alm_cache; // ALM cache

        // `xi` is either the cached `xi` if one exists, or an reference to an
        // empty vector, otherwise. We do that becaues the user has the option
        // to not use any ALM/PM constraints; in that case, `alm_cache.xi` is
        // `None`
        let xi_empty = Vec::new();
        let xi = if let Some(xi_cached) = &alm_cache.xi {
            &xi_cached
        } else {
            &xi_empty
        };
        // Construct psi and psi_grad (as functions of `u` alone); it is
        // psi(u) = psi(u; xi) and psi_grad(u) = phi_grad(u; xi)
        // psi: R^nu --> R
        let psi = |u: &[f64], psi_val: &mut f64| -> FunctionCallResult {
            (alm_problem.parametric_cost)(u, &xi, psi_val)
        };
        // psi_grad: R^nu --> R^nu
        let psi_grad = |u: &[f64], psi_grad: &mut [f64]| -> FunctionCallResult {
            (alm_problem.parametric_gradient)(u, &xi, psi_grad)
        };
        // define the inner problem
        let inner_problem = Problem::new(&self.alm_problem.constraints, psi_grad, psi);
        // The AKKT-tolerance decreases until it reaches the target tolerance
        // We don't need to update the tolerance here; this is done in
        // `update_inner_akkt_tolerance` which updates the AKKT-tolerance (epsilon)
        // in the PANOCCache instance held by AlmCache directly.
        let mut inner_solver = PANOCOptimizer::new(inner_problem, &mut alm_cache.panoc_cache)
            // Set the maximum duration of the inner solver to the available time, which is
            // stored in AlmCache, or set it to the maximum possible duration
            .with_max_duration(
                alm_cache
                    .available_time
                    .unwrap_or(std::time::Duration::from_secs(std::u64::MAX)),
            )
            // Set the maximum number of inner iterations
            .with_max_iter(self.max_inner_iterations);
        // this method returns the result of .solve:
        inner_solver.solve(u)
    }

    fn is_exit_criterion_satisfied(&self) -> bool {
        let cache = &self.alm_cache;
        let problem = &self.alm_problem;
        // Criterion 1: ||Delta y|| <= c * delta
        //              If n1 = 0 (if there are not ALM-type constraints)
        //              then this criterion is automatically satisfied
        let criterion_1 = problem.n1 == 0
            || if let Some(xi) = &cache.xi {
                let c = xi[0];
                cache.iteration > 0
                    && cache.delta_y_norm_plus <= c * self.delta_tolerance + SMALL_EPSILON
            } else {
                true
            };
        // Criterion 2: ||F2(u+)|| <= delta
        //              If n2 = 0, there are no PM-type constraints, so this
        //              criterion is automatically satisfied
        let criterion_2 =
            problem.n2 == 0 || cache.f2_norm_plus <= self.delta_tolerance + SMALL_EPSILON;
        // Criterion 3: epsilon_nu <= epsilon
        //              This function will panic is there is no akkt_tolerance
        //              This should never happen because we set the AKKT tolerance
        //              in the constructor and can never become `None` again
        let criterion_3 =
            cache.panoc_cache.akkt_tolerance.unwrap() <= self.epsilon_tolerance + SMALL_EPSILON;
        criterion_1 && criterion_2 && criterion_3
    }

    /// Whether the penalty parameter should not be updated
    fn is_penalty_stall_criterion(&self) -> bool {
        let cache = &self.alm_cache;
        let problem = &self.alm_problem;
        // Check whether the penalty parameter should not be updated
        // This is if iteration = 0, or there has been a sufficient
        // decrease in infeasibility
        if cache.iteration == 0 {
            return true;
        }
        let is_alm = problem.n1 > 0;
        let is_pm = problem.n2 > 0;
        let criterion_alm = cache.delta_y_norm_plus
            <= self.sufficient_decrease_coeff * cache.delta_y_norm + SMALL_EPSILON;
        let criterion_pm =
            cache.f2_norm_plus <= self.sufficient_decrease_coeff * cache.f2_norm + SMALL_EPSILON;
        if is_alm && !is_pm {
            return criterion_alm;
        } else if !is_alm && is_pm {
            return criterion_pm;
        } else if is_alm && is_pm {
            return criterion_alm && criterion_pm;
        }

        false
    }

    fn update_penalty_parameter(&mut self) {
        let cache = &mut self.alm_cache;
        if let Some(xi) = &mut cache.xi {
            xi[0] *= self.penalty_update_factor;
        }
    }

    fn update_inner_akkt_tolerance(&mut self) {
        let cache = &mut self.alm_cache;
        // epsilon_{nu+1} := max(epsilon, beta*epsilon_nu)
        cache.panoc_cache.set_akkt_tolerance(f64::max(
            cache.panoc_cache.akkt_tolerance.unwrap() * self.epsilon_update_factor,
            self.epsilon_tolerance,
        ));
    }

    fn final_cache_update(&mut self) {
        let cache = &mut self.alm_cache;
        cache.iteration += 1;
        cache.delta_y_norm = cache.delta_y_norm_plus;
        cache.f2_norm = cache.f2_norm_plus;
        if let (Some(xi), Some(y_plus)) = (&mut cache.xi, &cache.y_plus) {
            xi[1..].copy_from_slice(&y_plus);
        }
        cache.panoc_cache.reset();
    }

    /// Step of ALM algorithm
    ///
    /// # Description
    ///
    /// It involves the following actions:
    ///
    /// - Projects `y` on `Y`
    /// - Solves the inner problem for the current `xi` up to tol. `epsilon`
    /// - Updates the Lagrange multipliers
    /// - Computes infeasibilities
    /// - Exits if the temination criteria are satisfied OR
    /// - Updates the penalty parameter
    /// - Shrinks the inner tolerance and
    /// - Updates the ALM cache
    ///
    fn step(&mut self, u: &mut [f64]) -> Result<InnerProblemStatus, SolverError> {
        // store the exit status of the inner problem in this problem
        // (we'll need to return it within `InnerProblemStatus`)
        let mut inner_exit_status: ExitStatus = ExitStatus::Converged;

        // Project y on Y
        self.project_on_set_y();

        // If the inner problem fails miserably, the failure should be propagated
        // upstream (using `?`). If the inner problem has not converged, that is fine,
        // we should keep solving.
        self.solve_inner_problem(u).map(|status: SolverStatus| {
            let inner_iters = status.iterations();
            self.alm_cache.last_inner_problem_norm_fpr = status.norm_fpr();
            self.alm_cache.inner_iteration_count += inner_iters;
            inner_exit_status = status.exit_status();
        })?;

        // TODO: Check whether the inner problem has converged; set a limit on
        // FPR above which the outer loop cannot reduce the error? (not sure how)

        // Update Lagrange multipliers:
        // y_plus <-- y + c*[F1(u_plus) - Proj_C(F1(u_plus) + y/c)]
        self.update_lagrange_multipliers(u)?;

        // Compute infeasibilities
        self.compute_pm_infeasibility(u)?; // penalty method: F2(u_plus) and its norm
        self.compute_alm_infeasibility()?; // ALM: ||y_plus - y||

        // Check exit criterion
        if self.is_exit_criterion_satisfied() {
            // Do not continue the outer iteration
            // An (epsilon, delta)-AKKT point has been found
            return Ok(InnerProblemStatus::new(false, inner_exit_status));
        } else if !self.is_penalty_stall_criterion() {
            self.update_penalty_parameter();
        }

        // Update inner problem tolerance
        self.update_inner_akkt_tolerance();

        // conclusive step: updated iteration count, resets PANOC cache,
        // sets f2_norm = f2_norm_plus etc
        self.final_cache_update();

        Ok(InnerProblemStatus::new(true, inner_exit_status)) // `true` means do continue the outer iterations
    }

    fn compute_cost_at_solution(&mut self, u: &mut [f64]) -> Result<f64, SolverError> {
        /* WORK IN PROGRESS */
        let alm_problem = &self.alm_problem; // Problem
        let alm_cache = &mut self.alm_cache; // ALM Cache
        let mut empty_vec = std::vec::Vec::new(); // Empty vector
        let xi: &mut std::vec::Vec<f64> = alm_cache.xi.as_mut().unwrap_or(&mut empty_vec);
        let mut __c: f64 = 0.0;
        if !xi.is_empty() {
            __c = xi[0];
            xi[0] = 0.0;
        }
        let mut cost_value: f64 = 0.0;
        (alm_problem.parametric_cost)(u, xi, &mut cost_value)?;
        if !xi.is_empty() {
            xi[0] = __c;
        }
        Ok(cost_value)
    }

    /* ---------------------------------------------------------------------------- */
    /*          MAIN API                                                            */
    /* ---------------------------------------------------------------------------- */

    /// Solve the specified ALM problem
    ///
    ///
    pub fn solve(&mut self, u: &mut [f64]) -> Result<AlmOptimizerStatus, SolverError> {
        let mut num_outer_iterations = 0;
        let tic = std::time::Instant::now();
        let mut exit_status = ExitStatus::Converged;
        self.alm_cache.reset(); // first, reset the cache
        self.alm_cache.available_time = self.max_duration;

        self.alm_cache
            .panoc_cache
            .set_akkt_tolerance(self.epsilon_inner_initial);

        let mut inner = InnerProblemStatus::new(false, ExitStatus::Converged);
        for _outer_iters in 1..=self.max_outer_iterations {
            if let Some(max_duration) = self.max_duration {
                let available_time_left = max_duration.checked_sub(tic.elapsed());
                self.alm_cache.available_time = available_time_left;
                if available_time_left.is_none() {
                    // no time left for outer iterations!
                    exit_status = ExitStatus::NotConvergedOutOfTime;
                    break;
                }
            }
            num_outer_iterations += 1;
            inner = self.step(u)?;
            if inner.inner_problem_exit_status == ExitStatus::NotConvergedOutOfTime {
                // the inner problem solver says there was no time left
                exit_status = ExitStatus::NotConvergedOutOfTime;
                break;
            }
            if !inner.outer_continue_iterating {
                break;
            }
        }

        // after outer loop: if the outer loop has terminated, and it was no interrupted
        // because it ran out of time, then the final exit status should be the exit
        // status of the last inner problem (the last inner problem determines the success
        // or failure of the overall computation)
        if exit_status != ExitStatus::NotConvergedOutOfTime {
            exit_status = inner.inner_problem_exit_status;
        }

        // after outer loop: if the maximum number of outer iterations was reached
        // and the last invocation to self.step() suggests that the outer loop should
        // continue, this means that the solver reached the max num of OUTER iterations
        if num_outer_iterations == self.max_outer_iterations && inner.outer_continue_iterating {
            exit_status = ExitStatus::NotConvergedIterations;
        }

        // obtain the penalty parameter
        let c = if let Some(xi) = &self.alm_cache.xi {
            xi[0]
        } else {
            0.0
        };

        let cost = self.compute_cost_at_solution(u)?;
        let status = AlmOptimizerStatus::new(exit_status)
            .with_solve_time(tic.elapsed())
            .with_inner_iterations(self.alm_cache.inner_iteration_count)
            .with_outer_iterations(num_outer_iterations)
            .with_last_problem_norm_fpr(self.alm_cache.last_inner_problem_norm_fpr)
            .with_delta_y_norm(self.alm_cache.delta_y_norm_plus)
            .with_f2_norm(self.alm_cache.f2_norm_plus)
            .with_penalty(c)
            .with_cost(cost);
        if self.alm_problem.n1 > 0 {
            let status = status.with_lagrange_multipliers(
                &self
                    .alm_cache
                    .y_plus
                    .as_ref()
                    .expect("Although n1 > 0, there is no vector y (Lagrange multipliers)"),
            );
            Ok(status)
        } else {
            Ok(status)
        }
    }
}

/* ---------------------------------------------------------------------------- */
/*          TESTS                                                               */
/* ---------------------------------------------------------------------------- */
#[cfg(test)]
mod tests {

    use crate::{
        alm::*,
        core::{constraints::*, panoc::*, ExitStatus},
        matrix_operations,
        mocks::*,
        FunctionCallResult,
    };

    fn make_dummy_alm_problem(
        n1: usize,
        n2: usize,
    ) -> AlmProblem<
        impl Fn(&[f64], &mut [f64]) -> FunctionCallResult,
        impl Fn(&[f64], &mut [f64]) -> FunctionCallResult,
        impl Fn(&[f64], &[f64], &mut [f64]) -> FunctionCallResult,
        impl Fn(&[f64], &[f64], &mut f64) -> FunctionCallResult,
        impl Constraint,
        impl Constraint,
        impl Constraint,
    > {
        // Main problem data
        let psi = void_parameteric_cost;
        let d_psi = void_parameteric_gradient;
        let bounds = Ball2::new(None, 10.0);
        // ALM-type data
        let f1: Option<MappingType> = if n1 == 0 {
            NO_MAPPING
        } else {
            Some(void_mapping)
        };
        let set_c = if n1 > 0 {
            Some(Ball2::new(None, 1.50))
        } else {
            None::<Ball2>
        };
        let set_y: Option<Ball2> = if n1 > 0 {
            Some(Ball2::new(None, 2.0))
        } else {
            None::<Ball2>
        };
        // Penalty-type data
        let f2: Option<MappingType> = if n2 == 0 {
            NO_MAPPING
        } else {
            Some(void_mapping)
        };
        // problem
        AlmProblem::new(bounds, set_c, set_y, psi, d_psi, f1, f2, n1, n2)
    }

    #[test]
    fn t_setter_methods() {
        let (tolerance, nx, n1, n2, lbfgs_mem) = (1e-8, 10, 5, 0, 3);
        let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
        let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);
        let alm_problem = make_dummy_alm_problem(n1, n2);

        let alm_optimizer = AlmOptimizer::new(&mut alm_cache, alm_problem);

        // Test: the initial value of the penalty parameter is positive
        if let Some(xi) = &alm_optimizer.alm_cache.xi {
            assert!(xi[0] > std::f64::EPSILON);
        }

        // Test: with_initial_penalty
        let alm_optimizer = alm_optimizer.with_initial_penalty(7.0);
        assert!(!alm_optimizer.alm_cache.xi.is_none());
        if let Some(xi) = &alm_optimizer.alm_cache.xi {
            unit_test_utils::assert_nearly_equal(
                7.0,
                xi[0],
                1e-10,
                1e-12,
                "initial penalty parameter not set properly",
            );
        }

        // Test: with_initial_lagrange_multipliers
        let y_init = vec![2.0, 3.0, 4.0, 5.0, 6.0];
        let alm_optimizer = alm_optimizer.with_initial_lagrange_multipliers(&y_init);
        if let Some(xi) = &alm_optimizer.alm_cache.xi {
            unit_test_utils::assert_nearly_equal_array(
                &y_init,
                &xi[1..],
                1e-10,
                1e-12,
                "initial Langrange multipliers not set properly",
            );
        }

        // Test: with_initial_inner_tolerance
        let alm_optimizer = alm_optimizer.with_initial_inner_tolerance(5e-3);
        unit_test_utils::assert_nearly_equal(
            5e-3,
            alm_optimizer.epsilon_inner_initial,
            1e-10,
            1e-12,
            "initial tolerance not properly set",
        );
        if let Some(akkt_tolerance) = alm_optimizer.alm_cache.panoc_cache.akkt_tolerance {
            unit_test_utils::assert_nearly_equal(
                5e-3,
                akkt_tolerance,
                1e-10,
                1e-12,
                "initial tolerance not properly set",
            );
        } else {
            panic!("PANOCCache has no (initial) AKKT-tolerance set");
        }
    }

    #[test]
    fn t_project_on_set_y() {
        let (tolerance, nx, n1, n2, lbfgs_mem) = (1e-8, 10, 4, 0, 3);
        let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
        let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);
        let alm_problem = make_dummy_alm_problem(n1, n2);

        // y0 = [2, 3, 4, 10]
        // ||y0|| = 11.3578166916005
        // The projection of y0 on Y = Ball(0; 2) is 2*y0/||y0|| (since y0 not in C)
        // > P_C(y0) = [0.352180362530250
        //              0.528270543795374
        //              0.704360725060499
        //              1.760901812651248]
        //
        let mut alm_optimizer = AlmOptimizer::new(&mut alm_cache, alm_problem)
            .with_initial_penalty(25.0)
            .with_initial_lagrange_multipliers(&[2., 3., 4., 10.]);

        alm_optimizer.project_on_set_y();
        if let Some(xi_after_proj) = &alm_optimizer.alm_cache.xi {
            println!("xi = {:#?}", xi_after_proj);
            let y_projected_correct = [
                0.352_180_362_530_250,
                0.528_270_543_795_374,
                0.704_360_725_060_499,
                1.760_901_812_651_248,
            ];
            unit_test_utils::assert_nearly_equal_array(
                &xi_after_proj[1..],
                &y_projected_correct,
                1e-10,
                1e-15,
                "wrong projection on Y",
            );
            unit_test_utils::assert_nearly_equal(
                25.0,
                xi_after_proj[0],
                1e-10,
                1e-16,
                "penalty parameter affected by projection step (on Y)",
            );
        } else {
            panic!("no xi found after projection!");
        }
    }

    #[test]
    fn t_compute_pm_infeasibility() {
        // Tests whether compute_pm_infeasibility() works properly: it need to compute
        // F2(u_plus) and ||F2(u_plus)||. It stores F2(u_plus) in alm_cache.w_pm
        let (tolerance, nx, n1, n2, lbfgs_mem) = (1e-6, 5, 0, 2, 3);
        let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
        let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);
        let psi = void_parameteric_cost;
        let d_psi = void_parameteric_gradient;
        let f2 = Some(|u: &[f64], res: &mut [f64]| -> FunctionCallResult {
            res[0] = matrix_operations::sum(u);
            res[1] = matrix_operations::norm2_squared(u);
            Ok(())
        });
        let bounds = Ball2::new(None, 10.0);
        let alm_problem =
            AlmProblem::new(bounds, NO_SET, NO_SET, psi, d_psi, NO_MAPPING, f2, n1, n2);
        let mut alm_optimizer =
            AlmOptimizer::new(&mut alm_cache, alm_problem).with_initial_penalty(10.0);

        let u_plus = vec![1.0, 5.0, -2.0, 9.0, -6.0];
        assert!(alm_optimizer.compute_pm_infeasibility(&u_plus).is_ok());
        let alm_cache = &alm_optimizer.alm_cache;
        let f2_u_plus = &alm_cache.w_pm.as_ref().unwrap();
        println!("F2(u_plus) = {:#?}", f2_u_plus);
        unit_test_utils::assert_nearly_equal_array(
            &[7., 147.],
            &f2_u_plus,
            1e-10,
            1e-12,
            "F2(u) is wrong",
        );
        // ||F2(u_plus)|| = 147.166572291400
        println!("||F2(u_plus)|| = {}", alm_cache.f2_norm_plus);
        unit_test_utils::assert_nearly_equal(
            alm_cache.f2_norm_plus,
            147.166_572_291_400,
            1e-12,
            1e-12,
            "||F2(u_plus)|| is wrong",
        );
    }

    #[test]
    fn t_compute_alm_infeasibility() {
        let (tolerance, nx, n1, n2, lbfgs_mem) = (1e-6, 5, 4, 0, 3);
        let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
        let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);
        let psi = void_parameteric_cost;
        let d_psi = void_parameteric_gradient;
        let f1 = Some(void_mapping);
        let set_c = Some(Ball2::new(None, 1.0));
        let bounds = Ball2::new(None, 10.0);
        let set_y = Some(Ball2::new(None, 2.0));
        let alm_problem = AlmProblem::new(bounds, set_c, set_y, psi, d_psi, f1, NO_MAPPING, n1, n2);
        // Set y0 = [2, 3, 4, 10]
        let mut alm_optimizer = AlmOptimizer::new(&mut alm_cache, alm_problem)
            .with_initial_penalty(10.0)
            .with_initial_lagrange_multipliers(&[2., 3., 4., 10.]);
        {
            let cache = &mut alm_optimizer.alm_cache;
            // Set y1 = [10, 20, 11, 100]
            if let Some(y_plus) = &mut cache.y_plus {
                y_plus.copy_from_slice(&[10., 20., 11., 100.]);
            }
        }
        assert!(alm_optimizer.compute_alm_infeasibility().is_ok());
        unit_test_utils::assert_nearly_equal(
            92.206_290_457_864_1,
            alm_optimizer.alm_cache.delta_y_norm_plus,
            1e-10,
            1e-12,
            "delta_y_plus is wrong",
        );
    }

    #[test]
    fn t_update_lagrange_multipliers() {
        let (tolerance, nx, n1, n2, lbfgs_mem) = (1e-6, 5, 2, 0, 3);
        let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
        let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);
        let psi = void_parameteric_cost;
        let d_psi = void_parameteric_gradient;
        let f1 = Some(|u: &[f64], res: &mut [f64]| -> FunctionCallResult {
            res[0] = matrix_operations::sum(u);
            res[1] = matrix_operations::norm2_squared(&u);
            Ok(())
        });
        let set_c = Some(Ball2::new(None, 1.5));
        let bounds = Ball2::new(None, 10.0);
        let set_y = Some(Ball2::new(None, 2.0));
        let alm_problem = AlmProblem::new(bounds, set_c, set_y, psi, d_psi, f1, NO_MAPPING, n1, n2);

        // Set y0 = [2, 3, 4, 10]
        let mut alm_optimizer = AlmOptimizer::new(&mut alm_cache, alm_problem)
            .with_initial_penalty(10.0)
            .with_initial_lagrange_multipliers(&[2., 3.]);
        let u = [3.0, 5.0, 7.0, 9.0, 11.];
        assert!(alm_optimizer.update_lagrange_multipliers(&u).is_ok());

        println!("xi = {:#?}", alm_optimizer.alm_cache.w_alm_aux);
        unit_test_utils::assert_nearly_equal_array(
            &[350.163_243_585_489, 2_838.112_880_538_07],
            &alm_optimizer
                .alm_cache
                .y_plus
                .as_ref()
                .expect("no y_plus found (it is None)"),
            1e-12,
            1e-12,
            "y_plus is wrong",
        );
    }

    #[test]
    fn t_update_inner_akkt_tolerance() {
        let (tolerance, nx, n1, n2, lbfgs_mem) = (1e-8, 10, 0, 0, 3);
        let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
        let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);
        let alm_problem = make_dummy_alm_problem(n1, n2);
        let mut alm_optimizer = AlmOptimizer::new(&mut alm_cache, alm_problem)
            .with_epsilon_tolerance(2e-5)
            .with_initial_inner_tolerance(1e-1)
            .with_inner_tolerance_update_factor(0.2);

        alm_optimizer.update_inner_akkt_tolerance();

        unit_test_utils::assert_nearly_equal(
            0.1,
            alm_optimizer.epsilon_inner_initial,
            1e-16,
            1e-12,
            "target tolerance altered by update_inner_akkt_tolerance",
        );

        unit_test_utils::assert_nearly_equal(
            0.02,
            alm_optimizer
                .alm_cache
                .panoc_cache
                .akkt_tolerance
                .expect("there should be a set AKKT tolerance"),
            1e-12,
            1e-12,
            "panoc_cache tolerance is not properly updated",
        );

        for _i in 1..=5 {
            alm_optimizer.update_inner_akkt_tolerance();
        }
        unit_test_utils::assert_nearly_equal(
            2e-5,
            alm_optimizer
                .alm_cache
                .panoc_cache
                .akkt_tolerance
                .expect("there should be a set AKKT tolerance"),
            1e-12,
            1e-12,
            "panoc_cache tolerance is not properly updated",
        );
    }

    #[test]
    fn t_update_penalty_parameter() {
        let (tolerance, nx, n1, n2, lbfgs_mem) = (1e-6, 5, 0, 2, 3);
        let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
        let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);
        let alm_problem = make_dummy_alm_problem(n1, n2);
        let mut alm_optimizer = AlmOptimizer::new(&mut alm_cache, alm_problem)
            .with_initial_penalty(5.0)
            .with_penalty_update_factor(15.0);
        if let Some(xi) = &alm_optimizer.alm_cache.xi {
            unit_test_utils::assert_nearly_equal(xi[0], 5.0, 1e-16, 1e-12, "wrong initial penalty");
        }
        alm_optimizer.update_penalty_parameter();
        if let Some(xi) = &alm_optimizer.alm_cache.xi {
            unit_test_utils::assert_nearly_equal(
                xi[0],
                75.0,
                1e-16,
                1e-12,
                "wrong updated penalty",
            );
        }
        alm_optimizer.update_penalty_parameter();
        if let Some(xi) = &alm_optimizer.alm_cache.xi {
            unit_test_utils::assert_nearly_equal(
                xi[0],
                1125.0,
                1e-16,
                1e-12,
                "wrong updated penalty",
            );
        }
    }

    #[test]
    fn t_final_cache_update() {
        let (tolerance, nx, n1, n2, lbfgs_mem) = (1e-6, 5, 2, 2, 3);
        let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
        let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);
        let alm_problem = make_dummy_alm_problem(n1, n2);
        let mut alm_optimizer = AlmOptimizer::new(&mut alm_cache, alm_problem);
        alm_optimizer.alm_cache.reset();
        alm_optimizer.alm_cache.delta_y_norm_plus = 1.2345;
        alm_optimizer.alm_cache.f2_norm_plus = 3.45678;
        if let Some(xi) = &mut alm_optimizer.alm_cache.xi {
            xi[1..].copy_from_slice(&[5.6, 7.8]);
        }
        assert_eq!(
            0, alm_optimizer.alm_cache.iteration,
            "initial iteration count should be 0"
        );

        alm_optimizer.final_cache_update();

        assert_eq!(
            1, alm_optimizer.alm_cache.iteration,
            "iteration count not updated"
        );
        unit_test_utils::assert_nearly_equal(
            3.45678,
            alm_optimizer.alm_cache.f2_norm,
            1e-16,
            1e-12,
            "f2_norm was not updated after final_cache_update()",
        );
        unit_test_utils::assert_nearly_equal(
            1.2345,
            alm_optimizer.alm_cache.delta_y_norm,
            1e-16,
            1e-12,
            "delta_y_norm was not updated after final_cache_update()",
        );
        assert_eq!(
            0, alm_optimizer.alm_cache.panoc_cache.iteration,
            "panoc_cache iteration count not updated"
        );
        println!("cache now = {:#?}", &alm_optimizer.alm_cache);
    }

    #[test]
    fn t_is_exit_criterion_satisfied() {
        let (tolerance, nx, n1, n2, lbfgs_mem) = (1e-6, 5, 2, 2, 3);
        let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
        let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);
        alm_cache.iteration = 2;
        let alm_problem = make_dummy_alm_problem(n1, n2);
        let alm_optimizer =
            AlmOptimizer::new(&mut alm_cache, alm_problem).with_delta_tolerance(1e-3);

        // should not exit yet...
        assert!(
            !alm_optimizer.is_exit_criterion_satisfied(),
            "exists right away"
        );

        let mut alm_optimizer = alm_optimizer
            .with_initial_inner_tolerance(1e-3)
            .with_epsilon_tolerance(1e-3);
        assert!(!alm_optimizer.is_exit_criterion_satisfied());

        alm_optimizer.alm_cache.delta_y_norm_plus = 1e-3;
        assert!(!alm_optimizer.is_exit_criterion_satisfied());

        alm_optimizer.alm_cache.f2_norm_plus = 1e-3;
        assert!(alm_optimizer.is_exit_criterion_satisfied());
    }

    #[test]
    fn t_is_penalty_stall_criterion() {
        let (tolerance, nx, n1, n2, lbfgs_mem) = (1e-8, 10, 1, 1, 3);
        let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
        let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);
        let alm_problem = make_dummy_alm_problem(n1, n2);
        let mut alm_optimizer = AlmOptimizer::new(&mut alm_cache, alm_problem)
            .with_sufficient_decrease_coefficient(0.1);

        // should stall because iteration = 0
        assert!(alm_optimizer.is_penalty_stall_criterion());

        alm_optimizer.alm_cache.iteration = 4;
        assert!(!alm_optimizer.is_penalty_stall_criterion());

        alm_optimizer.alm_cache.delta_y_norm = 100.0;
        alm_optimizer.alm_cache.delta_y_norm_plus = 10.0;
        alm_optimizer.alm_cache.f2_norm = 200_000.0;
        alm_optimizer.alm_cache.f2_norm_plus = 20_000.0;

        assert!(alm_optimizer.is_penalty_stall_criterion());
        println!("cache = {:#?}", alm_optimizer.alm_cache);
    }

    #[test]
    fn t_is_penalty_stall_criterion_alm() {
        // --- ONLY ALM (n1 > 0, n2 = 0)
        let (tolerance, nx, n1, n2, lbfgs_mem) = (1e-8, 10, 1, 0, 3);
        let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
        let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);
        let alm_problem = make_dummy_alm_problem(n1, n2);
        let mut alm_optimizer = AlmOptimizer::new(&mut alm_cache, alm_problem)
            .with_sufficient_decrease_coefficient(0.1);

        // should stall because iteration = 0
        assert!(alm_optimizer.is_penalty_stall_criterion());

        alm_optimizer.alm_cache.iteration = 4;
        assert!(!alm_optimizer.is_penalty_stall_criterion());

        alm_optimizer.alm_cache.delta_y_norm = 100.0;
        alm_optimizer.alm_cache.delta_y_norm_plus = 10.0;
        alm_optimizer.alm_cache.f2_norm = 0.0;
        alm_optimizer.alm_cache.f2_norm_plus = 0.0;

        assert!(alm_optimizer.is_penalty_stall_criterion());
        println!("cache = {:#?}", alm_optimizer.alm_cache);
    }

    #[test]
    fn t_is_penalty_stall_criterion_pm() {
        // -- ONLY PM (n1 = 0, n2 > 0)
        let (tolerance, nx, n1, n2, lbfgs_mem) = (1e-8, 10, 0, 1, 3);
        let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
        let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);
        let alm_problem = make_dummy_alm_problem(n1, n2);
        let mut alm_optimizer = AlmOptimizer::new(&mut alm_cache, alm_problem)
            .with_sufficient_decrease_coefficient(0.1);

        // should stall because iteration = 0
        assert!(alm_optimizer.is_penalty_stall_criterion());

        alm_optimizer.alm_cache.iteration = 4;
        assert!(!alm_optimizer.is_penalty_stall_criterion());

        alm_optimizer.alm_cache.delta_y_norm = 0.0;
        alm_optimizer.alm_cache.delta_y_norm_plus = 0.0;
        alm_optimizer.alm_cache.f2_norm = 200_000.0;
        alm_optimizer.alm_cache.f2_norm_plus = 20_000.0;

        assert!(alm_optimizer.is_penalty_stall_criterion());
        println!("cache = {:#?}", alm_optimizer.alm_cache);
    }

    #[test]
    fn t_solve_inner_problem() {
        // MATLAB code to find the solution of this problem:
        //
        // % file: psi_cost.m
        // function y = psi_cost(x, xi)
        // y = 0.5*x'*x + xi(1)*sum(x);
        // m = min([length(x), length(xi) - 1]);
        // y = y + xi(2:m+1)'*x(1:m);
        //
        // % Then, run the following:
        // f = @(x) psi_cost(x, [1.0; 5.0; 6.0])
        // x_sol = fmincon(f,[0;0;0;0;0],[],[],[],[],-5*ones(5,1),zeros(5,1))
        //

        // NOTE: Variable `tolerance` is the tolerance on FPR; this is important
        //       so as to have an accurate solution of the inner problem, but we
        //       are actually using a different inner termination criterion with
        //       tolerance specified by AlmOptimizer.with_initial_inner_tolerance
        let (tolerance, nx, n1, n2, lbfgs_mem) = (0.1, 5, 2, 0, 3);
        let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
        let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);
        let psi = psi_cost_dummy;
        let d_psi = psi_gradient_dummy;
        let f1 = Some(void_mapping);
        let set_c = Some(Ball2::new(None, 1.5));
        let xmin = vec![-5.0; nx];
        let xmax = vec![0.0; nx];
        let bounds = Rectangle::new(Some(&xmin), Some(&xmax));
        let set_y = Some(Ball2::new(None, 2.0));
        let alm_problem = AlmProblem::new(bounds, set_c, set_y, psi, d_psi, f1, NO_MAPPING, n1, n2);

        // Set y0 = [5.0, 6.0] and initial penalty = 1.0 (so, xi = [1.0, 5.0, 6.0])
        let mut alm_optimizer = AlmOptimizer::new(&mut alm_cache, alm_problem)
            .with_initial_lagrange_multipliers(&[5.0, 6.0])
            .with_initial_penalty(1.0)
            .with_epsilon_tolerance(1e-12)
            .with_initial_inner_tolerance(1e-12);
        let mut u = vec![0.0; nx];
        let result = alm_optimizer.solve_inner_problem(&mut u);
        println!("result = {:#?}", &result);
        println!("u = {:#?}", &u);
        assert!(result.is_ok());
        let solver_status = result.unwrap();
        assert!(solver_status.has_converged());
        assert_eq!(ExitStatus::Converged, solver_status.exit_status());
        unit_test_utils::assert_nearly_equal_array(
            &u,
            &[-5.0, -5.0, -1.0, -1.0, -1.0],
            1e-10,
            1e-10,
            "inner problem solution is wrong",
        );
    }
}
