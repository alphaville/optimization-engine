{% if activate_clib_generation -%}
// ---Export functionality from Rust to C/C++------------------------------------------------------------

/// Solver cache (structure `{{meta.optimizer_name}}Cache`)
///
#[allow(non_camel_case_types)]
#[no_mangle]
pub struct {{meta.optimizer_name}}Cache {
    cache: AlmCache,
}

impl {{meta.optimizer_name}}Cache {
    pub fn new(cache: AlmCache) -> Self {
        {{meta.optimizer_name}}Cache { cache }
    }
}

/// {{meta.optimizer_name}} version of ExitStatus
/// Structure: `{{meta.optimizer_name}}ExitStatus`
#[allow(non_camel_case_types)]
#[repr(C)]
#[no_mangle]
pub enum {{meta.optimizer_name}}ExitStatus {
    /// The algorithm has converged
    ///
    /// All termination criteria are satisfied and the algorithm
    /// converged within the available time and number of iterations
    {{meta.optimizer_name}}Converged,
    /// Failed to converge because the maximum number of iterations was reached
    {{meta.optimizer_name}}NotConvergedIterations,
    /// Failed to converge because the maximum execution time was reached
    {{meta.optimizer_name}}NotConvergedOutOfTime,
    /// If the gradient or cost function cannot be evaluated internally
    {{meta.optimizer_name}}NotConvergedCost,
    /// Computation failed and NaN/Infinite value was obtained
    {{meta.optimizer_name}}NotConvergedNotFiniteComputation,
}

/// {{meta.optimizer_name}} version of AlmOptimizerStatus
/// Structure: `{{meta.optimizer_name}}SolverStatus`
///
#[repr(C)]
#[no_mangle]
pub struct {{meta.optimizer_name}}SolverStatus {
    /// Exit status
    exit_status: {{meta.optimizer_name}}ExitStatus,
    /// Number of outer iterations
    num_outer_iterations: c_ulong,
    /// Total number of inner iterations
    ///
    /// This is the sum of the numbers of iterations of
    /// inner solvers
    num_inner_iterations: c_ulong,
    /// Norm of the fixed-point residual of the the problem
    last_problem_norm_fpr: c_double,
    /// Total solve time
    solve_time_ns: c_ulonglong,
    /// Penalty value
    penalty: c_double,
    /// Norm of delta y divided by the penalty parameter
    delta_y_norm_over_c: c_double,
    /// Norm of F2(u)
    f2_norm: c_double,
    /// Value of cost function at solution
    cost: c_double,
    /// Lagrange multipliers
    {%- if problem.dim_constraints_aug_lagrangian() > 0 %}
    lagrange: [c_double; {{meta.optimizer_name|upper}}_N1]
    {% else %}
    lagrange: *const c_double
    {% endif -%}
}

/// Allocate memory and setup the solver
#[no_mangle]
pub extern "C" fn {{meta.optimizer_name|lower}}_new() -> *mut {{meta.optimizer_name}}Cache {
    Box::into_raw(Box::new({{meta.optimizer_name}}Cache::new(initialize_solver())))
}

/// Solve the parametric optimization problem for a given parameter
/// .
/// .
/// Arguments:
/// - `instance`: re-useable instance of AlmCache, which should be created using
///   `{{meta.optimizer_name|lower}}_new` (and should be destroyed once it is not
///   needed using `{{meta.optimizer_name|lower}}_free`
/// - `u`: (on entry) initial guess of solution, (on exit) solution
///   (length: `{{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES`)
/// - `params`:  static parameters of the optimizer
///   (length: `{{meta.optimizer_name|upper}}_NUM_PARAMETERS`)
/// - `y0`: Initial guess of Lagrange multipliers (if `0`, the default will
///   be used; length: `{{meta.optimizer_name|upper}}_N1`)
/// - `c0`: Initial penalty parameter (provide `0` to use the default initial
///   penalty parameter
/// .
/// .
/// Returns:
/// Instance of `{{meta.optimizer_name}}SolverStatus`, with the solver status
/// (e.g., number of inner/outer iterations, measures of accuracy, solver time,
/// and the array of Lagrange multipliers at the solution).
#[no_mangle]
pub unsafe extern "C" fn {{meta.optimizer_name|lower}}_solve(
    instance: *mut {{meta.optimizer_name}}Cache,
    u: *mut c_double,
    params: *const c_double,
    y0: *const c_double,
    c0: *const c_double,
) -> {{meta.optimizer_name}}SolverStatus {

    // Convert all pointers into the required data structures
    let instance: &mut {{meta.optimizer_name}}Cache = {
        assert!(!instance.is_null());
        &mut *instance
    };

    // "*mut c_double" to "&mut [f64]"
    let u : &mut [f64] = {
        assert!(!u.is_null());
        std::slice::from_raw_parts_mut(u as *mut f64, {{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES)
    };

    // "*const c_double" to "&[f64]"
    let params : &[f64] = {
        assert!(!params.is_null());
        std::slice::from_raw_parts(params as *const f64, {{meta.optimizer_name|upper}}_NUM_PARAMETERS)
    };

    let c0_option: Option<f64> = if c0.is_null() {
        None::<f64>
    } else {
        Some(*c0)
    };

    let y0_option: Option<Vec<f64>> = if y0.is_null() {
        None::<Vec<f64>>
    } else {
        Some(std::slice::from_raw_parts(y0 as *mut f64, {{meta.optimizer_name|upper}}_N1).to_vec())
    };

    // Invoke `solve`
    let status = solve(params,&mut instance.cache, u, &y0_option, &c0_option);

    // Check solution status and cast it as `{{meta.optimizer_name}}SolverStatus`
    match status {
        Ok(status) => {{meta.optimizer_name}}SolverStatus {
            exit_status: match status.exit_status() {
                core::ExitStatus::Converged => {{meta.optimizer_name}}ExitStatus::{{meta.optimizer_name}}Converged,
                core::ExitStatus::NotConvergedIterations => {{meta.optimizer_name}}ExitStatus::{{meta.optimizer_name}}NotConvergedIterations,
                core::ExitStatus::NotConvergedOutOfTime => {{meta.optimizer_name}}ExitStatus::{{meta.optimizer_name}}NotConvergedOutOfTime,
            },
            num_outer_iterations: status.num_outer_iterations() as c_ulong,
            num_inner_iterations: status.num_inner_iterations() as c_ulong,
            last_problem_norm_fpr: status.last_problem_norm_fpr(),
            solve_time_ns: status.solve_time().as_nanos() as c_ulonglong,
            penalty: status.penalty() as c_double,
            delta_y_norm_over_c: status.delta_y_norm_over_c() as c_double,
            f2_norm: status.f2_norm() as c_double,
            cost: status.cost() as c_double,
            lagrange: match status.lagrange_multipliers() {
                Some({% if problem.dim_constraints_aug_lagrangian() == 0 %}_{% endif %}y) => {
                {%- if problem.dim_constraints_aug_lagrangian() > 0 %}
                    let mut y_array : [f64; {{meta.optimizer_name|upper}}_N1] = [0.0; {{meta.optimizer_name|upper}}_N1];
                    y_array.copy_from_slice(&y);
                    y_array
                {% else %}
                    std::ptr::null::<c_double>()
                {% endif %}
                },
                None => {
                {%- if problem.dim_constraints_aug_lagrangian() > 0 %}
                    [0.0; {{meta.optimizer_name|upper}}_N1]
                {% else %}
                    std::ptr::null::<c_double>()
                {% endif -%}
                }
            }
        },
        Err(e) => {{meta.optimizer_name}}SolverStatus {
            exit_status: match e {
                SolverError::Cost => {{meta.optimizer_name}}ExitStatus::{{meta.optimizer_name}}NotConvergedCost,
                SolverError::NotFiniteComputation => {{meta.optimizer_name}}ExitStatus::{{meta.optimizer_name}}NotConvergedNotFiniteComputation,
            },
            num_outer_iterations: std::u64::MAX as c_ulong,
            num_inner_iterations: std::u64::MAX as c_ulong,
            last_problem_norm_fpr: std::f64::INFINITY,
            solve_time_ns: std::u64::MAX as c_ulonglong,
            penalty: std::f64::INFINITY as c_double,
            delta_y_norm_over_c: std::f64::INFINITY as c_double,
            f2_norm: std::f64::INFINITY as c_double,
            cost: std::f64::INFINITY as c_double,
            lagrange: {%- if problem.dim_constraints_aug_lagrangian() > 0 -%}
                    [0.0; {{meta.optimizer_name|upper}}_N1]
                    {%- else -%}std::ptr::null::<c_double>(){%- endif %}
        },
    }
}

/// Deallocate the solver's memory, which has been previously allocated
/// using `{{meta.optimizer_name|lower}}_new`
#[no_mangle]
pub unsafe extern "C" fn {{meta.optimizer_name|lower}}_free(instance: *mut {{meta.optimizer_name}}Cache) {
    // Add impl
    assert!(!instance.is_null());
    Box::from_raw(instance);
}
{% endif %}