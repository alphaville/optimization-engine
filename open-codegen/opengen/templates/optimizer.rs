//
// Auto-generated file by OptimizationEngine
// See https://alphaville.github.io/optimization-engine/
//
//

{% if activate_clib_generation -%}
use libc::{c_double, c_ulong, c_ulonglong};
{% endif %}
use optimization_engine::{constraints::*, panoc::*, alm::*, *};

// ---Private Constants----------------------------------------------------------------------------------

/// Tolerance of inner solver
const EPSILON_TOLERANCE: f64 = {{solver_config.tolerance or 0.0001}};

/// Initial tolerance
const INITIAL_EPSILON_TOLERANCE: f64 = {{solver_config.initial_tolerance or 0.0001}};

/// Update factor for inner tolerance
const EPSILON_TOLERANCE_UPDATE_FACTOR: f64 = {{solver_config.inner_tolerance_update_factor or 0.1}};

/// Delta tolerance
const DELTA_TOLERANCE: f64 = {{solver_config.constraints_tolerance or 0.0001}};

/// LBFGS memory
const LBFGS_MEMORY: usize = {{solver_config.lbfgs_memory or 10}};

/// Maximum number of iterations of the inner solver
const MAX_INNER_ITERATIONS: usize = {{solver_config.max_inner_iterations or 10000}};

/// Maximum number of outer iterations
const MAX_OUTER_ITERATIONS: usize = {{solver_config.max_outer_iterations or 10}};

/// Maximum execution duration in microseconds
const MAX_DURATION_MICROS: u64 = {{solver_config.max_duration_micros}};

/// Penalty update factor
const PENALTY_UPDATE_FACTOR: f64 = {{solver_config.penalty_weight_update_factor or 10.0}};

/// Initial penalty
const INITIAL_PENALTY_PARAMETER: Option<f64> = {% if solver_config.initial_penalty  is not none %}Some({{ solver_config.initial_penalty }}){% else %}None{% endif %};

/// Sufficient decrease coefficient
const SUFFICIENT_INFEASIBILITY_DECREASE_COEFFICIENT: f64 = {{solver_config.sufficient_decrease_coefficient or 0.1}};

/// Whether preconditioning should be applied
const DO_PRECONDITIONING: bool = {{ solver_config.preconditioning | lower }};

// ---Public Constants-----------------------------------------------------------------------------------

/// Number of decision variables
pub const {{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES: usize = {{problem.dim_decision_variables()}};

/// Number of parameters
pub const {{meta.optimizer_name|upper}}_NUM_PARAMETERS: usize = {{problem.dim_parameters()}};

/// Number of parameters associated with augmented Lagrangian
pub const {{meta.optimizer_name|upper}}_N1: usize = {{problem.dim_constraints_aug_lagrangian()}};

/// Number of penalty constraints
pub const {{meta.optimizer_name|upper}}_N2: usize = {{problem.dim_constraints_penalty() or 0}};

{% include "c/optimizer_cinterface.rs" %}

// ---Parameters of the constraints----------------------------------------------------------------------

{% if 'Ball1' == problem.constraints.__class__.__name__ or 'Ball2' == problem.constraints.__class__.__name__ or 'BallInf' == problem.constraints.__class__.__name__ or 'Sphere2' == problem.constraints.__class__.__name__ -%}
/// Constraints: Centre of Ball
const CONSTRAINTS_BALL_XC: Option<&[f64]> = {% if problem.constraints.center is not none %}Some(&[{{problem.constraints.center | join(', ')}}]){% else %}None{% endif %};

/// Constraints: Radius of Ball
const CONSTRAINTS_BALL_RADIUS : f64 = {{problem.constraints.radius}};
{% elif 'Rectangle' == problem.constraints.__class__.__name__ -%}
const CONSTRAINTS_XMIN :Option<&[f64]> = {% if problem.constraints.xmin is not none %}Some(&[
{%- for xmini in problem.constraints.xmin -%}
{%- if float('-inf') == xmini -%}std::f64::NEG_INFINITY{%- else -%}{{xmini}}{%- endif -%},
{%- endfor -%}
]){% else %}None{% endif %};
const CONSTRAINTS_XMAX :Option<&[f64]> = {% if problem.constraints.xmax is not none %}Some(&[
{%- for xmaxi in problem.constraints.xmax -%}
{%- if float('inf') == xmaxi -%}std::f64::INFINITY{%- else -%}{{xmaxi}}{%- endif -%},
{%- endfor -%}
]){% else %}None{% endif %};
{% endif %}



{% if problem.alm_set_c is not none %}
// ---Parameters of ALM-type constraints (Set C)---------------------------------------------------------
{% if 'Ball2' == problem.alm_set_c.__class__.__name__ or 'BallInf' == problem.alm_set_c.__class__.__name__ -%}
/// Constraints: Centre of Euclidean Ball
const SET_C_BALL_XC: Option<&[f64]> = {% if problem.alm_set_c.center is not none %}Some(&[{{problem.alm_set_c.center | join(', ')}}]){% else %}None{% endif %};

/// Constraints: Radius of Euclidean Ball
const SET_C_BALL_RADIUS : f64 = {{problem.alm_set_c.radius}};
{% elif 'Rectangle' == problem.alm_set_c.__class__.__name__ -%}
const SET_C_XMIN :Option<&[f64]> = {% if problem.alm_set_c.xmin is not none %}Some(&[
{%- for xmini in problem.alm_set_c.xmin -%}
{%- if float('-inf') == xmini -%}std::f64::NEG_INFINITY{%- else -%}{{xmini}}{%- endif -%},
{%- endfor -%}
]){% else %}None{% endif %};
const SET_C_XMAX :Option<&[f64]> = {% if problem.alm_set_c.xmax is not none %}Some(&[
{%- for xmaxi in problem.alm_set_c.xmax -%}
{%- if float('inf') == xmaxi -%}std::f64::INFINITY{%- else -%}{{xmaxi}}{%- endif -%},
{%- endfor -%}
]){% else %}None{% endif %};
{% endif %}
{% endif %}


{% if problem.alm_set_y is not none -%}
// ---Parameters of ALM-type constraints (Set Y)---------------------------------------------------------
{% if 'Ball1' == problem.alm_set_y.__class__.__name__ or 'Ball2' == problem.alm_set_y.__class__.__name__ or 'BallInf' == problem.alm_set_y.__class__.__name__ -%}
/// Constraints: Centre of Euclidean Ball
const SET_Y_BALL_XC: Option<&[f64]> = {% if problem.alm_set_y.center is not none %}Some(&[{{problem.alm_set_y.center | join(', ')}}]){% else %}None{% endif %};

/// Constraints: Radius of Euclidean Ball
const SET_Y_BALL_RADIUS : f64 = {{problem.alm_set_y.radius}};
{% elif 'Rectangle' == problem.alm_set_y.__class__.__name__ -%}
/// Y_min
const SET_Y_XMIN :Option<&[f64]> = {% if problem.alm_set_y.xmin is not none %}Some(&[{{problem.alm_set_y.xmin|join(', ')}}]){% else %}None{% endif %};

/// Y_max
const SET_Y_XMAX :Option<&[f64]> = {% if problem.alm_set_y.xmax is not none %}Some(&[{{problem.alm_set_y.xmax|join(', ')}}]){% else %}None{% endif %};
{% endif %}
{% endif %}


// ---Internal private helper functions------------------------------------------------------------------

/// Make constraints U
fn make_constraints() -> impl Constraint {
    {% if 'Ball2' == problem.constraints.__class__.__name__ -%}
    // - Euclidean ball:
    Ball2::new(CONSTRAINTS_BALL_XC, CONSTRAINTS_BALL_RADIUS)
    {% elif 'BallInf' == problem.constraints.__class__.__name__ -%}
    // - Infinity ball:
    BallInf::new(CONSTRAINTS_BALL_XC, CONSTRAINTS_BALL_RADIUS)
    {% elif 'Ball1' == problem.constraints.__class__.__name__ -%}
    // - Ball1:
    Ball1::new(CONSTRAINTS_BALL_XC, CONSTRAINTS_BALL_RADIUS)
    {% elif 'Sphere2' == problem.constraints.__class__.__name__ -%}
    // - Sphere2:
    Sphere2::new(CONSTRAINTS_BALL_XC, CONSTRAINTS_BALL_RADIUS)
    {% elif 'Simplex' == problem.constraints.__class__.__name__ -%}
    // - Simplex:
    let alpha_simplex : f64 = {{problem.constraints.alpha}};
    Simplex::new(alpha_simplex)
    {% elif 'Rectangle' == problem.constraints.__class__.__name__ -%}
    // - Rectangle:
    Rectangle::new(CONSTRAINTS_XMIN, CONSTRAINTS_XMAX)
    {% elif 'FiniteSet' == problem.constraints.__class__.__name__ -%}
    // - Finite Set:
    let data: &[&[f64]] = &[
    {% for point in problem.constraints.points %}&[{{point|join(', ')}}],{% endfor %}
    ];
    FiniteSet::new(data)
    {% elif 'Halfspace' == problem.constraints.__class__.__name__ -%}
    // - Halfspace:
    let offset: f64 = {{problem.constraints.offset}};
    let normal_vector: &[f64] = &[{{problem.constraints.normal_vector | join(', ')}}];
    Halfspace::new(normal_vector, offset)
    {% elif 'NoConstraints' == problem.constraints.__class__.__name__ -%}
    // - No constraints (whole Rn):
    NoConstraints::new()
    {% elif 'Zero' == problem.constraints.__class__.__name__ -%}
    // - Zero!
    Zero::new()
    {% elif 'CartesianProduct' == problem.constraints.__class__.__name__ -%}
        // - Cartesian product of constraints:
        let bounds = CartesianProduct::new();
        {% for set_i in problem.constraints.constraints %}
        let idx_{{loop.index}} = {{problem.constraints.segments[loop.index-1]+1}};
        {% if 'Ball2' == set_i.__class__.__name__ -%}
        let radius_{{loop.index}} = {{set_i.radius}};
        let center_{{loop.index}}: Option<&[f64]> = {% if set_i.center is not none %}Some(&[{{set_i.center | join(', ')}}]){% else %}None{% endif %};
        let set_{{loop.index}} = Ball2::new(center_{{loop.index}}, radius_{{loop.index}});
        let bounds = bounds.add_constraint(idx_{{loop.index}}, set_{{loop.index}});        
        {% elif 'BallInf' == set_i.__class__.__name__ -%}
        let radius_{{loop.index}} = {{set_i.radius}};
        let center_{{loop.index}}: Option<&[f64]> = {% if set_i.center is not none %}Some(&[{{set_i.center | join(', ')}}]){% else %}None{% endif %};
        let set_{{loop.index}} = BallInf::new(center_{{loop.index}}, radius_{{loop.index}});
        let bounds = bounds.add_constraint(idx_{{loop.index}}, set_{{loop.index}});
        {% elif 'Ball1' == set_i.__class__.__name__ -%}
        let radius_{{loop.index}} = {{set_i.radius}};
        let center_{{loop.index}}: Option<&[f64]> = {% if set_i.center is not none %}Some(&[{{set_i.center | join(', ')}}]){% else %}None{% endif %};
        let set_{{loop.index}} = Ball1::new(center_{{loop.index}}, radius_{{loop.index}});
        let bounds = bounds.add_constraint(idx_{{loop.index}}, set_{{loop.index}});
        {% elif 'Sphere2' == set_i.__class__.__name__ -%}
        let radius_{{loop.index}} = {{set_i.radius}};
        let center_{{loop.index}}: Option<&[f64]> = {% if set_i.center is not none %}Some(&[{{set_i.center | join(', ')}}]){% else %}None{% endif %};
        let set_{{loop.index}} = Sphere2::new(center_{{loop.index}}, radius_{{loop.index}});
        let bounds = bounds.add_constraint(idx_{{loop.index}}, set_{{loop.index}});
        {% elif 'Simplex' == set_i.__class__.__name__ -%}
        let alpha_{{loop.index}} = {{set_i.alpha}};        
        let set_{{loop.index}} = Simplex::new(alpha_{{loop.index}});
        let bounds = bounds.add_constraint(idx_{{loop.index}}, set_{{loop.index}});
        {% elif 'Rectangle' == set_i.__class__.__name__ -%}
        let xmin_{{loop.index}} :Option<&[f64]> = {% if set_i.xmin is not none %}Some(&[
        {%- for xmini in set_i.xmin -%}
        {%- if float('-inf') == xmini -%}std::f64::NEG_INFINITY{%- else -%}{{xmini}}{%- endif -%},
        {%- endfor -%}
        ]){% else %}None{% endif %};
        let xmax_{{loop.index}}:Option<&[f64]> = {% if set_i.xmax is not none %}Some(&[
        {%- for xmaxi in set_i.xmax -%}
        {%- if float('inf') == xmaxi -%}std::f64::INFINITY{%- else -%}{{xmaxi}}{%- endif -%},
        {%- endfor -%}
        ]){% else %}None{% endif %};
        let set_{{loop.index}} = Rectangle::new(xmin_{{loop.index}}, xmax_{{loop.index}});
        let bounds = bounds.add_constraint(idx_{{loop.index}}, set_{{loop.index}});
        {% elif 'FiniteSet' == set_i.__class__.__name__ -%}
        let data_{{loop.index}}: &[&[f64]] = &[{% for point in set_i.points %}&[{{point|join(', ')}}],{% endfor %}];
        let set_{{loop.index}} = FiniteSet::new(data_{{loop.index}});
        let bounds = bounds.add_constraint(idx_{{loop.index}}, set_{{loop.index}});
        {% elif 'Halfspace' == set_i.__class__.__name__ -%}
        let normal_vector_{{loop.index}} = &[{{set_i.normal_vector | join(', ')}}];
        let offset_{{loop.index}} = {{ set_i.offset }};
        let bounds = bounds.add_constraint(idx_{{loop.index}}, Halfspace::new(normal_vector_{{loop.index}}, offset_{{loop.index}}));
        {% elif 'NoConstraints' == set_i.__class__.__name__ -%}
        let bounds = bounds.add_constraint(idx_{{loop.index}}, NoConstraints::new());
        {% elif 'Zero' == set_i.__class__.__name__ -%}
        let bounds = bounds.add_constraint(idx_{{loop.index}}, Zero::new());
        {% endif -%}
    {% endfor %}
    bounds
    {% endif -%}
}

{% if problem.alm_set_c is not none -%}
/// Make set C
fn make_set_c() -> impl Constraint {
    {% if 'Ball2' == problem.alm_set_c.__class__.__name__ -%}
    Ball2::new(SET_C_BALL_XC, SET_C_BALL_RADIUS)
    {% elif 'BallInf' == problem.alm_set_c.__class__.__name__ -%}
    BallInf::new(SET_C_BALL_XC, SET_C_BALL_RADIUS)
    {% elif 'Ball1' == problem.alm_set_c.__class__.__name__ -%}
    Ball1::new(SET_C_BALL_XC, SET_C_BALL_RADIUS)
    {% elif 'Simplex' == problem.alm_set_c.__class__.__name__ -%}
    let set_c_simplex_alpha : f64 = {{problem.alm_set_y.alpha}};
    Simplex::new(set_c_simplex_alpha)
    {% elif 'Rectangle' == problem.alm_set_c.__class__.__name__ -%}
    Rectangle::new(SET_C_XMIN, SET_C_XMAX)
    {% elif 'NoConstraints' == problem.alm_set_c.__class__.__name__ -%}
    NoConstraints::new()
    {% elif 'Zero' == problem.alm_set_c.__class__.__name__ -%}
    Zero::new()
    {% elif 'CartesianProduct' == problem.alm_set_c.__class__.__name__ -%}
        // Cartesian product of constraints (Set C)
        let set_c = CartesianProduct::new();
        {% for set_i in problem.alm_set_c.constraints %}
            // Set type: {{ set_i.__class__.__name__ }}
            let idx_{{loop.index}} = {{problem.alm_set_c.segments[loop.index-1]+1}};
            {% if 'Ball2' == set_i.__class__.__name__ -%}
            let radius_{{loop.index}} = {{set_i.radius}};
            let center_{{loop.index}}: Option<&[f64]> = {% if set_i.center is not none %}Some(&[{{set_i.center | join(', ')}}]){% else %}None{% endif %};
            let set_{{loop.index}} = Ball2::new(center_{{loop.index}}, radius_{{loop.index}});
            let set_c = set_c.add_constraint(idx_{{loop.index}}, set_{{loop.index}});
            {% elif 'BallInf' == set_i.__class__.__name__ -%}
            let radius_{{loop.index}} = {{set_i.radius}};
            let center_{{loop.index}}: Option<&[f64]> = {% if set_i.center is not none %}Some(&[{{set_i.center | join(', ')}}]){% else %}None{% endif %};
            let set_{{loop.index}} = BallInf::new(center_{{loop.index}}, radius_{{loop.index}});
            let set_c = set_c.add_constraint(idx_{{loop.index}}, set_{{loop.index}});
            {% elif 'Ball1' == set_i.__class__.__name__ -%}
            let radius_{{loop.index}} = {{set_i.radius}};
            let center_{{loop.index}}: Option<&[f64]> = {% if set_i.center is not none %}Some(&[{{set_i.center | join(', ')}}]){% else %}None{% endif %};
            let set_{{loop.index}} = Ball1::new(center_{{loop.index}}, radius_{{loop.index}});
            let set_c = set_c.add_constraint(idx_{{loop.index}}, set_{{loop.index}});
            {% elif 'Simplex' == set_i.__class__.__name__ -%}
            let alpha_smplx_{{loop.index}} = {{set_i.alpha}};
            let set_{{loop.index}} = Simplex::new(alpha_smplx_{{loop.index}});
            let set_c = set_c.add_constraint(idx_{{loop.index}}, set_{{loop.index}});
            {% elif 'Rectangle' == set_i.__class__.__name__ -%}
            let xmin_{{loop.index}} :Option<&[f64]> = {% if set_i.xmin is not none %}Some(&[
            {%- for xmini in set_i.xmin -%}
            {%- if float('-inf') == xmini -%}std::f64::NEG_INFINITY{%- else -%}{{xmini}}{%- endif -%},
            {%- endfor -%}
            ]){% else %}None{% endif %};
            let xmax_{{loop.index}}:Option<&[f64]> = {% if set_i.xmax is not none %}Some(&[
            {%- for xmaxi in set_i.xmax -%}
            {%- if float('inf') == xmaxi -%}std::f64::INFINITY{%- else -%}{{xmaxi}}{%- endif -%},
            {%- endfor -%}
            ]){% else %}None{% endif %};
            let set_{{loop.index}} = Rectangle::new(xmin_{{loop.index}}, xmax_{{loop.index}});
            let set_c = set_c.add_constraint(idx_{{loop.index}}, set_{{loop.index}});
            {% elif 'FiniteSet' == set_i.__class__.__name__ -%}
            let data_{{loop.index}}: &[&[f64]] = &[{% for point in set_i.points %}&[{{point|join(', ')}}],{% endfor %}];
            let set_{{loop.index}} = FiniteSet::new(data_{{loop.index}});
            let set_c = set_c.add_constraint(idx_{{loop.index}}, set_{{loop.index}});
            {% elif 'NoConstraints' == set_i.__class__.__name__ -%}
            let set_c = set_c.add_constraint(idx_{{loop.index}}, NoConstraints::new());
            {% elif 'Zero' == set_i.__class__.__name__ -%}
            let set_c = set_c.add_constraint(idx_{{loop.index}}, Zero::new());
            {% endif -%}
        {% endfor %}
    set_c
    {% endif -%}
}
{% endif %}

{% if problem.alm_set_y is not none -%}
/// Make set Y
fn make_set_y() -> impl Constraint {
    {% if 'Ball2' == problem.alm_set_y.__class__.__name__ -%}
    Ball2::new(SET_Y_BALL_XC, SET_Y_BALL_RADIUS)
    {% elif 'BallInf' == problem.alm_set_y.__class__.__name__ -%}
    BallInf::new(SET_Y_BALL_XC, SET_Y_BALL_RADIUS)
    {% elif 'Rectangle' == problem.alm_set_y.__class__.__name__ -%}
    Rectangle::new(SET_Y_XMIN, SET_Y_XMAX)
    {% elif 'NoConstraints' == problem.alm_set_y.__class__.__name__ -%}
    NoConstraints::new()
    {% elif 'Zero' == problem.alm_set_y.__class__.__name__ -%}
    Zero::new()
    {% endif -%}
}
{% endif %}

// ---Main public API functions--------------------------------------------------------------------------


/// Initialisation of the solver
pub fn initialize_solver() -> AlmCache {
    let panoc_cache = PANOCCache::new({{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES, EPSILON_TOLERANCE, LBFGS_MEMORY);
    {% if solver_config.cbfgs_alpha is not none and solver_config.cbfgs_epsilon is not none -%}
        let panoc_cache = panoc_cache.with_cbfgs_parameters({{solver_config.cbfgs_alpha}}, {{solver_config.cbfgs_epsilon}}, {{solver_config.cbfgs_sy_epsilon}});
    {% endif -%}
    AlmCache::new(panoc_cache, {{meta.optimizer_name|upper}}_N1, {{meta.optimizer_name|upper}}_N2)
}

/// If preconditioning has been applied, then at the end (after a solution has been obtained)
/// we need to undo the scaling and update the cost function
fn unscale_result(solver_status: &mut Result<AlmOptimizerStatus, SolverError>) {
    if let Ok(sss) = solver_status {
        let w_cost : f64 = icasadi_{{meta.optimizer_name}}::get_w_cost();
        sss.update_cost(sss.cost() / w_cost);
    }
}

/// Solver interface
///
/// ## Arguments
/// - `p`: static parameter vector of the optimization problem
/// - `alm_cache`: Instance of AlmCache
/// - `u`: Initial guess
/// - `y0` (optional) initial vector of Lagrange multipliers
/// - `c0` (optional) initial penalty
///
/// ## Returns
/// This function returns either an instance of AlmOptimizerStatus with information about the
/// solution, or a SolverError object if something goes wrong
pub fn solve(
    p: &[f64],
    alm_cache: &mut AlmCache,
    u: &mut [f64],
    y0: &Option<Vec<f64>>,
    c0: &Option<f64>,
) -> Result<AlmOptimizerStatus, SolverError> {

    assert_eq!(p.len(), {{meta.optimizer_name|upper}}_NUM_PARAMETERS, "Wrong number of parameters (p)");
    assert_eq!(u.len(), {{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES, "Wrong number of decision variables (u)");

    // Start by initialising the optimiser interface (e.g., set w=1)
    icasadi_{{meta.optimizer_name}}::init_{{ meta.optimizer_name }}();

    let mut rho_init : f64 = 1.0;
    if DO_PRECONDITIONING {
        // Compute the preconditioning parameters (w's)
        // The scaling parameters will be stored internally in `interface.c`
        icasadi_{{meta.optimizer_name}}::precondition(u, p);

        // Compute initial penalty
        icasadi_{{meta.optimizer_name}}::initial_penalty(u, p, & mut rho_init);
    }

    let psi = |u: &[f64], xi: &[f64], cost: &mut f64| -> Result<(), SolverError> {
        icasadi_{{meta.optimizer_name}}::cost(u, xi, p, cost);
        Ok(())
    };
    let grad_psi = |u: &[f64], xi: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
        icasadi_{{meta.optimizer_name}}::grad(u, xi, p, grad);
        Ok(())
    };
    {% if problem.dim_constraints_aug_lagrangian() > 0 %}
    let f1 = |u: &[f64], res: &mut [f64]| -> Result<(), SolverError> {
        icasadi_{{meta.optimizer_name}}::mapping_f1(u, p, res);
        Ok(())
    };{% endif %}
    {% if problem.dim_constraints_penalty() %}let f2 = |u: &[f64], res: &mut [f64]| -> Result<(), SolverError> {
        icasadi_{{meta.optimizer_name}}::mapping_f2(u, p, res);
        Ok(())
    };{% endif -%}
    let bounds = make_constraints();

    {% if problem.dim_constraints_aug_lagrangian() > 0 -%}
    let set_y = make_set_y();
    let set_c = make_set_c();
    {% endif -%}

    let alm_problem = AlmProblem::new(
        bounds,
        {% if problem.dim_constraints_aug_lagrangian() > 0 %}Some(set_c){% else %}NO_SET{% endif %},
        {% if problem.dim_constraints_aug_lagrangian() > 0 %}Some(set_y){% else %}NO_SET{% endif %},
        psi,
        grad_psi,
        {% if problem.dim_constraints_aug_lagrangian() > 0 %}Some(f1){% else %}NO_MAPPING{% endif %},
        {% if problem.dim_constraints_penalty() %}Some(f2){% else %}NO_MAPPING{% endif %},
        {{meta.optimizer_name|upper}}_N1,
        {{meta.optimizer_name|upper}}_N2,
    );

    let mut alm_optimizer = AlmOptimizer::new(alm_cache, alm_problem)
        .with_delta_tolerance(DELTA_TOLERANCE)
        .with_epsilon_tolerance(EPSILON_TOLERANCE)
        .with_initial_inner_tolerance(INITIAL_EPSILON_TOLERANCE)
        .with_inner_tolerance_update_factor(EPSILON_TOLERANCE_UPDATE_FACTOR)
        .with_max_duration(std::time::Duration::from_micros(MAX_DURATION_MICROS))
        .with_max_outer_iterations(MAX_OUTER_ITERATIONS)
        .with_max_inner_iterations(MAX_INNER_ITERATIONS)
        .with_initial_penalty(c0.unwrap_or(INITIAL_PENALTY_PARAMETER.unwrap_or(rho_init)))
        .with_penalty_update_factor(PENALTY_UPDATE_FACTOR)
        .with_sufficient_decrease_coefficient(SUFFICIENT_INFEASIBILITY_DECREASE_COEFFICIENT);

    // solve the problem using `u`, the initial condition `u`, and
    // initial vector of Lagrange multipliers, if provided;
    // returns the problem status (instance of `AlmOptimizerStatus`)
    if let Some(y0_) = y0 {
        let mut alm_optimizer = alm_optimizer.with_initial_lagrange_multipliers(y0_);
        let mut solution_status = alm_optimizer.solve(u);
        unscale_result(&mut solution_status);
        solution_status
    } else {
        let mut solution_status = alm_optimizer.solve(u);
        unscale_result(&mut solution_status);
        solution_status
    }

}
