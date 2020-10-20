///
/// Auto-generated python bindings for optimizer: {{ meta.optimizer_name }}
/// Generated at: {{timestamp_created}}
///
use optimization_engine::alm::*;

use pyo3::prelude::*;
use pyo3::wrap_pyfunction;

use {{ meta.optimizer_name }}::*;

#[pymodule]
fn {{ meta.optimizer_name }}(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(solver, m)?)?;
    m.add_class::<OptimizerSolution>()?;
    m.add_class::<Solver>()?;
    Ok(())
}

#[pyfunction]
fn solver() -> PyResult<Solver> {
    let cache = initialize_solver();
    Ok(Solver { cache })
}

/// Solution and solution status of optimizer
#[pyclass]
struct OptimizerSolution {
    #[pyo3(get)]
    exit_status: String,
    #[pyo3(get)]
    num_outer_iterations: usize,
    #[pyo3(get)]
    num_inner_iterations: usize,
    #[pyo3(get)]
    last_problem_norm_fpr: f64,
    #[pyo3(get)]
    f1_infeasibility: f64,
    #[pyo3(get)]
    f2_norm: f64,
    #[pyo3(get)]
    solve_time_ms: f64,
    #[pyo3(get)]
    penalty: f64,
    #[pyo3(get)]
    solution: Vec<f64>,
    #[pyo3(get)]
    lagrange_multipliers: Vec<f64>,
    #[pyo3(get)]
    cost: f64,
}

#[pyclass]
struct Solver {
    cache: AlmCache,
}

#[pymethods]
impl Solver {
    /// Run solver
    ///
    #[text_signature = "($self, p, initial_guess, initial_y, initial_penalty)"]
    fn run(
        &mut self,
        p: Vec<f64>,
        initial_guess: Option<Vec<f64>>,
        initial_lagrange_multipliers: Option<Vec<f64>>,
        initial_penalty: Option<f64>,
    ) -> PyResult<Option<OptimizerSolution>> {
        let mut u = [0.0; {{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES];

        // ----------------------------------------------------
        // Set initial value
        // ----------------------------------------------------
        if let Some(u0) = initial_guess {
            if u0.len() != {{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES {
                println!(
                    "1600 -> Initial guess has incompatible dimensions: {} != {}",
                    u0.len(),
                    {{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES
                );
                return Ok(None);
            }
            u.copy_from_slice(&u0);
        }

        // ----------------------------------------------------
        // Check lagrange multipliers
        // ----------------------------------------------------
        if let Some(y0) = &initial_lagrange_multipliers {
            if y0.len() != {{meta.optimizer_name|upper}}_N1 {
                println!(
                    "1700 -> wrong dimension of Langrange multipliers: {} != {}",
                    y0.len(),
                    {{meta.optimizer_name|upper}}_N1
                );
                return Ok(None);
            }
        }

        // ----------------------------------------------------
        // Check parameter
        // ----------------------------------------------------
        if p.len() != {{meta.optimizer_name|upper}}_NUM_PARAMETERS {
            println!(
                "3003 -> wrong number of parameters: {} != {}",
                p.len(),
                {{meta.optimizer_name|upper}}_NUM_PARAMETERS
            );
            return Ok(None);
        }

        // ----------------------------------------------------
        // Run solver
        // ----------------------------------------------------
        let solver_status = solve(
            &p,
            &mut self.cache,
            &mut u,
            &initial_lagrange_multipliers,
            &initial_penalty,
        );

        match solver_status {
            Ok(status) => Ok(Some(OptimizerSolution {
                exit_status: format!("{:?}", status.exit_status()),
                num_outer_iterations: status.num_outer_iterations(),
                num_inner_iterations: status.num_inner_iterations(),
                last_problem_norm_fpr: status.last_problem_norm_fpr(),
                f1_infeasibility: status.delta_y_norm_over_c(),
                f2_norm: status.f2_norm(),
                penalty: status.penalty(),
                lagrange_multipliers: status.lagrange_multipliers().clone().unwrap_or_default(),
                solve_time_ms: (status.solve_time().as_nanos() as f64) / 1e6,
                solution: u.to_vec(),
                cost: status.cost(),
            })),
            Err(_) => {
                println!("2000 -> Problem solution failed (solver error)");
                Ok(None)
            }
        }
    }
}
