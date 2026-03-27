///
/// Auto-generated python bindings for optimizer: {{ meta.optimizer_name }}
///
use optimization_engine::alm::*;

use pyo3::class::basic::PyObjectProtocol;
use pyo3::prelude::*;
use pyo3::wrap_pyfunction;

use {{ meta.optimizer_name }}::*;

#[pymodule]
fn {{ meta.optimizer_name }}(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(solver, m)?)?;
    m.add_class::<SolverStatus>()?;
    m.add_class::<SolverError>()?;
    m.add_class::<SolverResponse>()?;
    m.add_class::<Solver>()?;
    m.add("OptimizerSolution", m.getattr("SolverStatus")?)?;
    Ok(())
}

#[pyfunction]
fn solver() -> PyResult<Solver> {
    let cache = initialize_solver();
    Ok(Solver { cache })
}

#[derive(Clone)]
struct SolverStatusData {
    exit_status: String,
    num_outer_iterations: usize,
    num_inner_iterations: usize,
    last_problem_norm_fpr: f64,
    f1_infeasibility: f64,
    f2_norm: f64,
    solve_time_ms: f64,
    penalty: f64,
    solution: Vec<f64>,
    lagrange_multipliers: Vec<f64>,
    cost: f64,
}

impl SolverStatusData {
    fn from_status(status: AlmOptimizerStatus, solution: &[f64]) -> Self {
        SolverStatusData {
            exit_status: format!("{:?}", status.exit_status()),
            num_outer_iterations: status.num_outer_iterations(),
            num_inner_iterations: status.num_inner_iterations(),
            last_problem_norm_fpr: status.last_problem_norm_fpr(),
            f1_infeasibility: status.delta_y_norm_over_c(),
            f2_norm: status.f2_norm(),
            penalty: status.penalty(),
            lagrange_multipliers: status.lagrange_multipliers().clone().unwrap_or_default(),
            solve_time_ms: (status.solve_time().as_nanos() as f64) / 1e6,
            solution: solution.to_vec(),
            cost: status.cost(),
        }
    }
}

#[derive(Clone)]
struct SolverErrorData {
    code: i32,
    message: String,
}

enum SolverResponsePayload {
    Ok(SolverStatusData),
    Err(SolverErrorData),
}

/// Solution and solution status of optimizer
#[pyclass]
struct SolverStatus {
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

impl From<SolverStatusData> for SolverStatus {
    fn from(status: SolverStatusData) -> Self {
        SolverStatus {
            exit_status: status.exit_status,
            num_outer_iterations: status.num_outer_iterations,
            num_inner_iterations: status.num_inner_iterations,
            last_problem_norm_fpr: status.last_problem_norm_fpr,
            f1_infeasibility: status.f1_infeasibility,
            f2_norm: status.f2_norm,
            solve_time_ms: status.solve_time_ms,
            penalty: status.penalty,
            solution: status.solution,
            lagrange_multipliers: status.lagrange_multipliers,
            cost: status.cost,
        }
    }
}

#[pyproto]
impl PyObjectProtocol for SolverStatus {
    fn __repr__(&self) -> PyResult<String> {
        Ok(format!(
            "SolverStatus(exit_status={:?}, num_outer_iterations={}, num_inner_iterations={}, last_problem_norm_fpr={}, f1_infeasibility={}, f2_norm={}, solve_time_ms={}, penalty={}, cost={})",
            self.exit_status,
            self.num_outer_iterations,
            self.num_inner_iterations,
            self.last_problem_norm_fpr,
            self.f1_infeasibility,
            self.f2_norm,
            self.solve_time_ms,
            self.penalty,
            self.cost
        ))
    }
}

#[pyclass]
struct SolverError {
    #[pyo3(get)]
    code: i32,
    #[pyo3(get)]
    message: String,
}

impl From<SolverErrorData> for SolverError {
    fn from(error: SolverErrorData) -> Self {
        SolverError {
            code: error.code,
            message: error.message,
        }
    }
}

#[pyproto]
impl PyObjectProtocol for SolverError {
    fn __repr__(&self) -> PyResult<String> {
        Ok(format!(
            "SolverError(code={}, message={:?})",
            self.code,
            self.message
        ))
    }
}

#[pyclass]
struct SolverResponse {
    payload: SolverResponsePayload,
}

#[pyproto]
impl PyObjectProtocol for SolverResponse {
    fn __repr__(&self) -> PyResult<String> {
        match &self.payload {
            SolverResponsePayload::Ok(status) => Ok(format!(
                "SolverResponse(ok=True, exit_status={:?}, num_outer_iterations={}, num_inner_iterations={})",
                status.exit_status,
                status.num_outer_iterations,
                status.num_inner_iterations
            )),
            SolverResponsePayload::Err(error) => Ok(format!(
                "SolverResponse(ok=False, code={}, message={:?})",
                error.code,
                error.message
            )),
        }
    }
}

#[pymethods]
impl SolverResponse {
    fn is_ok(&self) -> bool {
        matches!(self.payload, SolverResponsePayload::Ok(_))
    }

    fn get(&self, py: Python<'_>) -> PyResult<PyObject> {
        match &self.payload {
            SolverResponsePayload::Ok(status) => {
                Ok(Py::new(py, SolverStatus::from(status.clone()))?.into_py(py))
            }
            SolverResponsePayload::Err(error) => {
                Ok(Py::new(py, SolverError::from(error.clone()))?.into_py(py))
            }
        }
    }
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
    ) -> PyResult<SolverResponse> {
        let mut u = [0.0; {{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES];

        // ----------------------------------------------------
        // Set initial value
        // ----------------------------------------------------
        if let Some(u0) = initial_guess {
            if u0.len() != {{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES {
                return Ok(SolverResponse {
                    payload: SolverResponsePayload::Err(SolverErrorData {
                        code: 1600,
                        message: format!(
                            "initial guess has incompatible dimensions: provided {}, expected {}",
                            u0.len(),
                            {{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES
                        ),
                    }),
                });
            }
            u.copy_from_slice(&u0);
        }

        // ----------------------------------------------------
        // Check lagrange multipliers
        // ----------------------------------------------------
        if let Some(y0) = &initial_lagrange_multipliers {
            if y0.len() != {{meta.optimizer_name|upper}}_N1 {
                return Ok(SolverResponse {
                    payload: SolverResponsePayload::Err(SolverErrorData {
                        code: 1700,
                        message: format!(
                            "wrong dimension of Langrange multipliers: provided {}, expected {}",
                            y0.len(),
                            {{meta.optimizer_name|upper}}_N1
                        ),
                    }),
                });
            }
        }

        // ----------------------------------------------------
        // Check parameter
        // ----------------------------------------------------
        if p.len() != {{meta.optimizer_name|upper}}_NUM_PARAMETERS {
            return Ok(SolverResponse {
                payload: SolverResponsePayload::Err(SolverErrorData {
                    code: 3003,
                    message: format!(
                        "wrong number of parameters: provided {}, expected {}",
                        p.len(),
                        {{meta.optimizer_name|upper}}_NUM_PARAMETERS
                    ),
                }),
            });
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
            Ok(status) => Ok(SolverResponse {
                payload: SolverResponsePayload::Ok(SolverStatusData::from_status(status, &u)),
            }),
            Err(err) => Ok(SolverResponse {
                payload: SolverResponsePayload::Err(SolverErrorData {
                    code: 2000,
                    message: format!("problem solution failed: {}", err),
                }),
            }),
        }
    }
}
