//use icasadi;
use libc::{c_double, c_ulonglong};
use optimization_engine::{constraints::*, panoc::*, *};
use std::{num::NonZeroUsize, slice, time::Duration};

const PROBLEM_SIZE: usize = 10;
// const PARAMETERS_SIZE: usize = 20;
const LBFGS_MEMORY_SIZE: usize = 10;
const MAX_ITERATIONS: usize = 100;
const MAX_DURATION_NS: u64 = 1_000_000_000;
const TOLERANCE: f64 = 1e-4;

/// Opaque wrapper around PANOCCache, needed for cbindgen to generate a struct
pub struct PanocInstance {
    cache: panoc::PANOCCache,
}

impl PanocInstance {
    pub fn new() -> Self {
        PanocInstance {
            cache: PANOCCache::new(
                NonZeroUsize::new(PROBLEM_SIZE).unwrap(),
                //NonZeroUsize::new(icasadi::num_decision_variables()).unwrap(),
                TOLERANCE,
                NonZeroUsize::new(LBFGS_MEMORY_SIZE).unwrap(),
            ),
        }
    }
}

/// C version of SolverStatus
#[repr(C)]
pub struct SolverStatus {
    /// number of iterations for convergence
    num_iter: c_ulonglong,
    /// time it took to solve
    solve_time_ns: c_ulonglong,
    /// norm of the fixed-point residual (FPR)
    fpr_norm: c_double,
    /// cost value at the candidate solution
    cost_value: c_double,
}

//
// Example cost function, can be from icasadi just as well
//
fn rosenbrock_cost(a: f64, b: f64, u: &[f64]) -> f64 {
    (a - u[0]).powi(2) + b * (u[1] - u[0].powi(2)).powi(2)
}

fn rosenbrock_grad(a: f64, b: f64, u: &[f64], grad: &mut [f64]) {
    grad[0] = 2.0 * u[0] - 2.0 * a - 4.0 * b * u[0] * (-u[0].powi(2) + u[1]);
    grad[1] = b * (-2.0 * u[0].powi(2) + 2.0 * u[1]);
}

/// Allocate memory for the solver
#[no_mangle]
pub extern "C" fn panoc_new() -> *mut PanocInstance {
    // Add impl
    Box::into_raw(Box::new(PanocInstance::new()))
}

/// Run the solver on the input and parameters
#[no_mangle]
pub extern "C" fn panoc_solve(instance: *mut PanocInstance, u_ptr: *mut c_double) -> SolverStatus {
    let cache: &mut PANOCCache = unsafe {
        assert!(!instance.is_null());
        &mut (&mut *instance).cache
    };

    let mut u = unsafe {
        assert!(!u_ptr.is_null());
        slice::from_raw_parts_mut(u_ptr as *mut f64, PROBLEM_SIZE)
        //slice::from_raw_parts_mut(u_ptr as *mut f64, icasadi::num_decision_variables())
    };

    // let mut params = unsafe {
    //     assert!(!params_ptr.is_null());
    //     slice::from_raw_parts_mut(params_ptr as *mut f64, PARAMETERS_SIZE)
    //     slice::from_raw_parts_mut(params_ptr as *mut f64, icasadi::num_static_parameters())
    // };

    // let df = |u: &[f64], grad: &mut [f64]| -> Result<(), Error> {
    //     if icasadi::icasadi_grad(u, &params, grad) == 0 {
    //         Ok(())
    //     } else {
    //         Err(Error::Cost)
    //     }
    // };

    // let f = |u: &[f64], c: &mut f64| -> Result<(), Error> {
    //     if icasadi::icasadi_cost(u, &params, c) == 0 {
    //         Ok(())
    //     } else {
    //         Err(Error::Cost)
    //     }
    // };

    // define the cost function and its gradient
    let a = 1.0;
    let b = 200.0;
    let radius = 1.0;

    let df = |u: &[f64], grad: &mut [f64]| -> Result<(), Error> {
        rosenbrock_grad(a, b, u, grad);
        Ok(())
    };

    let f = |u: &[f64], c: &mut f64| -> Result<(), Error> {
        *c = rosenbrock_cost(a, b, u);
        Ok(())
    };

    // Danger danger, allocation!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // All constraints internally use Vec<_> we should probably change that
    // ------------------------------------------------------------------------------------------
    let bounds = Ball2::new_at_origin_with_radius(radius);
    // ------------------------------------------------------------------------------------------

    let problem = Problem::new(bounds, df, f);

    // Create PANOC
    let mut panoc = if MAX_DURATION_NS > 0 {
        PANOCOptimizer::new(problem, cache)
            .with_max_iter(MAX_ITERATIONS)
            .with_max_duration(Duration::from_nanos(MAX_DURATION_NS))
    } else {
        PANOCOptimizer::new(problem, cache).with_max_iter(MAX_ITERATIONS)
    };

    // Invoke the solver
    let status = panoc.solve(&mut u);

    SolverStatus {
        num_iter: status.iterations() as c_ulonglong,
        solve_time_ns: status.solve_time().as_nanos() as c_ulonglong,
        fpr_norm: status.norm_fpr() as c_double,
        cost_value: status.cost_value() as c_double,
    }
}

/// Deallocate the solver's memory
#[no_mangle]
pub extern "C" fn panoc_free(instance: *mut PanocInstance) {
    // Add impl
    unsafe {
        assert!(!instance.is_null());
        Box::from_raw(instance);
    }
}
