use icasadi;
use libc::{c_double, c_ulong, c_ulonglong};
use optimization_engine::{constraints::*, panoc::*, *};
use std::{num::NonZeroUsize, slice, time};

/// Opaque wrapper around PANOCCache, needed for cbindgen to generate a struct
pub struct PanocInstance {
    cache: panoc::PANOCCache,
    max_iterations: usize,
    max_solve_time: Option<time::Duration>,
}

impl PanocInstance {
    pub fn new(
        lbfgs_memory_size: usize,
        tolerance: f64,
        max_solve_time: Option<time::Duration>,
        max_iterations: usize,
    ) -> Self {
        assert!(max_iterations > 0);

        PanocInstance {
            cache: PANOCCache::new(
                NonZeroUsize::new(icasadi::NUM_DECISION_VARIABLES as usize).unwrap(),
                tolerance,
                NonZeroUsize::new(lbfgs_memory_size).unwrap(),
            ),
            max_iterations,
            max_solve_time,
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

/// Allocate memory and setup the solver
/// Note that `max_solve_time_ns` may be set to 0 if an infinite time in desired
#[no_mangle]
pub extern "C" fn panoc_new(
    lbfgs_memory_size: c_ulong,
    tolerance: c_double,
    max_solve_time_ns: c_ulonglong,
    max_iterations: c_ulong,
) -> *mut PanocInstance {
    let max_solve_time = if max_solve_time_ns > 0 {
        Some(time::Duration::from_nanos(max_solve_time_ns as u64))
    } else {
        None
    };

    Box::into_raw(Box::new(PanocInstance::new(
        lbfgs_memory_size as usize,
        tolerance as f64,
        max_solve_time,
        max_iterations as usize,
    )))
}

/// Run the solver on the input and parameters without constraints
#[no_mangle]
pub extern "C" fn panoc_solve_no_constraints(
    instance: *mut PanocInstance,
    u: *mut c_double,
    params: *const c_double,
) -> SolverStatus {
    let bounds = NoConstraints::new();

    panoc_solve_with_bound(instance, u, params, bounds)
}

/// Run the solver on the input and parameters without constraints
/// The `center`
#[no_mangle]
pub extern "C" fn panoc_solve_with_ball2_constraints(
    instance: *mut PanocInstance,
    u: *mut c_double,
    params: *const c_double,
    center: *const c_double,
    radius: c_double,
) -> SolverStatus {
    let center = unsafe {
        if center.is_null() {
            None
        } else {
            Some(slice::from_raw_parts(
                center as *const f64,
                icasadi::NUM_DECISION_VARIABLES as usize,
            ))
        }
    };

    let bounds = Ball2::new(center, radius as f64);

    panoc_solve_with_bound(instance, u, params, bounds)
}

/// Run the solver on the input and parameters with rectangle constraints
/// xmin
#[no_mangle]
pub extern "C" fn panoc_solve_with_rectangle_constraints(
    instance: *mut PanocInstance,
    u: *mut c_double,
    params: *const c_double,
    xmin: *const c_double,
    xmax: *const c_double,
) -> SolverStatus {
    let xmin = unsafe {
        if xmin.is_null() {
            None
        } else {
            Some(slice::from_raw_parts(
                xmin as *const f64,
                icasadi::NUM_DECISION_VARIABLES as usize,
            ))
        }
    };

    let xmax = unsafe {
        if xmax.is_null() {
            None
        } else {
            Some(slice::from_raw_parts(
                xmax as *const f64,
                icasadi::NUM_DECISION_VARIABLES as usize,
            ))
        }
    };

    let bounds = Rectangle::new(xmin, xmax);

    panoc_solve_with_bound(instance, u, params, bounds)
}

/// Generic solve method over constraints
fn panoc_solve_with_bound<ConstraintType: Constraint>(
    instance: *mut PanocInstance,
    u_ptr: *mut c_double,
    params_ptr: *const c_double,
    bounds: ConstraintType,
) -> SolverStatus {
    let instance: &mut PanocInstance = unsafe {
        assert!(!instance.is_null());
        &mut *instance
    };

    let mut u = unsafe {
        assert!(!u_ptr.is_null());
        //slice::from_raw_parts_mut(u_ptr as *mut f64, PROBLEM_SIZE)
        slice::from_raw_parts_mut(u_ptr as *mut f64, icasadi::NUM_DECISION_VARIABLES as usize)
    };

    let params = unsafe {
        assert!(!params_ptr.is_null());
        //slice::from_raw_parts(params_ptr as *const f64, PARAMETERS_SIZE)
        slice::from_raw_parts(
            params_ptr as *mut f64,
            icasadi::NUM_STATIC_PARAMETERS as usize,
        )
    };

    let df = |u: &[f64], grad: &mut [f64]| -> Result<(), Error> {
        if icasadi::icasadi_grad(u, &params, grad) == 0 {
            Ok(())
        } else {
            Err(Error::Cost)
        }
    };

    let f = |u: &[f64], c: &mut f64| -> Result<(), Error> {
        if icasadi::icasadi_cost(u, &params, c) == 0 {
            Ok(())
        } else {
            Err(Error::Cost)
        }
    };

    let problem = Problem::new(bounds, df, f);

    // Create PANOC
    let mut panoc = if let Some(dur) = instance.max_solve_time {
        PANOCOptimizer::new(problem, &mut instance.cache)
            .with_max_iter(instance.max_iterations)
            .with_max_duration(dur)
    } else {
        PANOCOptimizer::new(problem, &mut instance.cache).with_max_iter(instance.max_iterations)
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
