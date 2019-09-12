use crate::alm::*;
use crate::core::constraints;
use crate::core::panoc::*;
use crate::SolverError;

#[test]
fn t_create_alm_cache() {
    let tolerance = 1e-8;
    let nx = 10;
    let n1 = 5;
    let n2 = 0;
    let lbfgs_mem = 3;
    let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
    let alm_cache = AlmCache::new(panoc_cache, n1, n2);
    assert!(alm_cache.iteration == 0, "iter != 0");
    assert!(
        alm_cache.y.expect("No y allocated").len() == n1,
        "Wrong length (n1)"
    );
    assert!(
        alm_cache.y_plus.expect("No y_plus allocated").len() == n1,
        "Wrong length (n1)"
    );
    assert!(
        alm_cache.w_pm.is_none(),
        "Memory for w_pm should not have been allocated"
    );
}

#[test]
fn t_create_alm_problem() {
    let f = |_u: &[f64], _p: &[f64], _cost: &mut f64| -> Result<(), SolverError> { Ok(()) };
    let df = |_u: &[f64], _p: &[f64], _grad: &mut [f64]| -> Result<(), SolverError> { Ok(()) };

    {
        // Construct an instance of AlmProblem without any AL-type data
        let n1 = 0;
        let n2 = 0;
        let bounds = constraints::Ball2::new(None, 10.0);
        let _alm_problem = AlmProblem::new(
            bounds, NO_SET, NO_SET, f, df, NO_MAPPING, NO_MAPPING, n1, n2,
        );
    }

    {
        // Construct an AlmProblem with AL-type constraints, but no PM-type ones
        let f1 = |_u: &[f64], _f1up: &mut [f64]| -> Result<(), SolverError> { Ok(()) };
        let soc = constraints::SecondOrderCone::new(1.5);
        let bounds = constraints::Ball2::new(None, 10.0);
        let y_set = constraints::NoConstraints::new();
        let _alm_problem = AlmProblem::new(
            bounds,
            Some(soc),
            Some(y_set),
            f,
            df,
            Some(f1),
            NO_MAPPING,
            1,
            0,
        );
    }

    {
        // Construct an AlmProblem with both AL-type and PM-type constraints
        let f1 = |_u: &[f64], _f1up: &mut [f64]| -> Result<(), SolverError> { Ok(()) };
        let f2 = |_u: &[f64], _f2up: &mut [f64]| -> Result<(), SolverError> { Ok(()) };
        let soc = constraints::SecondOrderCone::new(1.5);
        let bounds = constraints::Ball2::new(None, 10.0);
        let y_set = constraints::NoConstraints::new();
        let _alm_problem = AlmProblem::new(
            bounds,
            Some(soc),
            Some(y_set),
            f,
            df,
            Some(f1),
            Some(f2),
            1,
            1,
        );
    }
}

#[should_panic]
#[test]
fn t_create_alm_problem_fail_alm() {
    let f = |_u: &[f64], _p: &[f64], _cost: &mut f64| -> Result<(), SolverError> { Ok(()) };
    let df = |_u: &[f64], _p: &[f64], _grad: &mut [f64]| -> Result<(), SolverError> { Ok(()) };
    let n1 = 1; // n1 = 1, but there is no set C
    let n2 = 0; // no F2 (and n2 = 0)
    let y_set = constraints::NoConstraints::new();
    let bounds = constraints::Ball2::new(None, 10.0);
    let _alm_problem = AlmProblem::new(
        bounds,
        NO_SET,
        Some(y_set),
        f,
        df,
        NO_MAPPING,
        NO_MAPPING,
        n1,
        n2,
    );
}

#[should_panic]
#[test]
fn t_create_alm_problem_fail_pm() {
    let f = |_u: &[f64], _p: &[f64], _cost: &mut f64| -> Result<(), SolverError> { Ok(()) };
    let df = |_u: &[f64], _p: &[f64], _grad: &mut [f64]| -> Result<(), SolverError> { Ok(()) };
    let f2 = |_u: &[f64], _f2up: &mut [f64]| -> Result<(), SolverError> { Ok(()) };
    let bounds = constraints::Ball2::new(None, 10.0);
    let n1 = 1;
    let n2 = 0; // there is an f2, but n2 = 0
    let _alm_problem = AlmProblem::new(bounds, NO_SET, NO_SET, f, df, NO_MAPPING, Some(f2), n1, n2);
}

#[test]
fn t_create_alm_optimizer() {
    let tolerance = 1e-8;
    let nx = 10;
    let n1 = 5;
    let n2 = 0;
    let lbfgs_mem = 3;
    let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
    let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);

    let f = |_u: &[f64], _p: &[f64], _cost: &mut f64| -> Result<(), SolverError> { Ok(()) };
    let df = |_u: &[f64], _p: &[f64], _grad: &mut [f64]| -> Result<(), SolverError> { Ok(()) };
    let f1 = |_u: &[f64], _grad: &mut [f64]| -> Result<(), SolverError> { Ok(()) };
    let set_c = constraints::Ball2::new(None, 1.50);

    // Construct an instance of AlmProblem without any AL-type data
    let bounds = constraints::Ball2::new(None, 10.0);
    let set_y = constraints::Ball2::new(None, 1.0);
    let alm_problem = AlmProblem::new(
        bounds,
        Some(set_c),
        Some(set_y),
        f,
        df,
        Some(f1),
        NO_MAPPING,
        n1,
        n2,
    );

    let mut alm_optimizer = AlmOptimizer::new(&mut alm_cache, alm_problem)
        .with_delta_tolerance(1e-4)
        .with_max_outer_iterations(10);
    alm_optimizer.set_lagrange_multipliers_init(&vec![5.0; n1]);

    println!(
        "result = {:?}",
        alm_optimizer.solve(&mut vec![0.0; 2], &vec![0.0; 2])
    );
}
