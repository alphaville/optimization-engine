use crate::{
    alm::*,
    core::{constraints::*, panoc::*, ExitStatus},
    mocks, FunctionCallResult,
};

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
        alm_cache.xi.expect("No xi allocated").len() == n1 + 1,
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
    let f = |_u: &[f64], _p: &[f64], _cost: &mut f64| -> FunctionCallResult { Ok(()) };
    let df = |_u: &[f64], _p: &[f64], _grad: &mut [f64]| -> FunctionCallResult { Ok(()) };

    {
        // Construct an instance of AlmProblem without any AL-type data
        let n1 = 0;
        let n2 = 0;
        let bounds = Ball2::new(None, 10.0);
        let _alm_problem = AlmProblem::new(
            bounds, NO_SET, NO_SET, f, df, NO_MAPPING, NO_MAPPING, n1, n2,
        );
    }

    {
        // Construct an AlmProblem with AL-type constraints, but no PM-type ones
        let f1 = |_u: &[f64], _f1up: &mut [f64]| -> FunctionCallResult { Ok(()) };
        let soc = SecondOrderCone::new(1.5);
        let bounds = Ball2::new(None, 10.0);
        let y_set = NoConstraints::new();
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
        let f1 = |_u: &[f64], _f1up: &mut [f64]| -> FunctionCallResult { Ok(()) };
        let f2 = |_u: &[f64], _f2up: &mut [f64]| -> FunctionCallResult { Ok(()) };
        let soc = SecondOrderCone::new(1.5);
        let bounds = Ball2::new(None, 10.0);
        let y_set = NoConstraints::new();
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
    let f = |_u: &[f64], _p: &[f64], _cost: &mut f64| -> FunctionCallResult { Ok(()) };
    let df = |_u: &[f64], _p: &[f64], _grad: &mut [f64]| -> FunctionCallResult { Ok(()) };
    let n1 = 1; // n1 = 1, but there is no set C
    let n2 = 0; // no F2 (and n2 = 0)
    let y_set = NoConstraints::new();
    let bounds = Ball2::new(None, 10.0);
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
    let f = |_u: &[f64], _p: &[f64], _cost: &mut f64| -> FunctionCallResult { Ok(()) };
    let df = |_u: &[f64], _p: &[f64], _grad: &mut [f64]| -> FunctionCallResult { Ok(()) };
    let f2 = |_u: &[f64], _f2up: &mut [f64]| -> FunctionCallResult { Ok(()) };
    let bounds = Ball2::new(None, 10.0);
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

    let f = |_u: &[f64], _p: &[f64], _cost: &mut f64| -> FunctionCallResult { Ok(()) };
    let df = |_u: &[f64], _p: &[f64], _grad: &mut [f64]| -> FunctionCallResult { Ok(()) };
    let f1 = |_u: &[f64], _result: &mut [f64]| -> FunctionCallResult { Ok(()) };
    let set_c = Ball2::new(None, 1.50);

    // Construct an instance of AlmProblem without any PM-type data
    let bounds = Ball2::new(None, 10.0);
    let set_y = Ball2::new(None, 1.0);
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
        .with_max_outer_iterations(10)
        .with_initial_lagrange_multipliers(&vec![5.0; n1]);

    let mut u = vec![0.0; nx];
    println!("result = {:?}", alm_optimizer.solve(&mut u));
}

#[test]
fn t_alm_numeric_test_1() {
    let tolerance = 1e-8;
    let nx = 3;
    let n1 = 2;
    let n2 = 0;
    let lbfgs_mem = 3;
    let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
    let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);

    let set_c = Ball2::new(None, 1.0);
    let bounds = Ball2::new(None, 10.0);
    let set_y = Ball2::new(None, 10000.0);

    let factory = AlmFactory::new(
        mocks::f0,
        mocks::d_f0,
        Some(mocks::mapping_f1_affine),
        Some(mocks::mapping_f1_affine_jacobian_product),
        NO_MAPPING,
        NO_JACOBIAN_MAPPING,
        Some(set_c),
        n2,
    );

    let set_c_b = Ball2::new(None, 1.0);
    let alm_problem = AlmProblem::new(
        bounds,
        Some(set_c_b),
        Some(set_y),
        |u: &[f64], xi: &[f64], cost: &mut f64| -> FunctionCallResult { factory.psi(u, xi, cost) },
        |u: &[f64], xi: &[f64], grad: &mut [f64]| -> FunctionCallResult {
            factory.d_psi(u, xi, grad)
        },
        Some(mocks::mapping_f1_affine),
        NO_MAPPING,
        n1,
        n2,
    );

    let mut alm_optimizer = AlmOptimizer::new(&mut alm_cache, alm_problem)
        .with_delta_tolerance(1e-4)
        .with_max_outer_iterations(30)
        .with_epsilon_tolerance(1e-5)
        .with_initial_inner_tolerance(1e-2)
        .with_inner_tolerance_update_factor(0.5)
        .with_initial_penalty(1.0)
        .with_penalty_update_factor(1.2)
        .with_sufficient_decrease_coefficient(0.1)
        .with_initial_lagrange_multipliers(&vec![5.0; n1]);

    let mut u = vec![0.0; nx];
    let solver_result = alm_optimizer.solve(&mut u);
    assert!(solver_result.is_ok());
    let r = solver_result.unwrap();
    println!("r = {:#?}", r);
    println!("y* = {:#?}", r.lagrange_multipliers());
    println!("time = {:?}", r.solve_time());
    assert_eq!(ExitStatus::Converged, r.exit_status());
    assert!(r.num_outer_iterations() > 0 && r.num_outer_iterations() <= 30);
    assert!(r.last_problem_norm_fpr() < tolerance);

    let mut f1res = vec![0.0; 2];
    assert!(mocks::mapping_f1_affine(&u, &mut f1res).is_ok());

    // --- Test value of optimal cost
    let cost_actual = r.cost();
    let mut cost_expected = 0.0;
    assert!(mocks::f0(&u, &mut cost_expected).is_ok());
    unit_test_utils::assert_nearly_equal(
        cost_actual,
        cost_expected,
        1e-4,
        1e-8,
        "cost value is wrong",
    );
}

fn mapping_f2(u: &[f64], res: &mut [f64]) -> FunctionCallResult {
    res[0] = u[0];
    res[1] = u[1];
    res[2] = u[2] - u[0];
    res[3] = u[2] - u[0] - u[1];
    Ok(())
}

fn jac_mapping_f2_tr(_u: &[f64], d: &[f64], res: &mut [f64]) -> FunctionCallResult {
    res[0] = d[0] - d[2] - d[3];
    res[1] = d[1] - d[3];
    res[2] = d[2] + d[3];
    Ok(())
}

#[test]
fn t_alm_numeric_test_2() {
    let tolerance = 1e-8;
    let nx = 3;
    let n1 = 2;
    let n2 = 4;
    let lbfgs_mem = 3;
    let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
    let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);

    let set_c = Ball2::new(None, 1.0);
    let bounds = Ball2::new(None, 10.0);
    let set_y = Ball2::new(None, 10000.0);

    let factory = AlmFactory::new(
        mocks::f0,
        mocks::d_f0,
        Some(mocks::mapping_f1_affine),
        Some(mocks::mapping_f1_affine_jacobian_product),
        Some(mapping_f2),
        Some(jac_mapping_f2_tr),
        Some(set_c),
        n2,
    );

    let set_c_b = Ball2::new(None, 1.0);
    let alm_problem = AlmProblem::new(
        bounds,
        Some(set_c_b),
        Some(set_y),
        |u: &[f64], xi: &[f64], cost: &mut f64| -> FunctionCallResult { factory.psi(u, xi, cost) },
        |u: &[f64], xi: &[f64], grad: &mut [f64]| -> FunctionCallResult {
            factory.d_psi(u, xi, grad)
        },
        Some(mocks::mapping_f1_affine),
        Some(mapping_f2),
        n1,
        n2,
    );

    let mut alm_optimizer = AlmOptimizer::new(&mut alm_cache, alm_problem)
        .with_delta_tolerance(1e-4)
        .with_epsilon_tolerance(1e-5)
        .with_initial_inner_tolerance(1e-4);

    let mut u = vec![0.0; nx];
    let solver_result = alm_optimizer.solve(&mut u);
    assert!(solver_result.is_ok());
    let r = solver_result.unwrap();
    assert_eq!(ExitStatus::Converged, r.exit_status());
    assert!(r.num_outer_iterations() > 0 && r.num_outer_iterations() <= 10);
    assert!(r.last_problem_norm_fpr() < tolerance);

    let mut f1res = vec![0.0; 2];
    assert!(mocks::mapping_f1_affine(&u, &mut f1res).is_ok());
    println!("r = {:#?}", r);
    let mut f2u = vec![0.0; n2];
    assert!(mapping_f2(&u, &mut f2u).is_ok());
    assert!(crate::matrix_operations::norm2(&f2u) < 1e-4);
    println!("F2(u*) = {:#?}", &f2u);

    println!("y = {:#?}", r.lagrange_multipliers());
}

// Trait alias (type aliases are not stable yet, so the alternative is to use
// the following trait definition, i.e., to "extend" Fn and implement it)
// See https://bit.ly/2zJvd6g
trait CostFunction: Fn(&[f64], &[f64], &mut f64) -> FunctionCallResult {}
impl<T> CostFunction for T where T: Fn(&[f64], &[f64], &mut f64) -> FunctionCallResult + ?Sized {}

// Badass implementation: (i) returning a Box-ed closure: that's how we can
// return closures from a function, (ii) moving the context of the function
// (using `move`), (iii) `param` should leave at least as long as the box
// See: https://bit.ly/36icE5r
fn make_mockito_f0<'a>(c: f64, param: &'a [f64]) -> impl CostFunction + 'a {
    Box::new(
        move |_u: &[f64], _xi: &[f64], cost: &mut f64| -> FunctionCallResult {
            *cost = c + param[0];
            Ok(())
        },
    )
}

fn make_mockito_jacobian(_param: &[f64]) -> Box<JacobianMappingType> {
    Box::new(|_u: &[f64], _xi: &[f64], _grad: &mut [f64]| -> FunctionCallResult { Ok(()) })
}

#[test]
fn t_alm_numeric_test_repeat() {
    let tolerance = 1e-8;
    let nx = 3;
    let n1 = 2;
    let n2 = 4;
    let lbfgs_mem = 3;
    let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
    let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);

    for i in 1..3 {
        let bounds = Ball2::new(None, 10.0);
        let set_y = Ball2::new(None, 10000.0);

        let parameter = [1.0, 2.0 * i as f64];

        let f0 = make_mockito_f0(1.0, &parameter);
        let jf = make_mockito_jacobian(&parameter);

        let set_c_b = Ball2::new(None, 1.0);
        let alm_problem = AlmProblem::new(
            bounds,
            Some(set_c_b),
            Some(set_y),
            f0,
            jf,
            Some(mocks::mapping_f1_affine),
            Some(mapping_f2),
            n1,
            n2,
        );

        // ALM *borrows* the cache: lovely! Otherwise we wouldn't be able to
        // run this in a loop with the cache allocated outside the loop once
        let mut alm_optimizer = AlmOptimizer::new(&mut alm_cache, alm_problem)
            .with_delta_tolerance(1e-4)
            .with_epsilon_tolerance(1e-5)
            .with_initial_inner_tolerance(1e-4);

        let mut u = vec![0.0; nx];
        let solver_result = alm_optimizer.solve(&mut u);
        assert!(solver_result.is_ok());
        assert_eq!(ExitStatus::Converged, solver_result.unwrap().exit_status());
    }
}

#[test]
fn t_alm_numeric_test_out_of_outer_iterations() {
    let tolerance = 1e-8;
    let nx = 3;
    let n1 = 2;
    let n2 = 4;
    let lbfgs_mem = 3;
    let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
    let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);

    let set_c = Ball2::new(None, 1.0);
    let bounds = Ball2::new(None, 10.0);
    let set_y = Ball2::new(None, 10000.0);

    let factory = AlmFactory::new(
        mocks::f0,
        mocks::d_f0,
        Some(mocks::mapping_f1_affine),
        Some(mocks::mapping_f1_affine_jacobian_product),
        Some(mapping_f2),
        Some(jac_mapping_f2_tr),
        Some(set_c),
        n2,
    );

    let set_c_b = Ball2::new(None, 1.0);
    let alm_problem = AlmProblem::new(
        bounds,
        Some(set_c_b),
        Some(set_y),
        |u: &[f64], xi: &[f64], cost: &mut f64| -> FunctionCallResult { factory.psi(u, xi, cost) },
        |u: &[f64], xi: &[f64], grad: &mut [f64]| -> FunctionCallResult {
            factory.d_psi(u, xi, grad)
        },
        Some(mocks::mapping_f1_affine),
        Some(mapping_f2),
        n1,
        n2,
    );

    let mut alm_optimizer = AlmOptimizer::new(&mut alm_cache, alm_problem)
        .with_delta_tolerance(1e-4)
        .with_epsilon_tolerance(1e-5)
        .with_initial_inner_tolerance(1e-4)
        .with_max_outer_iterations(3);

    let mut u = vec![0.0; nx];
    let solver_result = alm_optimizer.solve(&mut u);
    println!("{:#?}", solver_result);
    assert!(solver_result.is_ok());
    assert_eq!(
        ExitStatus::NotConvergedIterations,
        solver_result.unwrap().exit_status()
    );
}

#[test]
fn t_alm_numeric_test_out_of_inner_iterations() {
    let tolerance = 1e-8;
    let nx = 3;
    let n1 = 2;
    let n2 = 4;
    let lbfgs_mem = 3;
    let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
    let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);

    let set_c = Ball2::new(None, 1.0);
    let bounds = Ball2::new(None, 10.0);
    let set_y = Ball2::new(None, 10000.0);

    let factory = AlmFactory::new(
        mocks::f0,
        mocks::d_f0,
        Some(mocks::mapping_f1_affine),
        Some(mocks::mapping_f1_affine_jacobian_product),
        Some(mapping_f2),
        Some(jac_mapping_f2_tr),
        Some(set_c),
        n2,
    );

    let set_c_b = Ball2::new(None, 1.0);
    let alm_problem = AlmProblem::new(
        bounds,
        Some(set_c_b),
        Some(set_y),
        |u: &[f64], xi: &[f64], cost: &mut f64| -> FunctionCallResult { factory.psi(u, xi, cost) },
        |u: &[f64], xi: &[f64], grad: &mut [f64]| -> FunctionCallResult {
            factory.d_psi(u, xi, grad)
        },
        Some(mocks::mapping_f1_affine),
        Some(mapping_f2),
        n1,
        n2,
    );

    let mut alm_optimizer = AlmOptimizer::new(&mut alm_cache, alm_problem)
        .with_delta_tolerance(1e-4)
        .with_epsilon_tolerance(1e-5)
        .with_initial_inner_tolerance(1e-4)
        .with_max_inner_iterations(10);

    let mut u = vec![0.0; nx];
    let solver_result = alm_optimizer.solve(&mut u);
    println!("{:#?}", solver_result);
    assert!(solver_result.is_ok());
    assert_eq!(
        ExitStatus::NotConvergedIterations,
        solver_result.unwrap().exit_status()
    );
}

#[test]
fn t_alm_numeric_test_out_of_time() {
    let tolerance = 1e-8;
    let nx = 3;
    let n1 = 2;
    let n2 = 4;
    let lbfgs_mem = 3;
    let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
    let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);

    let set_c = Ball2::new(None, 1.0);
    let bounds = Ball2::new(None, 10.0);
    let set_y = Ball2::new(None, 10000.0);

    let factory = AlmFactory::new(
        mocks::f0,
        mocks::d_f0,
        Some(mocks::mapping_f1_affine),
        Some(mocks::mapping_f1_affine_jacobian_product),
        Some(mapping_f2),
        Some(jac_mapping_f2_tr),
        Some(set_c),
        n2,
    );

    let set_c_b = Ball2::new(None, 1.0);
    let alm_problem = AlmProblem::new(
        bounds,
        Some(set_c_b),
        Some(set_y),
        |u: &[f64], xi: &[f64], cost: &mut f64| -> FunctionCallResult { factory.psi(u, xi, cost) },
        |u: &[f64], xi: &[f64], grad: &mut [f64]| -> FunctionCallResult {
            factory.d_psi(u, xi, grad)
        },
        Some(mocks::mapping_f1_affine),
        Some(mapping_f2),
        n1,
        n2,
    );

    let mut alm_optimizer = AlmOptimizer::new(&mut alm_cache, alm_problem)
        .with_delta_tolerance(1e-4)
        .with_epsilon_tolerance(1e-5)
        .with_initial_inner_tolerance(1e-4)
        .with_max_duration(std::time::Duration::from_micros(400));

    let mut u = vec![0.0; nx];
    let solver_result = alm_optimizer.solve(&mut u);
    println!("{:?}", solver_result);
    assert!(solver_result.is_ok());
    assert_eq!(
        ExitStatus::NotConvergedOutOfTime,
        solver_result.unwrap().exit_status()
    );
}

#[test]
fn t_alm_numeric_test_no_mappings() {
    let tolerance = 1e-8;
    let nx = 3;
    let n1 = 0;
    let n2 = 0;
    let lbfgs_mem = 3;
    let panoc_cache = PANOCCache::new(nx, tolerance, lbfgs_mem);
    let mut alm_cache = AlmCache::new(panoc_cache, n1, n2);

    let bounds = Ball2::new(None, 10.0);

    let factory = AlmFactory::new(
        mocks::f0,
        mocks::d_f0,
        NO_MAPPING,
        NO_JACOBIAN_MAPPING,
        NO_MAPPING,
        NO_JACOBIAN_MAPPING,
        NO_SET,
        n2,
    );

    let alm_problem = AlmProblem::new(
        bounds,
        NO_SET,
        NO_SET,
        |u: &[f64], xi: &[f64], cost: &mut f64| -> FunctionCallResult { factory.psi(u, xi, cost) },
        |u: &[f64], xi: &[f64], grad: &mut [f64]| -> FunctionCallResult {
            factory.d_psi(u, xi, grad)
        },
        NO_MAPPING,
        NO_MAPPING,
        n1,
        n2,
    );

    let mut alm_optimizer = AlmOptimizer::new(&mut alm_cache, alm_problem)
        .with_delta_tolerance(1e-16)
        .with_epsilon_tolerance(1e-5)
        .with_initial_inner_tolerance(1e-5);

    let mut u = vec![0.0; nx];
    let solver_result = alm_optimizer.solve(&mut u);
    assert!(solver_result.is_ok());
    let res = solver_result.unwrap();
    assert_eq!(ExitStatus::Converged, res.exit_status());
    assert_eq!(1, res.num_outer_iterations());
    assert!(res.last_problem_norm_fpr() <= 1e-5);
}
