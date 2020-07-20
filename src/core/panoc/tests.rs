use crate::core::panoc::panoc_engine::PANOCEngine;
use crate::core::panoc::*;
use crate::core::*;
use crate::{mocks, FunctionCallResult};

const N_DIM: usize = 2;
#[test]
fn t_panoc_init() {
    let radius = 0.2;
    let ball = constraints::Ball2::new(None, radius);
    let problem = Problem::new(&ball, mocks::my_gradient, mocks::my_cost);
    let mut panoc_cache = PANOCCache::new(N_DIM, 1e-6, 5);

    {
        let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);
        let mut u = [0.75, -1.4];
        panoc_engine.init(&mut u).unwrap();
        assert!(2.549_509_967_743_775 > panoc_engine.cache.lipschitz_constant);
        assert!(0.372_620_625_931_781 < panoc_engine.cache.gamma, "gamma");
        println!("----------- {} ", panoc_engine.cache.cost_value);
        unit_test_utils::assert_nearly_equal(
            6.34125,
            panoc_engine.cache.cost_value,
            1e-4,
            1e-10,
            "cost value",
        );
        unit_test_utils::assert_nearly_equal_array(
            &[0.35, -3.05],
            &panoc_engine.cache.gradient_u,
            1e-4,
            1e-10,
            "gradient at u",
        );
    }
    println!("cache = {:#?}", &panoc_cache);
}

fn print_panoc_engine<'a, GradientType, ConstraintType, CostType>(
    panoc_engine: &PANOCEngine<'a, GradientType, ConstraintType, CostType>,
) where
    GradientType: Fn(&[f64], &mut [f64]) -> FunctionCallResult,
    CostType: Fn(&[f64], &mut f64) -> FunctionCallResult,
    ConstraintType: constraints::Constraint,
{
    println!("> fpr       = {:?}", &panoc_engine.cache.gamma_fpr);
    println!("> fpr       = {:.2e}", panoc_engine.cache.norm_gamma_fpr);
    println!("> L         = {:.3}", panoc_engine.cache.lipschitz_constant);
    println!("> gamma     = {:.10}", panoc_engine.cache.gamma);
    println!("> tau       = {:.3}", panoc_engine.cache.tau);
    println!("> lbfgs dir = {:.11?}", panoc_engine.cache.direction_lbfgs);
}

#[test]
fn t_test_panoc_basic() {
    let bounds = constraints::Ball2::new(None, 0.2);
    let problem = Problem::new(&bounds, mocks::my_gradient, mocks::my_cost);
    let tolerance = 1e-9;
    let mut panoc_cache = PANOCCache::new(2, tolerance, 5);
    let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);

    let mut u = [0.0, 0.0];
    panoc_engine.init(&mut u).unwrap();
    panoc_engine.step(&mut u).unwrap();
    let fpr0 = panoc_engine.cache.norm_gamma_fpr;
    println!("fpr0 = {}", fpr0);

    for i in 1..=100 {
        println!("----------------------------------------------------");
        println!("> iter      = {}", i);
        print_panoc_engine(&panoc_engine);
        println!("> u         = {:.14?}", u);
        if panoc_engine.step(&mut u) != Ok(true) {
            break;
        }
    }
    println!("final |fpr| = {}", panoc_engine.cache.norm_gamma_fpr);
    assert!(panoc_engine.cache.norm_gamma_fpr <= tolerance);
    unit_test_utils::assert_nearly_equal_array(&u, &mocks::SOLUTION_A, 1e-6, 1e-8, "");
}

#[test]
fn t_test_panoc_hard() {
    let radius: f64 = 0.05;
    let bounds = constraints::Ball2::new(None, radius);
    let problem = Problem::new(
        &bounds,
        mocks::hard_quadratic_gradient,
        mocks::hard_quadratic_cost,
    );
    let n: usize = 3;
    let lbfgs_memory: usize = 10;
    let tolerance_fpr: f64 = 1e-12;
    let mut panoc_cache = PANOCCache::new(n, tolerance_fpr, lbfgs_memory);
    let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);

    let mut u = [-20., 10., 0.2];
    panoc_engine.init(&mut u).unwrap();

    println!("L     = {}", panoc_engine.cache.lipschitz_constant);
    println!("gamma = {}", panoc_engine.cache.gamma);
    println!("sigma = {}", panoc_engine.cache.sigma);

    let mut i = 1;
    println!("\n*** ITERATION   1");
    while panoc_engine.step(&mut u) == Ok(true) && i < 100 {
        i += 1;
        println!("+ u_plus               = {:?}", u);
        println!("\n*** ITERATION {:3}", i);
    }

    println!("\nsol = {:?}", u);
    assert!(panoc_engine.cache.norm_gamma_fpr <= tolerance_fpr);
    unit_test_utils::assert_nearly_equal_array(&u, &mocks::SOLUTION_HARD, 1e-6, 1e-8, "");
}

#[test]
fn t_test_panoc_rosenbrock() {
    let tolerance = 1e-12;
    let a_param = 1.0;
    let b_param = 100.0;
    let cost_gradient = |u: &[f64], grad: &mut [f64]| -> FunctionCallResult {
        mocks::rosenbrock_grad(a_param, b_param, u, grad);
        Ok(())
    };
    let cost_function = |u: &[f64], c: &mut f64| -> FunctionCallResult {
        *c = mocks::rosenbrock_cost(a_param, b_param, u);
        Ok(())
    };
    let bounds = constraints::Ball2::new(None, 1.0);
    let problem = Problem::new(&bounds, cost_gradient, cost_function);
    let mut panoc_cache = PANOCCache::new(2, tolerance, 2).with_cbfgs_parameters(2.0, 1e-6, 1e-12);
    let mut panoc_engine = PANOCEngine::new(problem, &mut panoc_cache);
    let mut u_solution = [-1.5, 0.9];
    panoc_engine.init(&mut u_solution).unwrap();
    let mut idx = 1;
    while panoc_engine.step(&mut u_solution) == Ok(true) && idx < 50 {
        idx += 1;
    }
    assert!(panoc_engine.cache.norm_gamma_fpr <= tolerance);
    println!("u = {:?}", u_solution);
}
