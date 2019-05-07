//! PANOC super-fast algorithm
use super::Problem;
use crate::constraints;
use std::time;

mod panoc_cache;
mod panoc_engine;
mod panoc_optimizer;

/// Cache for PANOC
///
/// This struct carries all the information needed at every step of the algorithm.
///
/// An instance of `PANOCCache` needs to be allocated once and a (mutable) reference to it should
/// be passed to instances of [PANOCEngine](struct.PANOCEngine.html)
///
/// Subsequently, a `PANOCEngine` is used to construct an instance of `PANOCAlgorithm`
///
#[derive(Debug)]
pub struct PANOCCache {
    lbfgs: lbfgs::Lbfgs,
    gradient_u: Vec<f64>,
    u_half_step: Vec<f64>,
    gradient_step: Vec<f64>,
    direction_lbfgs: Vec<f64>,
    u_plus: Vec<f64>,
    rhs_ls: f64,
    lhs_ls: f64,
    gamma_fpr: Vec<f64>,
    gamma: f64,
    tolerance: f64,
    norm_gamma_fpr: f64,
    tau: f64,
    lipschitz_constant: f64,
    sigma: f64,
    cost_value: f64,
    iteration: usize,
}

/// Engine for PANOC algorithm
pub struct PANOCEngine<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32,
    CostType: Fn(&[f64], &mut f64) -> i32,
    ConstraintType: constraints::Constraint,
{
    problem: Problem<GradientType, ConstraintType, CostType>,
    cache: &'a mut PANOCCache,
}

pub struct PANOCOptimizer<'a, GradientType, ConstraintType, CostType>
where
    GradientType: Fn(&[f64], &mut [f64]) -> i32,
    CostType: Fn(&[f64], &mut f64) -> i32,
    ConstraintType: constraints::Constraint,
{
    panoc_engine: &'a mut PANOCEngine<'a, GradientType, ConstraintType, CostType>,
    max_iter: usize,
    max_duration: Option<time::Duration>,
}

#[cfg(test)]
mod tests;
