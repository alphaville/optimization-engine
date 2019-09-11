use crate::{constraints, SolverError};

pub struct AlmProblem<ParametricMappingType, ParametricGradientType, SetType, ParametricCostType>
where
    ParametricMappingType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricGradientType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricCostType: Fn(&[f64], &[f64], &mut f64) -> Result<(), SolverError>,
    SetType: constraints::Constraint,
{
    /// main constraints (prio)
    pub(crate) constraints: SetType,
    /// Set C for ALM-type constraints
    pub(crate) alm_set_c: Option<SetType>,
    /// Set Y for Lagrange multipliers (convex, compact)
    pub(crate) alm_set_y: Option<SetType>,
    /// parametric cost function, psi(u; p)
    pub(crate) parametric_cost: ParametricCostType,
    /// gradient of parametric cost function, psi'(u; p)
    pub(crate) parametric_gradient: ParametricGradientType,
    /// Mapping F1(u; p)
    pub(crate) mapping_f1: Option<ParametricMappingType>,
    /// Mapping F2(u; p)
    pub(crate) mapping_f2: Option<ParametricMappingType>,
    /// number of ALM-type parameters (range dim of F1 and C)
    pub(crate) n1: usize,
    /// number of PM-type parameters (range dim of F2)
    pub(crate) n2: usize,
}

impl<ParametricMappingType, ParametricGradientType, SetType, ParametricCostType>
    AlmProblem<ParametricMappingType, ParametricGradientType, SetType, ParametricCostType>
where
    ParametricMappingType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricGradientType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricCostType: Fn(&[f64], &[f64], &mut f64) -> Result<(), SolverError>,
    SetType: constraints::Constraint,
{
    pub fn new(
        constraints: SetType,
        alm_set_c: Option<SetType>,
        alm_set_y: Option<SetType>,
        parametric_cost: ParametricCostType,
        parametric_gradient: ParametricGradientType,
        mapping_f1: Option<ParametricMappingType>,
        mapping_f2: Option<ParametricMappingType>,
        n1: usize,
        n2: usize,
    ) -> Self {
        AlmProblem {
            constraints,
            alm_set_c,
            alm_set_y,
            parametric_cost,
            parametric_gradient,
            mapping_f1,
            mapping_f2,
            n1,
            n2,
        }
    }
}
