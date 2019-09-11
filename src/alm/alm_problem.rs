use crate::{constraints, SolverError};

pub struct AlmProblem<
    ParametricMappingType,
    ParametricGradientType,
    ConstraintsType,
    AlmSetC,
    LagrangeSetY,
    ParametricCostType,
> where
    ParametricMappingType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricGradientType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricCostType: Fn(&[f64], &[f64], &mut f64) -> Result<(), SolverError>,
    ConstraintsType: constraints::Constraint,
    AlmSetC: constraints::Constraint,
    LagrangeSetY: constraints::Constraint,
{
    //
    // NOTE: the reason why we need to define different set types (ConstraintsType,
    // AlmSetC, LagrangeSetY) is that these three sets are allowed to be of different
    // type; their actual sized type is not Constraint! (e.g., None::<Constraint> does
    // not have a known type)
    //
    /// main constraints (prio)
    pub(crate) constraints: ConstraintsType,
    /// Set C for ALM-type constraints
    pub(crate) alm_set_c: Option<AlmSetC>,
    /// Set Y for Lagrange multipliers (convex, compact)
    pub(crate) alm_set_y: Option<LagrangeSetY>,
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

impl<
        ParametricMappingType,
        ParametricGradientType,
        ConstraintsType,
        AlmSetC,
        LagrangeSetY,
        ParametricCostType,
    >
    AlmProblem<
        ParametricMappingType,
        ParametricGradientType,
        ConstraintsType,
        AlmSetC,
        LagrangeSetY,
        ParametricCostType,
    >
where
    ParametricMappingType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricGradientType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricCostType: Fn(&[f64], &[f64], &mut f64) -> Result<(), SolverError>,
    ConstraintsType: constraints::Constraint,
    AlmSetC: constraints::Constraint,
    LagrangeSetY: constraints::Constraint,
{
    pub fn new(
        constraints: ConstraintsType,
        alm_set_c: Option<AlmSetC>,
        alm_set_y: Option<LagrangeSetY>,
        parametric_cost: ParametricCostType,
        parametric_gradient: ParametricGradientType,
        mapping_f1: Option<ParametricMappingType>,
        mapping_f2: Option<ParametricMappingType>,
        n1: usize,
        n2: usize,
    ) -> Self {
        // if one of `mapping_f1` and `alm_set_c` is provided, the other one
        // should be provided as well (it's ok for both to be None)
        assert!(
            !(mapping_f1.is_none() ^ alm_set_c.is_none()),
            "either F1 or C has not been provided"
        );
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
