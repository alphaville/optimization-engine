use crate::{constraints::Constraint, SolverError};

pub struct AlmProblem<
    MappingAlm,
    MappingPm,
    ParametricGradientType,
    ParametricCostType,
    ConstraintsType,
    AlmSetC,
    LagrangeSetY,
> where
    // This is function F1: R^xn --> R^n1 (ALM)
    MappingAlm: Fn(&[f64], &mut [f64]) -> Result<(), SolverError>,
    // This is function F2: R^xn --> R^n2 (PM)
    MappingPm: Fn(&[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricGradientType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricCostType: Fn(&[f64], &[f64], &mut f64) -> Result<(), SolverError>,
    ConstraintsType: Constraint,
    AlmSetC: Constraint,
    LagrangeSetY: Constraint,
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
    pub(crate) mapping_f1: Option<MappingAlm>,
    /// Mapping F2(u; p)
    pub(crate) mapping_f2: Option<MappingPm>,
    /// number of ALM-type parameters (range dim of F1 and C)
    pub(crate) n1: usize,
    /// number of PM-type parameters (range dim of F2)
    pub(crate) n2: usize,
}

impl<
        MappingAlm,
        MappingPm,
        ParametricGradientType,
        ParametricCostType,
        ConstraintsType,
        AlmSetC,
        LagrangeSetY,
    >
    AlmProblem<
        MappingAlm,
        MappingPm,
        ParametricGradientType,
        ParametricCostType,
        ConstraintsType,
        AlmSetC,
        LagrangeSetY,
    >
where
    MappingAlm: Fn(&[f64], &mut [f64]) -> Result<(), SolverError>,
    MappingPm: Fn(&[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricGradientType: Fn(&[f64], &[f64], &mut [f64]) -> Result<(), SolverError>,
    ParametricCostType: Fn(&[f64], &[f64], &mut f64) -> Result<(), SolverError>,
    ConstraintsType: Constraint,
    AlmSetC: Constraint,
    LagrangeSetY: Constraint,
{
    ///Constructs new instance of `AlmProblem`
    ///
    /// ## Arguments
    ///
    /// - `constraints`: hard constraints
    /// - `alm_set_c`: Set `C` of ALM-specific constraints
    /// - `alm_set_y`: Compact, convex set `Y` of Lagrange multipliers
    /// - `parametric_cost`: Parametric cost function, `f(x; p)`
    /// - `parametric_gradient`: Gradient of cost function wrt `x`, that is `df(x; p)`
    /// - `mapping_f1`: Mapping `F1` of ALM-specific constraints (`F1(x; p) in C`)
    /// - `mapping_f2`: Mapping `F2` of PM-specific constraints (`F2(x; p) = 0`)
    /// - `n1`: range dimension of `mapping_f1`
    /// - `n2`: range dimension of `mapping_f2`
    ///
    ///
    /// ## Returns
    ///
    /// Instance of `AlmProblem`
    ///
    /// ## Example
    ///
    ///
    /// ```rust
    /// use optimization_engine::{SolverError, alm::*, constraints::Ball2};
    ///
    /// let f = |_u: &[f64], _p: &[f64], _cost: &mut f64| -> Result<(), SolverError> { Ok(()) };
    /// let df = |_u: &[f64], _p: &[f64], _grad: &mut [f64]| -> Result<(), SolverError> { Ok(()) };
    /// let n1 = 0;
    /// let n2 = 0;
    /// let bounds = Ball2::new(None, 10.0);
    /// let _alm_problem = AlmProblem::new(
    ///     bounds, NO_SET, NO_SET, f, df, NO_MAPPING, NO_MAPPING, n1, n2,
    /// );
    /// ```
    ///
    pub fn new(
        constraints: ConstraintsType,
        alm_set_c: Option<AlmSetC>,
        alm_set_y: Option<LagrangeSetY>,
        parametric_cost: ParametricCostType,
        parametric_gradient: ParametricGradientType,
        mapping_f1: Option<MappingAlm>,
        mapping_f2: Option<MappingPm>,
        n1: usize,
        n2: usize,
    ) -> Self {
        // if one of `mapping_f1` and `alm_set_c` is provided, the other one
        // should be provided as well (it's ok for both to be None)
        assert!(
            !(mapping_f1.is_none() ^ alm_set_c.is_none()),
            "either F1 or C has not been provided"
        );
        assert!(!(alm_set_c.is_none() ^ (n1 == 0)), "C is Some iff n1 > 0");
        assert!(!(alm_set_y.is_none() ^ (n1 == 0)), "Y is Some iff n1 > 0");
        assert!(!(mapping_f2.is_none() ^ (n2 == 0)), "F2 is Some iff n2 > 0");

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
