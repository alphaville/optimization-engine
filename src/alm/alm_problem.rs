use crate::{constraints::Constraint, FunctionCallResult};

/// Definition of optimization problem to be solved with `AlmOptimizer`. The optimization
/// problem has the general form
///
/// $$\begin{aligned}
/// \mathrm{Minimize}\  f(u)
/// \\\\
/// u \in U
/// \\\\
/// F_1(u) \in C
/// \\\\
/// F_2(u) = 0
/// \end{aligned}$$
///
/// where
///
/// - $u\in\mathbb{R}^{n_u}$ is the decision variable,
/// - $f:\mathbb{R}^n\to\mathbb{R}$ is a $C^{1,1}$-smooth cost function,
/// - $U$ is a (not necessarily convex) closed subset of $\mathbb{R}^{n_u}$
///   on which we can easily compute projections (e.g., a rectangle, a ball,
///   a second-order cone, a finite set, etc),
/// - $F_1:\mathbb{R}^{n_u}\to\mathbb{R}^{n_1}$ and $F_2:\mathbb{R}^{n_u} \to\mathbb{R}^{n_2}$
///   are mappings with smooth partial derivatives, and
/// - $C\subseteq\mathbb{R}^{n_1}$ is a convex closed set on which we can easily compute projections.
///
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
    MappingAlm: Fn(&[f64], &mut [f64]) -> FunctionCallResult,
    // This is function F2: R^xn --> R^n2 (PM)
    MappingPm: Fn(&[f64], &mut [f64]) -> FunctionCallResult,
    ParametricGradientType: Fn(&[f64], &[f64], &mut [f64]) -> FunctionCallResult,
    ParametricCostType: Fn(&[f64], &[f64], &mut f64) -> FunctionCallResult,
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
    MappingAlm: Fn(&[f64], &mut [f64]) -> FunctionCallResult,
    MappingPm: Fn(&[f64], &mut [f64]) -> FunctionCallResult,
    ParametricGradientType: Fn(&[f64], &[f64], &mut [f64]) -> FunctionCallResult,
    ParametricCostType: Fn(&[f64], &[f64], &mut f64) -> FunctionCallResult,
    ConstraintsType: Constraint,
    AlmSetC: Constraint,
    LagrangeSetY: Constraint,
{
    ///Constructs new instance of `AlmProblem`
    ///
    /// # Arguments
    ///
    /// - `constraints`: hard constraints, set $U$
    /// - `alm_set_c`: Set $C$ of ALM-specific constraints (convex, closed)
    /// - `alm_set_y`: Compact, convex set $Y$ of Lagrange multipliers, which needs to be a
    ///    compact subset of $C^*$ (the convex conjugate of the convex set $C{}\subseteq{}\mathbb{R}^{n_1}$)
    /// - `parametric_cost`: Parametric cost function, $\psi(u, \xi)$, where $\xi = (c, y)$
    /// - `parametric_gradient`: Gradient of cost function wrt $u$, that is $\nabla_x \psi(u, \xi)$
    /// - `mapping_f1`: Mapping `F1` of ALM-specific constraints ($F1(u) \in C$)
    /// - `mapping_f2`: Mapping `F2` of PM-specific constraints ($F2(u) = 0$)
    /// - `n1`: range dimension of $F_1(u)$ (that is, `mapping_f1`)
    /// - `n2`: range dimension of $F_2(u)$ (that is, `mapping_f2`)
    ///
    ///
    /// # Returns
    ///
    /// Instance of `AlmProblem`
    ///
    /// # Example
    ///
    ///
    /// ```rust
    /// use optimization_engine::{FunctionCallResult, alm::*, constraints::Ball2};
    ///
    /// let psi = |_u: &[f64], _p: &[f64], _cost: &mut f64| -> FunctionCallResult { Ok(()) };
    /// let dpsi = |_u: &[f64], _p: &[f64], _grad: &mut [f64]| -> FunctionCallResult { Ok(()) };
    /// let n1 = 0;
    /// let n2 = 0;
    /// let bounds = Ball2::new(None, 10.0);
    /// let _alm_problem = AlmProblem::new(
    ///     bounds, NO_SET, NO_SET, psi, dpsi, NO_MAPPING, NO_MAPPING, n1, n2,
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
