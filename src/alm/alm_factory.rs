/* ---------------------------------------------------------------------------- */
/* ALM FACTORY                                                                  */
/*                                                                              */
/* About: The user provides f, df, F1, JF1'*d, F2 and C and MockAlmFactory      */
/*        prepares psi and d_psi, which can be used to define an AlmOptimizer   */
/* ---------------------------------------------------------------------------- */

use crate::{constraints::Constraint, matrix_operations, FunctionCallResult};

/// Prepares function $\psi$ and its gradient given the problem data: $f$, $\nabla{}f$,
/// and optionally $F_1$, $JF_1$, $C$ and $F_2$
///
/// # Types
///
/// - `Cost`: cost function $f:\mathbb{R}^{n_u} \to \mathbb{R}$ which is computed
///           by a function with signature:
///
///```rust,ignore
///fn f(u: &[f64], cost: &mut f64) -> FunctionCallResult
///```
///
///  where `cost` is updated with the value $f(u)$,
///
/// - `CostGradient`: gradient of the cost function, $\nabla f: \mathbb{R}^{n_u} \to \mathbb{R}^{n_u}$,
///                   which is computed by a function with signature
///
/// ```rust,ignore
/// fn df(u: &[f64], grad: &mut [f64]) -> FunctionCallResult
/// ```
///
/// where on exit `grad` stores the
///
/// - `MappingF1` and `MappingF2`: mappings $F_1:\mathbb{R}^n\to\mathbb{R}^{n_1}$
///                                and $F_2:\mathbb{R}^n\to\mathbb{R}^{n_2}$ which
///                                are computed by functions with signature
///
/// ```rust,ignore
/// fn mapping(u: &[f64], fu: &mut [f64]) -> FunctionCallResult
/// ```
///
/// - `JacobianMappingF1Trans` and `JacobianMappingF2Trans`: functions that compute
///    product of the form $JF_i(u)^\top{}d$ for given $d\in\mathbb{R}^{n_i}$ and
///    $u\in\mathbb{R}^{n_u}$
///
/// - `SetC`: A set $C\subseteq \mathbb{R}^{n_1}$, which is used in the definition
///           of the constraints $F_1(u) \in C$
///
/// The above are used to compute $\psi:\mathbb{R}^{n_u}\to\mathbb{R}$ for given
/// $u\in\mathbb{R}^{n_u}$ and $\xi=(c, y)\in\mathbb{R}^{n_1+1}$, where $c\in\mathbb{R}$
/// and $y\in\mathbb{R}^{n_1}$ are the penalty parameter and vector of Lagrange
/// multipliers respectively, according to
///
/// $$
/// \psi(u) = f(u) + \tfrac{c}{2}\left[\mathrm{dist}_C^2(F_1(u) + \bar{c}^{-1}y)
///         + \Vert F_2(u) \Vert^2\right],
/// $$
///
/// where $\bar{c} = \max\\{1,c\\}$ and
///
/// $$
/// \nabla \psi(u) = \nabla f(u) + c JF_1(u)^\top [t(u) - \Pi_C(t(u))]
///                              + c JF_2(u)^\top F_2(u).
/// $$
///
/// where $t(u) = F_1(u) + \bar{c}^{-1}y$.
///
pub struct AlmFactory<
    MappingF1,
    JacobianMappingF1Trans,
    MappingF2,
    JacobianMappingF2Trans,
    Cost,
    CostGradient,
    SetC,
> where
    Cost: Fn(&[f64], &mut f64) -> FunctionCallResult, // f(u, result)
    CostGradient: Fn(&[f64], &mut [f64]) -> FunctionCallResult, // df(u, result)
    MappingF1: Fn(&[f64], &mut [f64]) -> FunctionCallResult, // f1(u, result)
    JacobianMappingF1Trans: Fn(&[f64], &[f64], &mut [f64]) -> FunctionCallResult, // jf1(u, d, result)
    MappingF2: Fn(&[f64], &mut [f64]) -> FunctionCallResult,                      // f2(u, result)
    JacobianMappingF2Trans: Fn(&[f64], &[f64], &mut [f64]) -> FunctionCallResult, // jf2(u, d, result)
    SetC: Constraint,
{
    f: Cost,
    df: CostGradient,
    mapping_f1: Option<MappingF1>,
    jacobian_mapping_f1_trans: Option<JacobianMappingF1Trans>,
    mapping_f2: Option<MappingF2>,
    jacobian_mapping_f2_trans: Option<JacobianMappingF2Trans>,
    set_c: Option<SetC>,
    n2: usize,
}

impl<
        MappingF1,
        JacobianMappingF1Trans,
        MappingF2,
        JacobianMappingF2Trans,
        Cost,
        CostGradient,
        SetC,
    >
    AlmFactory<
        MappingF1,
        JacobianMappingF1Trans,
        MappingF2,
        JacobianMappingF2Trans,
        Cost,
        CostGradient,
        SetC,
    >
where
    Cost: Fn(&[f64], &mut f64) -> FunctionCallResult, // f(u, result)
    CostGradient: Fn(&[f64], &mut [f64]) -> FunctionCallResult, // df(u, result)
    MappingF1: Fn(&[f64], &mut [f64]) -> FunctionCallResult, // f1(u, result)
    JacobianMappingF1Trans: Fn(&[f64], &[f64], &mut [f64]) -> FunctionCallResult, // jf1(u, d, result)
    MappingF2: Fn(&[f64], &mut [f64]) -> FunctionCallResult,                      // f2(u, result)
    JacobianMappingF2Trans: Fn(&[f64], &[f64], &mut [f64]) -> FunctionCallResult, // jf2(u, d, result)
    SetC: Constraint,
{
    /// Construct a new instance of `MockFactory`
    ///
    /// # Arguments
    ///
    /// # Example
    ///
    /// ```rust
    /// use optimization_engine::{constraints::Ball2, alm::*, FunctionCallResult};
    ///
    /// let set_c = Ball2::new(None, 1.0);
    /// let n2 = 0;
    ///
    /// let f = |u: &[f64], cost: &mut f64| -> FunctionCallResult { Ok(()) };
    /// let df = |u: &[f64], grad: &mut [f64]| -> FunctionCallResult { Ok(()) };
    /// let f1 = |u: &[f64], f1u: &mut [f64]| -> FunctionCallResult { Ok(()) };
    /// let jf1_tr = |_u: &[f64], d: &[f64], res: &mut [f64]| -> FunctionCallResult { Ok(()) };
    ///
    /// let factory = AlmFactory::new(
    ///     f,
    ///     df,
    ///     Some(f1),
    ///     Some(jf1_tr),
    ///     NO_MAPPING,
    ///     NO_JACOBIAN_MAPPING,
    ///     Some(set_c),
    ///     n2,
    /// );
    /// ```
    ///
    pub fn new(
        f: Cost,
        df: CostGradient,
        mapping_f1: Option<MappingF1>,
        jacobian_mapping_f1_trans: Option<JacobianMappingF1Trans>,
        mapping_f2: Option<MappingF2>,
        jacobian_mapping_f2_trans: Option<JacobianMappingF2Trans>,
        set_c: Option<SetC>,
        n2: usize,
    ) -> Self {
        assert!(
            !(mapping_f2.is_none() ^ (n2 == 0)),
            "if n2 > 0 then and only then should you provide an F2"
        );
        assert!(
            !(jacobian_mapping_f2_trans.is_none() ^ mapping_f2.is_none()),
            "you must have JF2 together with F2"
        );
        assert!(
            !(mapping_f1.is_none() ^ jacobian_mapping_f1_trans.is_none()),
            "if n1 > 0 then and only then should you provide an F1"
        );
        assert!(
            !(mapping_f1.is_none() ^ set_c.is_none()),
            "F1 must be accompanied by a set C"
        );
        AlmFactory {
            f,
            df,
            mapping_f1,
            jacobian_mapping_f1_trans,
            mapping_f2,
            jacobian_mapping_f2_trans,
            set_c,
            n2,
        }
    }

    /// Computes function $\psi$ given by
    ///
    /// $$\psi(u) = f(u) + \tfrac{c}{2}\left[\mathrm{dist}_C^2\left(F_1(u) + \bar{c}^{-1}y\right)
    ///           + \Vert F_2(u) \Vert^2\right],$$
    ///
    /// where $\bar{c} = \max\\{1,c\\}$, $f:\mathbb{R}^{n_u}\to\mathbb{R}$ is a $C^{1,1}$ function,
    /// $F_1:\mathbb{R}^{n_u}\to\mathbb{R}^{n_1}$ and $F_2:\mathbb{R}^{n_u}\to\mathbb{R}^{n_2}$
    /// are smooth mappings, $C\subseteq \mathbb{R}^{n_1}$ is a closed set on which
    /// we can compute projections and $c\in\mathbb{R}$ and $y\in\mathbb{R}^{n_1}$ are the
    /// penalty parameter and vector of Lagrange multipliers respectively.
    ///
    /// # Arguments
    ///
    /// - `u`: vector $u$
    /// - `xi` is the vector $\xi = (c, y) \in \mathbb{R}^{n_1 + 1}$
    /// - `cost`: stores the value of $\psi(u; \xi)$ on exit
    ///
    /// # Returns
    ///
    /// This method returns `Ok(())` if the computation is successful or an appropriate
    /// `SolverError` otherwise.
    ///
    pub fn psi(&self, u: &[f64], xi: &[f64], cost: &mut f64) -> FunctionCallResult {
        (self.f)(u, cost)?;
        let ny = if !xi.is_empty() { xi.len() - 1 } else { 0 };
        let mut f1_u_plus_y_over_c = vec![0.0; ny];
        let mut s = vec![0.0; ny];
        if let (Some(set_c), Some(mapping_f1)) = (&self.set_c, &self.mapping_f1) {
            let penalty_parameter = xi[0];
            mapping_f1(u, &mut f1_u_plus_y_over_c)?; // f1_u = F1(u)
            let y_lagrange_mult = &xi[1..];
            // Note: In the first term below, we divide by 'max(c, 1)', instead of
            //       just 'c'. The reason is that this allows to set c=0 and
            //       retrieve the value of the original cost function
            // f1_u := F1(u) + y/max(1, c)
            f1_u_plus_y_over_c
                .iter_mut()
                .zip(y_lagrange_mult.iter())
                .for_each(|(ti, yi)| *ti += yi / f64::max(penalty_parameter, 1.0));
            s.copy_from_slice(&f1_u_plus_y_over_c);
            set_c.project(&mut s);
            *cost += 0.5
                * penalty_parameter
                * matrix_operations::norm2_squared_diff(&f1_u_plus_y_over_c, &s);
        }
        if let Some(f2) = &self.mapping_f2 {
            let c = xi[0];
            let mut z = vec![0.0; self.n2];
            f2(&u, &mut z)?;
            *cost += 0.5 * c * matrix_operations::norm2_squared(&z);
        }
        Ok(())
    }

    /// Computes the gradient of $\psi$
    ///
    /// The gradient of `psi` is given by
    ///
    /// $$\nabla \psi(u) = \nabla f(u) + c JF_1(u)^\top [t(u) - \Pi_C(t(u))]
    ///                                + c JF_2(u)^\top F_2(u),$$
    ///
    /// where $t(u) = F_1(u) + \max\\{1,c\\}^{-1}y$.
    ///
    /// # Arguments
    ///
    /// - `u`: vector $u$
    /// - `xi` is the vector $\xi = (c, y) \in \mathbb{R}^{n_1 + 1}$
    /// - `grad`: stores the value of $\nabla \psi(u; \xi)$ on exit
    ///
    /// # Returns
    ///
    /// This method returns `Ok(())` if the computation is successful or an appropriate
    /// `SolverError` otherwise.
    ///
    pub fn d_psi(&self, u: &[f64], xi: &[f64], grad: &mut [f64]) -> FunctionCallResult {
        let nu = u.len();

        // The following statement is needed to account for the case where n1=n2=0
        // when xi can be an empty array; we don't have any assertions for that as
        // for now this factory structure is for in-house use and testing only
        let ny = if !xi.is_empty() { xi.len() - 1 } else { 0 };

        (self.df)(u, grad)?; // grad := d_f0(u)

        // Compute the first part: c JF_1(u)^\top [t(u) - \Pi_C(t(u))]
        // (only if the user has provided C, F1 and JF1)
        if let (Some(set_c), Some(mapping_f1), Some(jf1t)) = (
            &self.set_c,
            &self.mapping_f1,
            &self.jacobian_mapping_f1_trans,
        ) {
            let c_penalty_parameter = xi[0];
            let mut f1_u_plus_y_over_c = vec![0.0; ny];
            let mut s_aux_var = vec![0.0; ny]; // auxiliary variable `s`
            let y_lagrange_mult = &xi[1..];
            let mut jac_prod = vec![0.0; nu];
            mapping_f1(&u, &mut f1_u_plus_y_over_c)?; // f1_u_plus_y_over_c = F1(u)
                                                      // f1_u_plus_y_over_c = F1(u) + y/c
            f1_u_plus_y_over_c
                .iter_mut()
                .zip(y_lagrange_mult.iter())
                .for_each(|(ti, yi)| *ti += yi / c_penalty_parameter);
            s_aux_var.copy_from_slice(&f1_u_plus_y_over_c); // s = t
            set_c.project(&mut s_aux_var); // s = Proj_C(F1(u) + y/c)

            // t = F1(u) + y/c - Proj_C(F1(u) + y/c)
            f1_u_plus_y_over_c
                .iter_mut()
                .zip(s_aux_var.iter())
                .for_each(|(ti, si)| *ti -= si);

            jf1t(&u, &f1_u_plus_y_over_c, &mut jac_prod)?;

            // grad += c*t
            grad.iter_mut()
                .zip(jac_prod.iter())
                .for_each(|(gradi, jac_prodi)| *gradi += c_penalty_parameter * jac_prodi);
        }

        // Compute second part: JF2(u)'*F2(u)
        if let (Some(f2), Some(jf2)) = (&self.mapping_f2, &self.jacobian_mapping_f2_trans) {
            let c = xi[0];
            let mut f2u_aux = vec![0.0; self.n2];
            let mut jf2u_times_f2u_aux = vec![0.0; self.n2];
            f2(&u, &mut f2u_aux)?; // f2u_aux = F2(u)
            jf2(&u, &f2u_aux, &mut jf2u_times_f2u_aux)?; // jf2u_times_f2u_aux = JF2(u)'*f2u_aux
                                                         //                    = JF2(u)'*F2(u)

            // grad += c * jf2u_times_f2u_aux
            grad.iter_mut()
                .zip(jf2u_times_f2u_aux.iter())
                .for_each(|(gradi, jf2u_times_f2u_aux_i)| *gradi += c * jf2u_times_f2u_aux_i);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use crate::{alm::*, constraints::*, mocks, FunctionCallResult, SolverError};

    #[test]
    fn t_mocking_alm_factory_psi() {
        let set_c = Ball2::new(None, 1.0);
        let factory = AlmFactory::new(
            mocks::f0,
            mocks::d_f0,
            Some(mocks::mapping_f1_affine),
            Some(mocks::mapping_f1_affine_jacobian_product),
            NO_MAPPING,
            NO_JACOBIAN_MAPPING,
            Some(set_c),
            0,
        );
        let u = [3.0, 5.0, 7.0];
        let xi = [2.0, 10.0, 20.0];
        let mut cost = 0.0;
        assert!(factory.psi(&u, &xi, &mut cost).is_ok());
        println!("cost = {}", cost);
        unit_test_utils::assert_nearly_equal(
            1_064.986_642_583_36,
            cost,
            1e-14,
            1e-10,
            "psi is wrong",
        );
    }

    #[test]
    fn t_mocking_alm_factory_grad_psi() {
        let set_c = Ball2::new(None, 1.0);
        let factory = AlmFactory::new(
            mocks::f0,
            mocks::d_f0,
            Some(mocks::mapping_f1_affine),
            Some(mocks::mapping_f1_affine_jacobian_product),
            NO_MAPPING,
            NO_JACOBIAN_MAPPING,
            Some(set_c),
            0,
        );
        let u = [3.0, -5.0, 7.0];
        let xi = [2.5, 11.0, 20.0];
        let mut grad_psi = [0.0; 3];
        assert!(factory.d_psi(&u, &xi, &mut grad_psi).is_ok());
        unit_test_utils::assert_nearly_equal_array(
            &[
                71.734_788_756_561_9,
                -32.222_828_648_567_5,
                46.571_199_153_042_2,
            ],
            &grad_psi,
            1e-12,
            1e-12,
            "d_psi is wrong",
        );
    }

    fn mapping_f2(u: &[f64], res: &mut [f64]) -> FunctionCallResult {
        res[0] = u[0];
        res[1] = u[1];
        res[2] = u[2] - u[0];
        res[3] = u[2] - u[0] - u[1];
        Ok(())
    }

    fn jac_mapping_f2_tr(_u: &[f64], d: &[f64], res: &mut [f64]) -> Result<(), crate::SolverError> {
        res[0] = d[0] - d[2] - d[3];
        res[1] = d[1] - d[3];
        res[2] = d[2] + d[3];
        Ok(())
    }

    #[test]
    fn t_mocking_alm_factory_psi_with_f2() {
        let set_c = Ball2::new(None, 1.0);
        let f2 = mapping_f2;
        let jac_f2_tr =
            |_u: &[f64], _d: &[f64], _res: &mut [f64]| -> Result<(), crate::SolverError> {
                Err(SolverError::NotFiniteComputation)
            };
        let factory = AlmFactory::new(
            mocks::f0,
            mocks::d_f0,
            Some(mocks::mapping_f1_affine),
            Some(mocks::mapping_f1_affine_jacobian_product),
            Some(f2),
            Some(jac_f2_tr),
            Some(set_c),
            4,
        );
        let u = [3.0, 5.0, 7.0];
        let xi = [2.0, 10.0, 20.0];
        let mut cost = 0.0;
        assert!(factory.psi(&u, &xi, &mut cost).is_ok());
        println!("cost = {}", cost);
        unit_test_utils::assert_nearly_equal(
            1.115_986_642_583_36e+03,
            cost,
            1e-12,
            1e-10,
            "psi is wrong",
        );
    }

    #[test]
    fn t_mocking_alm_factory_grad_psi_with_f2() {
        let set_c = Ball2::new(None, 1.0);
        let factory = AlmFactory::new(
            mocks::f0,
            mocks::d_f0,
            Some(mocks::mapping_f1_affine),
            Some(mocks::mapping_f1_affine_jacobian_product),
            Some(mapping_f2),
            Some(jac_mapping_f2_tr),
            Some(set_c),
            4,
        );
        let u = [3.0, 5.0, 7.0];
        let xi = [2.0, 10.0, 20.0];
        let mut grad_psi = [0.0; 3];
        assert!(factory.d_psi(&u, &xi, &mut grad_psi).is_ok());
        println!("grad = {:#?}", &grad_psi);
        unit_test_utils::assert_nearly_equal_array(
            &[
                124.214_512_432_589_49,
                180.871_274_908_669_62,
                46.962_043_731_516_474,
            ],
            &grad_psi,
            1e-12,
            1e-12,
            "d_psi is wrong",
        );
    }

    #[test]
    fn t_mocking_alm_factory_nomappings() {
        let factory = AlmFactory::new(
            mocks::f0,
            mocks::d_f0,
            NO_MAPPING,
            NO_JACOBIAN_MAPPING,
            NO_MAPPING,
            NO_JACOBIAN_MAPPING,
            NO_SET,
            0,
        );
        let u = [3.0, 5.0, 7.0];
        let xi = [];
        let mut grad_psi = [0.0; 3];
        assert!(factory.d_psi(&u, &xi, &mut grad_psi).is_ok());
        println!("grad = {:#?}", &grad_psi);
    }
}
