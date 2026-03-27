use super::Constraint;
use crate::FunctionCallResult;
use num::Float;

#[derive(Clone, Copy, Default)]
/// Set Zero, $\\{0\\}$
pub struct Zero {}

impl Zero {
    /// Constructs new instance of `Zero`
    ///
    /// # Example
    ///
    /// ```
    /// use optimization_engine::constraints::{Constraint, Zero};
    ///
    /// let zero = Zero::new();
    /// let mut x = [1.0, -2.0, 3.0];
    /// zero.project(&mut x).unwrap();
    /// ```
    #[must_use]
    pub fn new() -> Self {
        Zero {}
    }
}

impl<T: Float> Constraint<T> for Zero {
    /// Computes the projection on $\\{0\\}$, that is, $\Pi_{\\{0\\}}(x) = 0$
    /// for all $x$
    fn project(&self, x: &mut [T]) -> FunctionCallResult {
        x.iter_mut().for_each(|xi| *xi = T::zero());
        Ok(())
    }

    fn is_convex(&self) -> bool {
        true
    }
}
