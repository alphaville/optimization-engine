use super::Constraint;
use crate::FunctionCallResult;

/// The whole space, no constraints
#[derive(Default, Clone, Copy)]
pub struct NoConstraints {}

impl NoConstraints {
    /// Constructs new instance of `NoConstraints`
    ///
    /// # Example
    ///
    /// ```
    /// use optimization_engine::constraints::{Constraint, NoConstraints};
    ///
    /// let no_constraints = NoConstraints::new();
    /// let mut x = [1.0, -2.0, 3.0];
    /// no_constraints.project(&mut x).unwrap();
    /// ```
    ///
    #[must_use]
    pub fn new() -> NoConstraints {
        NoConstraints {}
    }
}

impl<T> Constraint<T> for NoConstraints {
    fn project(&self, _x: &mut [T]) -> FunctionCallResult {
        Ok(())
    }

    fn is_convex(&self) -> bool {
        true
    }
}
