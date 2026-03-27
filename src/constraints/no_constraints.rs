use super::Constraint;

/// The whole space, no constraints
#[derive(Default, Clone, Copy)]
pub struct NoConstraints {}

impl NoConstraints {
    /// Constructs new instance of `NoConstraints`
    ///
    #[must_use]
    pub fn new() -> NoConstraints {
        NoConstraints {}
    }
}

impl<T> Constraint<T> for NoConstraints {
    fn project(&self, _x: &mut [T]) {}

    fn is_convex(&self) -> bool {
        true
    }
}
