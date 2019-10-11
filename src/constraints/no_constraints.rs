use super::Constraint;

/// The whole space, no constraints
#[derive(Default, Clone, Copy)]
pub struct NoConstraints {}

impl NoConstraints {
    /// Constructs new instance of `NoConstraints`
    ///
    pub fn new() -> NoConstraints {
        NoConstraints {}
    }
}

impl Constraint for NoConstraints {
    fn project(&self, _x: &mut [f64]) {}

    fn is_convex(&self) -> bool {
        true
    }
}
