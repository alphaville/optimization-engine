use super::Constraint;

/// The whole space, no constraints
pub struct NoConstraints {}

impl NoConstraints {
    pub fn new() -> NoConstraints {
        NoConstraints {}
    }
}

impl Constraint for NoConstraints {
    fn project(&self, _x: &mut [f64]) {}
}
