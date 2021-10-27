use super::Constraint;

#[derive(Copy, Clone)]
///explain
pub struct Simplex {
    alpha: f64,
}

impl Simplex {
    /// explain
    pub fn new(alpha: f64) -> Self {
        assert!(alpha > 0.0);
        Simplex { alpha }
    }
}

impl Constraint for Simplex {
    fn project(&self, x: &mut [f64]) {}

    fn is_convex(&self) -> bool {
        true
    }
}

