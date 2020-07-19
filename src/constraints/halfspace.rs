use super::Constraint;

#[derive(Clone)]
/// Lala
pub struct Halfspace<'a> {
    normal_vector: &'a [f64],
    offset: f64,
}

impl<'a> Halfspace<'a> {
    /// new
    pub fn new(normal_vector: &'a [f64], offset: f64) -> Self {
        Halfspace {
            normal_vector,
            offset,
        }
    }
}

impl<'a> Constraint for Halfspace<'a> {
    fn project(&self, _x: &mut [f64]) {}

    fn is_convex(&self) -> bool {
        true
    }
}
