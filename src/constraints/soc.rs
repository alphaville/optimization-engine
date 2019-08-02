use super::Constraint;

///
/// A second-order cone (SOC)
///
/// A set of the form `{ (x, t) : ||x|| <= a*t}`, where `a` is a positive
/// scalar
pub struct SecondOrderCone {
    alpha: f64,
}

impl SecondOrderCone {
    /// Construct a new rectangle with given `xmin` and `xmax`
    pub fn new(alpha: f64) -> SecondOrderCone {
        assert!(alpha > 0.0); // alpha must be positive
        SecondOrderCone { alpha }
    }
}

impl Constraint for SecondOrderCone {
    fn project(&self, _x: &mut [f64]) {}
}
