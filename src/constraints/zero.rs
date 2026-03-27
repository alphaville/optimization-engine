use super::Constraint;
use crate::FunctionCallResult;

#[derive(Clone, Copy, Default)]
/// Set Zero, $\\{0\\}$
pub struct Zero {}

impl Zero {
    /// Constructs new instance of `Zero`
    pub fn new() -> Self {
        Zero {}
    }
}

impl Constraint for Zero {
    /// Computes the projection on $\\{0\\}$, that is, $\Pi_{\\{0\\}}(x) = 0$
    /// for all $x$
    fn project(&self, x: &mut [f64]) -> FunctionCallResult {
        x.iter_mut().for_each(|xi| *xi = 0.0);
        Ok(())
    }

    fn is_convex(&self) -> bool {
        true
    }
}
