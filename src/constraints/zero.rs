use super::Constraint;
use num::Float;

#[derive(Clone, Copy, Default)]
/// Set Zero, $\\{0\\}$
pub struct Zero {}

impl Zero {
    /// Constructs new instance of `Zero`
    #[must_use]
    pub fn new() -> Self {
        Zero {}
    }
}

impl<T: Float> Constraint<T> for Zero {
    /// Computes the projection on $\\{0\\}$, that is, $\Pi_{\\{0\\}}(x) = 0$
    /// for all $x$
    fn project(&self, x: &mut [T]) {
        x.iter_mut().for_each(|xi| *xi = T::zero());
    }

    fn is_convex(&self) -> bool {
        true
    }
}
