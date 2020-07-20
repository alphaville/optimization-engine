use super::Constraint;
use crate::matrix_operations;

#[derive(Clone)]
/// A halfspace is a set given by $H = \\{x \in \mathbb{R}^n {}:{} \langle c, x\rangle \leq b\\}$.
pub struct Halfspace<'a> {
    /// normal vector
    normal_vector: &'a [f64],
    /// offset
    offset: f64,
    /// squared Euclidean norm of the normal vector (computed once upon construction)
    normal_vector_squared_norm: f64,
}

impl<'a> Halfspace<'a> {
    /// A halfspace is a set given by $H = \\{x \in \mathbb{R}^n {}:{} \langle c, x\rangle \leq b\\}$,
    /// where $c$ is the normal vector of the halfspace and $b$ is an offset.
    ///
    /// This method constructs a new instance of `Halfspace` with a given normal
    /// vector and bias
    ///
    /// # Arguments
    ///
    /// - `normal_vector`: the normal vector, $c$, as a slice
    /// - `offset`: the offset parameter, $b$
    ///
    /// # Returns
    ///
    /// New instance of `Halfspace`
    ///
    /// # Panics
    ///
    /// Does not panic. Note: it does not panic if you provide an empty slice as `normal_vector`,
    /// but you should avoid doing that.
    ///
    /// # Example
    ///
    /// ```
    /// use optimization_engine::constraints::{Constraint, Halfspace};
    ///
    /// let normal_vector = [1., 2.];
    /// let offset = 1.0;
    /// let halfspace = Halfspace::new(&normal_vector, offset);
    /// let mut x = [-1., 3.];
    /// halfspace.project(&mut x);
    /// ```
    ///
    pub fn new(normal_vector: &'a [f64], offset: f64) -> Self {
        let normal_vector_squared_norm = matrix_operations::norm2_squared(normal_vector);
        Halfspace {
            normal_vector,
            offset,
            normal_vector_squared_norm,
        }
    }
}

impl<'a> Constraint for Halfspace<'a> {
    /// Projects on halfspace using the following formula:
    ///
    /// $$\begin{aligned}
    /// \mathrm{proj}_{H}(x) = \begin{cases}
    /// x,& \text{ if } \langle c, x\rangle \leq b
    /// \\\\
    /// x - \frac{\langle c, x\rangle - b}
    ///          {\\|c\\|}c,& \text{else}
    /// \end{cases}
    /// \end{aligned}$$
    ///
    /// where $H = \\{x \in \mathbb{R}^n {}:{} \langle c, x\rangle \leq b\\}$
    ///
    /// # Arguments
    ///
    /// - `x`: (in) vector to be projected on the current instance of a halfspace,
    ///    (out) projection on the second-order cone
    ///
    /// # Panics
    ///
    /// This method panics if the length of `x` is not equal to the dimension
    /// of the halfspace.
    ///
    fn project(&self, x: &mut [f64]) {
        let inner_product = matrix_operations::inner_product(x, self.normal_vector);
        if inner_product > self.offset {
            let factor = (inner_product - self.offset) / self.normal_vector_squared_norm;
            x.iter_mut()
                .zip(self.normal_vector.iter())
                .for_each(|(x, normal_vector_i)| *x -= factor * normal_vector_i);
        }
    }

    /// Halfspaces are convex sets
    ///
    /// # Returns
    ///
    /// Returns `true`
    fn is_convex(&self) -> bool {
        true
    }
}
