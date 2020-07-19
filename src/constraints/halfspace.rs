use super::Constraint;
use crate::matrix_operations;

#[derive(Clone)]
/// A halfspace is a set given by $H = \\{x \in \mathbb{R}^n {}:{} \langle c, x\rangle \leq b\\}$.
pub struct Halfspace<'a> {
    normal_vector: &'a [f64],
    offset: f64,
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
    /// # Panics
    ///
    /// Does not panic. Note: it does not panic if you provide an empty slice as `normal_vector`,
    /// but you should avoid doing that.
    ///
    /// # Example
    ///
    /// ```
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
    fn project(&self, x: &mut [f64]) {
        let inner_product = matrix_operations::inner_product(x, self.normal_vector);
        if inner_product > self.offset {
            let factor = (inner_product - self.offset) / self.normal_vector_squared_norm;
            x.iter_mut()
                .zip(self.normal_vector.iter())
                .for_each(|(x, nrm_vct)| *x -= factor * nrm_vct);
        }
    }

    fn is_convex(&self) -> bool {
        true
    }
}
