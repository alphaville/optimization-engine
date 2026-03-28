use super::Constraint;
use crate::matrix_operations;
use crate::FunctionCallResult;
use num::Float;
use std::iter::Sum;

#[derive(Clone)]
/// A hyperplane is a set given by $H = \\{x \in \mathbb{R}^n {}:{} \langle c, x\rangle = b\\}$.
pub struct Hyperplane<'a, T = f64> {
    /// normal vector
    normal_vector: &'a [T],
    /// offset
    offset: T,
    /// squared Euclidean norm of the normal vector (computed once upon construction)
    normal_vector_squared_norm: T,
}

impl<'a, T> Hyperplane<'a, T>
where
    T: Float + Sum<T>,
{
    /// A hyperplane is a set given by $H = \\{x \in \mathbb{R}^n {}:{} \langle c, x\rangle = b\\}$,
    /// where $c$ is the normal vector of the hyperplane and $b$ is an offset.
    ///
    /// This method constructs a new instance of `Hyperplane` with a given normal
    /// vector and offset.
    ///
    /// # Arguments
    ///
    /// - `normal_vector`: the normal vector, $c$, as a slice
    /// - `offset`: the offset parameter, $b$
    ///
    /// # Returns
    ///
    /// New instance of `Hyperplane`
    ///
    /// # Panics
    ///
    /// This method panics if the normal vector has zero Euclidean norm.
    ///
    /// # Example
    ///
    /// ```
    /// use optimization_engine::constraints::{Constraint, Hyperplane};
    ///
    /// let normal_vector = [1., 2.];
    /// let offset = 1.0;
    /// let hyperplane = Hyperplane::new(&normal_vector, offset);
    /// let mut x = [-1., 3.];
    /// hyperplane.project(&mut x).unwrap();
    /// ```
    ///
    pub fn new(normal_vector: &'a [T], offset: T) -> Self {
        let normal_vector_squared_norm = matrix_operations::norm2_squared(normal_vector);
        assert!(
            normal_vector_squared_norm > T::zero(),
            "normal_vector must have positive norm"
        );
        Hyperplane {
            normal_vector,
            offset,
            normal_vector_squared_norm,
        }
    }
}

impl<'a, T> Constraint<T> for Hyperplane<'a, T>
where
    T: Float + Sum<T>,
{
    /// Projects on the hyperplane using the formula:
    ///
    /// $$\begin{aligned}
    /// \mathrm{proj}_{H}(x) =
    /// x - \frac{\langle c, x\rangle - b}
    ///          {\\|c\\|^2}c.
    /// \end{aligned}$$
    ///
    /// where $H = \\{x \in \mathbb{R}^n {}:{} \langle c, x\rangle = b\\}$
    ///
    /// # Arguments
    ///
    /// - `x`: (in) vector to be projected on the current instance of a hyperplane,
    ///   (out) projection on the hyperplane
    ///
    /// # Panics
    ///
    /// This method panics if the length of `x` is not equal to the dimension
    /// of the hyperplane.
    ///
    fn project(&self, x: &mut [T]) -> FunctionCallResult {
        assert_eq!(x.len(), self.normal_vector.len(), "x has wrong dimension");
        let inner_product = matrix_operations::inner_product(x, self.normal_vector);
        let factor = (inner_product - self.offset) / self.normal_vector_squared_norm;
        x.iter_mut()
            .zip(self.normal_vector.iter())
            .for_each(|(x, nrm_vct)| *x = *x - factor * *nrm_vct);
        Ok(())
    }

    /// Hyperplanes are convex sets
    ///
    /// # Returns
    ///
    /// Returns `true`
    fn is_convex(&self) -> bool {
        true
    }
}
