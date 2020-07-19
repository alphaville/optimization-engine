use super::Constraint;
use crate::matrix_operations;

#[derive(Clone)]
/// Lala
pub struct Halfspace<'a> {
    normal_vector: &'a [f64],
    offset: f64,
    normal_vector_squared_norm: f64,
}

impl<'a> Halfspace<'a> {
    /// new
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
            let c = (inner_product - self.offset) / self.normal_vector_squared_norm;
            x.iter_mut()
                .zip(self.normal_vector.iter())
                .for_each(|(x, nrm_vct)| *x -= c * nrm_vct);
        }
    }

    fn is_convex(&self) -> bool {
        true
    }
}
