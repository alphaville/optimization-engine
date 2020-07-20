use super::Constraint;

///
/// A finite set, $X = \\{x_1, x_2, \ldots, x_n\\}\subseteq\mathbb{R}^n$, given vectors
/// $x_i\in\mathbb{R}^n$
///
#[derive(Clone, Copy)]
pub struct FiniteSet<'a> {
    /// The data is stored in a Vec-of-Vec datatype, that is, a vector
    /// of vectors
    data: &'a [&'a [f64]],
}

impl<'a> FiniteSet<'a> {
    /// Construct a finite set, $X = \\{x_1, x_2, \ldots, x_n\\}$, given vectors
    /// $x_i\in\mathbb{R}^n$
    ///
    ///
    /// # Arguments
    ///
    /// - data: vector of vectors (see example below)
    ///
    ///
    /// # Example
    ///
    /// ```
    /// use optimization_engine::constraints::{Constraint, FiniteSet};
    ///
    /// let data: &[&[f64]] = &[
    ///    &[1.0, 1.0],
    ///    &[0.0, 1.0],
    ///    &[1.0, 0.0],
    ///    &[0.0, 0.0],
    /// ];
    /// let finite_set = FiniteSet::new(data);
    /// ```
    ///
    ///
    /// # Panics
    ///
    /// This method will panic if (i) the given vector of data is empty
    /// and (ii) if the given vectors have unequal dimensions.
    ///
    pub fn new(data: &'a [&'a [f64]]) -> Self {
        // Do a sanity check...
        assert!(!data.is_empty(), "empty data not allowed");
        let n = data[0].len();
        for v in data.iter() {
            assert!(n == v.len(), "inconsistent dimensions");
        }
        FiniteSet { data }
    }
}

impl<'a> Constraint for FiniteSet<'a> {
    ///
    /// Projection on the current finite set
    ///
    /// Traverses the elements of the vector, computes norm-2 distances
    /// to each element, and updates the given vector `x` with the closest
    /// element from the finite set.
    ///
    ///
    /// # Parameters
    ///
    /// - `x`: (input) given vector, (output) projection on finite set
    ///
    ///
    /// # Example
    ///
    /// ```
    /// use optimization_engine::constraints::*;
    ///
    /// let data: &[&[f64]] = &[
    ///    &[0.0, 0.0],
    ///    &[1.0, 1.0],
    /// ];
    /// let finite_set = FiniteSet::new(data);
    /// let mut x = [0.7, 0.6];
    /// finite_set.project(&mut x); // compute projection
    /// ```
    ///
    /// # Panics
    ///
    /// Does not panic
    ///
    fn project(&self, x: &mut [f64]) {
        let mut idx: usize = 0;
        let mut best_distance: f64 = num::Float::infinity();
        for (i, v) in self.data.iter().enumerate() {
            let dist = crate::matrix_operations::norm2_squared_diff(v, x);
            if dist < best_distance {
                idx = i;
                best_distance = dist;
            }
        }
        x.copy_from_slice(&self.data[idx]);
    }

    fn is_convex(&self) -> bool {
        self.data.len() == 1 && !self.data[0].is_empty()
    }
}
