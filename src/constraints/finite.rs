use super::Constraint;
use crate::FunctionCallResult;
use num::Float;
use std::iter::Sum;

fn norm2_squared_diff<T: Float>(a: &[T], b: &[T]) -> T {
    assert_eq!(a.len(), b.len());
    a.iter()
        .zip(b.iter())
        .fold(T::zero(), |sum, (&x, &y)| sum + (x - y) * (x - y))
}

///
/// A finite set, $X = \\{x_1, x_2, \ldots, x_n\\}\subseteq\mathbb{R}^n$, given vectors
/// $x_i\in\mathbb{R}^n$
///
#[derive(Clone, Copy)]
pub struct FiniteSet<'a, T = f64> {
    /// The data is stored in a Vec-of-Vec datatype, that is, a vector
    /// of vectors
    data: &'a [&'a [T]],
}

impl<'a, T> FiniteSet<'a, T>
where
    T: Float + Sum<T>,
{
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
    /// This method will panic if the given vector of data is empty,
    /// or if the given vectors have unequal dimensions.
    ///
    pub fn new(data: &'a [&'a [T]]) -> Self {
        // Do a sanity check...
        assert!(!data.is_empty(), "empty data not allowed");
        let n = data[0].len();
        for v in data.iter() {
            assert!(n == v.len(), "inconsistent dimensions");
        }
        FiniteSet { data }
    }
}

impl<'a, T> Constraint<T> for FiniteSet<'a, T>
where
    T: Float + Sum<T>,
{
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
    /// finite_set.project(&mut x).unwrap(); // compute projection
    /// ```
    ///
    /// # Panics
    ///
    /// This method panics if the dimension of `x` is not equal to the
    /// dimension of the points in the finite set.
    ///
    fn project(&self, x: &mut [T]) -> FunctionCallResult {
        assert_eq!(x.len(), self.data[0].len(), "x has incompatible dimension");
        let mut idx: usize = 0;
        let mut best_distance = T::infinity();
        for (i, v) in self.data.iter().enumerate() {
            let dist = norm2_squared_diff(v, x);
            if dist < best_distance {
                idx = i;
                best_distance = dist;
            }
        }
        x.copy_from_slice(self.data[idx]);
        Ok(())
    }

    fn is_convex(&self) -> bool {
        self.data.len() == 1
    }
}
