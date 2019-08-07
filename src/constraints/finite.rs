use super::Constraint;

///
/// A finite set, `X = {x1, x2, ..., xn}`, where `xi` are given vectors
/// of equal dimensions.
///
pub struct FiniteSet {
    data: Vec<Vec<f64>>,
}

impl FiniteSet {
    /// Construct a finite set
    ///
    /// ### Parameters
    ///
    ///
    pub fn new(data: Vec<Vec<f64>>) -> FiniteSet {
        // Do a sanity check...
        assert!(data.len() > 0, "empty data not allowed");
        let n = data[0].len();
        for v in data.iter() {
            assert!(n == v.len(), "inconsistent dimensions");
        }
        FiniteSet { data: data }
    }
}

impl<'a> Constraint for FiniteSet {
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
}
