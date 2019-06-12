use super::Constraint;

/// Cartesian product of constraints
pub struct CartesianProduct<'a> {
    idx: Vec<usize>,
    constraints: Vec<&'a Constraint>,
}

impl<'a> CartesianProduct<'a> {
    pub fn new() -> Self {
        let mut fresh_idx_list: Vec<usize> = Vec::new();
        CartesianProduct {
            idx: fresh_idx_list,
            constraints: Vec::new(),
        }
    }

    pub fn dimension(&self) -> usize {
        *self.idx.last().unwrap_or(&0)
    }

    pub fn add_constraint(&mut self, i: usize, constraint: &'a Constraint) {
        assert!(self.dimension() < i);
        self.idx.push(i);
        self.constraints.push(constraint);
    }
}

impl<'a> Constraint for CartesianProduct<'a> {
    fn project(&self, x: &mut [f64]) {
        let mut j = 0;
        self.idx
            .iter()
            .zip(self.constraints.iter())
            .for_each(|(&i, c)| {
                c.project(&mut x[j..i]);
                j = i;
            });
    }
}
