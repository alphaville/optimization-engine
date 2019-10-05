use super::Constraint;

/// Cartesian product of constraints
///
/// Cartesian product of constraints, `C(0), C(1), ..., C(n-1)`, which is defined as a set
/// `C = C(0) x C(1) x ... x C(n-1)`, for some integer `n>1`. Sets `C(i)` are structures which
/// implement the trait `Constraint`.
///
/// In an n-dimensional space, a vector `x` is split in parts `x = (x(0), x(1), ..., x(n-1))`,
/// where `x(i)` has dimension `n(i)`. The constraint `x in C` is interpreted as `x(i) in C(i)`
/// for all `i=0,...,n-1`.
///
pub struct CartesianProduct<'a> {
    idx: Vec<usize>,
    constraints: Vec<Box<dyn Constraint + 'a>>,
}

impl<'a> CartesianProduct<'a> {
    /// Construct new instance of Cartesian product of constraints
    ///
    pub fn new() -> Self {
        CartesianProduct {
            idx: Vec::new(),
            constraints: Vec::new(),
        }
    }

    /// Dimension of the current constraints
    pub fn dimension(&self) -> usize {
        *self.idx.last().unwrap_or(&0)
    }

    /// Add constraint `x(i) in C(i)`
    ///
    /// Vector `x` is segmented into subvectors `x = (x(0), x(1), ..., x(n-1)`, where
    /// `x(0)` has length `n0`.
    ///
    ///
    /// # Arguments
    ///
    /// - `ni`: total length of vector `(x(0), ..., x(i))` (see example below)
    /// - `constraint`: constraint to be added implementation of trait `Constraint`
    ///
    ///
    /// # Returns
    ///
    /// Returns the current mutable and updated instance of the provided object
    ///
    /// # Example
    ///
    /// ```rust
    /// use optimization_engine::constraints::*;
    ///
    /// /*
    ///  * Cartesian product of two balls of dimensions 3 and 2,
    ///  * that is, x = (x0, x1), with x0 being 3-dimensional and
    ///  * x1 being 2-dimensional, that is, x0 = (x[0], x[1], x[2])
    ///  * and x2 = (x[3], x[4]).
    ///  */
    /// let idx1 = 3;
    /// let idx2 = 5;
    /// let ball1 = Ball2::new(None, 1.0);
    /// let ball2 = Ball2::new(None, 0.5);
    /// let mut cart_prod = CartesianProduct::new()
    ///     .add_constraint(idx1, ball1)
    ///     .add_constraint(idx2, ball2);
    /// ```
    ///
    /// # Panics
    ///
    /// The method panics if `ni` is less than or equal to the previous
    /// dimension of the cartesian product. For example, the following
    /// code will fail:
    ///
    /// ```compile_fail
    /// let mut cart_prod = CartesianProduct::new()
    ///     .add_constraint(7, &rectangle);     // OK, since 7  > 0
    ///     .add_constraint(10, &ball1);        // OK, since 10 > 7
    ///     .add_constraint(2, &ball3);         // 2 <= 10, so it will fail
    /// ```
    /// The method will panic if any of the associated projections panics.
    ///
    pub fn add_constraint(mut self, ni: usize, constraint: impl Constraint + 'a) -> Self {
        assert!(
            self.dimension() < ni,
            "provided index is smaller than or equal to previous index, or zero"
        );
        self.idx.push(ni);
        self.constraints.push(Box::new(constraint));
        self
    }
}

impl<'a> Constraint for CartesianProduct<'a> {
    /// Project onto Cartesian product of constraints
    ///
    /// The given vector `x` is updated with the projection on the set
    ///
    /// # Panics
    ///
    /// The method will panic if the dimension of `x` is not equal to the
    /// dimension of the Cartesian product (see `dimension()`)
    fn project(&self, x: &mut [f64]) {
        assert!(x.len() == self.dimension(), "x has wrong size");
        let mut j = 0;
        self.idx
            .iter()
            .zip(self.constraints.iter())
            .for_each(|(&i, c)| {
                c.project(&mut x[j..i]);
                j = i;
            });
    }

    fn is_convex(&self) -> bool {
        return self.constraints.iter().fold(true, |mut flag, cnstr| {
            flag &= cnstr.is_convex();
            flag
        });
    }
}
