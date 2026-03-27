use super::Constraint;
use crate::FunctionCallResult;

/// Cartesian product of constraints.
///
/// Cartesian product of constraints, $C_0, C_1, \ldots, C_{n-1}$,
/// which is defined as a set
///
/// $$
/// C = C_0 \times C_1 \times \ldots \times C_{n-1},
/// $$
///
/// for some integer $n>1$. Sets $C_i$ are structures which
/// implement the trait `Constraint`.
///
/// In an $n$-dimensional space, a vector $x$ is split in parts
///
/// $$x = (x_0, x_1, ..., x_{n-1}),$$
///
/// where $x_i$ has dimension $n_i$.
///
/// The constraint $x \in C$ is interpreted as $x_i \in C_i$
/// for all $i=0,\ldots, n-1$.
///
#[derive(Default)]
pub struct CartesianProduct<'a> {
    idx: Vec<usize>,
    constraints: Vec<Box<dyn Constraint + 'a>>,
}

impl<'a> CartesianProduct<'a> {
    /// Construct a new Cartesian product of constraints.
    ///
    /// # Note
    ///
    /// The use of `new_with_capacity` should be preferred over this method,
    /// when possible (provided you have an estimate of the number of sets
    /// your Cartesian product will consist of).
    ///
    pub fn new() -> Self {
        CartesianProduct {
            idx: Vec::new(),
            constraints: Vec::new(),
        }
    }

    /// Construct a new Cartesian product with a given capacity.
    ///
    /// # Arguments
    ///
    /// - `num_sets`: number of sets; this is used to allocate initial memory
    ///   (via `Vec::with_capacity`).
    ///
    /// # Returns
    ///
    /// A new instance of `CartesianProduct`.
    ///
    pub fn new_with_capacity(num_sets: usize) -> Self {
        CartesianProduct {
            idx: Vec::with_capacity(num_sets),
            constraints: Vec::with_capacity(num_sets),
        }
    }

    /// Dimension of the current constraints
    pub fn dimension(&self) -> usize {
        *self.idx.last().unwrap_or(&0)
    }

    /// Add a constraint `x_i \in C_i`.
    ///
    /// Vector `x` is segmented into subvectors
    /// $x = (x_0, x_1, \ldots, x_{n-1})$, where `x_0` has length `n_0`,
    /// `x_1` has length `n_1`, and so on.
    ///
    ///
    /// # Arguments
    ///
    /// - `ni`: total length of the vector `(x_0, \ldots, x_i)` (see example below)
    /// - `constraint`: constraint to add; it must implement the trait `Constraint`
    ///
    ///
    /// # Returns
    ///
    /// The updated Cartesian product.
    ///
    /// # Example
    ///
    /// ```rust
    /// use optimization_engine::constraints::*;
    ///
    /// /*
    ///  * Cartesian product of two balls of dimensions 3 and 2,
    ///  * that is, `x = (x0, x1)`, with `x0` being 3-dimensional and
    ///  * `x1` being 2-dimensional, so `x0 = (x[0], x[1], x[2])`
    ///  * and `x1 = (x[3], x[4])`.
    ///  */
    /// let idx1 = 3;
    /// let idx2 = 5;
    /// let ball1 = Ball2::new(None, 1.0);
    /// let ball2 = Ball2::new(None, 0.5);
    /// let cart_prod = CartesianProduct::new()
    ///     .add_constraint(idx1, ball1)
    ///     .add_constraint(idx2, ball2);
    /// ```
    ///
    /// # Panics
    ///
    /// The method panics if `ni` is less than or equal to the previous
    /// dimension of the Cartesian product. For example, the following
    /// code will fail:
    ///
    /// ```should_panic
    /// use optimization_engine::constraints::*;
    ///
    /// let rectangle = Rectangle::new(Some(&[-1.0; 7]), Some(&[1.0; 7]));
    /// let ball1 = Ball2::new(None, 1.0);
    /// let ball3 = Ball2::new(None, 1.0);
    ///
    /// let _cart_prod = CartesianProduct::new()
    ///     .add_constraint(7, rectangle)   // OK, since 7 > 0
    ///     .add_constraint(10, ball1)      // OK, since 10 > 7
    ///     .add_constraint(2, ball3);      // 2 <= 10, so it will panic
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
    /// Project onto the Cartesian product of constraints.
    ///
    /// The given vector `x` is updated with the projection on the set
    ///
    /// # Panics
    ///
    /// The method will panic if the dimension of `x` is not equal to the
    /// dimension of the Cartesian product (see `dimension()`)
    fn project(&self, x: &mut [f64]) -> FunctionCallResult {
        assert!(x.len() == self.dimension(), "x has wrong size");
        let mut j = 0;
        for (&i, c) in self.idx.iter().zip(self.constraints.iter()) {
            c.project(&mut x[j..i])?;
            j = i;
        }
        Ok(())
    }

    fn is_convex(&self) -> bool {
        self.constraints.iter().fold(true, |mut flag, cnstr| {
            flag &= cnstr.is_convex();
            flag
        })
    }
}
