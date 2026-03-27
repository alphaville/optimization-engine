use super::Constraint;
use num::Float;
use crate::FunctionCallResult;

#[derive(Clone, Copy)]
///
/// A rectangle, $R = \\{x \in \mathbb{R}^n {}:{} x_{\min} {}\leq{} x {}\leq{} x_{\max}\\}$
///
/// A set of the form $\\{x \in \mathbb{R}^n {}:{} x_{\min} {}\leq{} x {}\leq{} x_{\max}\\}$,
/// where $\leq$ is meant in the element-wise sense and either of $x_{\min}$ and $x_{\max}$ can
/// be equal to infinity.
pub struct Rectangle<'a, T = f64> {
    xmin: Option<&'a [T]>,
    xmax: Option<&'a [T]>,
}

impl<'a, T: Float> Rectangle<'a, T> {
    /// Construct a new rectangle with given $x_{\min}$ and $x_{\max}$
    ///
    /// # Arguments
    ///
    /// - `xmin`: minimum value of `x`
    /// - `xmax`: maximum value of `x`
    ///
    /// # Note
    ///
    /// Rectangle does not copy `xmin` and `xmax` internally; it only keeps
    /// a reference. You may set one of `xmin` and `xmax` to `None` (but not
    /// both).
    ///
    /// # Panics
    ///
    /// The method panics if:
    ///
    /// - Both `xmin` and `xmax` are `None` (use `NoConstraints` instead)
    /// - Both `xmin` and `xmax` have been provided, but they have incompatible
    ///   dimensions
    ///
    /// # Example
    ///
    /// ```
    /// use optimization_engine::constraints::{Constraint, Rectangle};
    ///
    /// let xmin = [-1.0, 0.0];
    /// let xmax = [1.0, 2.0];
    /// let rectangle = Rectangle::new(Some(&xmin), Some(&xmax));
    /// let mut x = [3.0, -4.0];
    /// rectangle.project(&mut x);
    /// ```
    ///
    pub fn new(xmin: Option<&'a [T]>, xmax: Option<&'a [T]>) -> Self {
        assert!(xmin.is_some() || xmax.is_some()); // xmin or xmax must be Some
        if let (Some(xmin), Some(xmax)) = (xmin, xmax) {
            assert_eq!(
                xmin.len(),
                xmax.len(),
                "incompatible dimensions of xmin and xmax"
            );
        }
        if let (Some(xmin), Some(xmax)) = (xmin, xmax) {
            assert!(
                xmin.iter()
                    .zip(xmax.iter())
                    .all(|(xmin_i, xmax_i)| xmin_i <= xmax_i),
                "xmin must be less than or equal to xmax"
            );
        }

        Rectangle { xmin, xmax }
    }
}

impl<'a, T: Float> Constraint<T> for Rectangle<'a, T> {
    fn project(&self, x: &mut [T]) -> FunctionCallResult {
        if let Some(xmin) = &self.xmin {
            assert_eq!(
                x.len(),
                xmin.len(),
                "x and xmin have incompatible dimensions"
            );
            x.iter_mut().zip(xmin.iter()).for_each(|(x_, xmin_)| {
                if *x_ < *xmin_ {
                    *x_ = *xmin_
                };
            });
        }

        if let Some(xmax) = &self.xmax {
            assert_eq!(
                x.len(),
                xmax.len(),
                "x and xmax have incompatible dimensions"
            );
            x.iter_mut().zip(xmax.iter()).for_each(|(x_, xmax_)| {
                if *x_ > *xmax_ {
                    *x_ = *xmax_
                };
            });
        }
        Ok(())
    }

    fn is_convex(&self) -> bool {
        true
    }
}
