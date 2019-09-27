use super::Constraint;

///
/// A rectangle, $R = \\{x \in \mathbb{R}^n {}:{} x_{\min} {}\leq{} x {}\leq{} x_{\max}\\}$
///
/// A set of the form $\\{x \in \mathbb{R}^n {}:{} x_{\min} {}\leq{} x {}\leq{} x_{\max}\\}$,
/// where $\leq$ is meant in the element-wise sense and either of $x_{\min}$ and $x_{\max}$ can
/// be equal to infinity.
pub struct Rectangle<'a> {
    xmin: Option<&'a [f64]>,
    xmax: Option<&'a [f64]>,
}

impl<'a> Rectangle<'a> {
    /// Construct a new rectangle with given $x_{\min}$ and $x_{\max}$
    ///
    /// ## Arguments
    /// 
    /// - `xmin`
    /// - `xmin`
    /// 
    /// ## Panics
    /// 
    /// The method panics if:
    /// 
    /// - Both `xmin` and `xmax` are `None` (use `NoConstraints` instead)
    /// - Both `xmin` and `xmax` have been provided, but they have incompatible 
    ///   dimensions
    /// 
    pub fn new(xmin: Option<&'a [f64]>, xmax: Option<&'a [f64]>) -> Rectangle<'a> {
        assert!(xmin != None || xmax != None); // xmin or xmax must be Some
        assert!(
            xmin.is_none() || xmax.is_none() || xmin.unwrap().len() == xmax.unwrap().len(),
            "incompatible dimensions of xmin and xmax"
        );
        Rectangle { xmin, xmax }
    }
}

impl<'a> Constraint for Rectangle<'a> {
    fn project(&self, x: &mut [f64]) {
        if let Some(xmin) = &self.xmin {
            x.iter_mut().zip(xmin.iter()).for_each(|(x_, xmin_)| {
                if *x_ < *xmin_ {
                    *x_ = *xmin_
                };
            });
        }

        if let Some(xmax) = &self.xmax {
            x.iter_mut().zip(xmax.iter()).for_each(|(x_, xmax_)| {
                if *x_ > *xmax_ {
                    *x_ = *xmax_
                };
            });
        }
    }
}
