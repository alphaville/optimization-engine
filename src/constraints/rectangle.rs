use super::Constraint;

///
/// A rectangle
///
/// A set of the form `{x : xmin <= x <= xmax}`, where `<=` is meant in the
/// element-wise sense and either of `xmin` and `xmax` can be equal to infinity.
pub struct Rectangle {
    xmin: Option<Vec<f64>>,
    xmax: Option<Vec<f64>>,
}

impl Rectangle {
    /// Construct a new rectangle with given `xmin` and `xmax`
    pub fn new(xmin_: Vec<f64>, xmax_: Vec<f64>) -> Rectangle {
        Rectangle {
            xmin: Some(xmin_),
            xmax: Some(xmax_),
        }
    }

    /// Construct a new rectangle with given `xmin` and no `xmax`
    ///
    /// Essentially, this is a halfspace
    pub fn new_only_xmin(xmin_: Vec<f64>) -> Rectangle {
        Rectangle {
            xmin: Some(xmin_),
            xmax: None,
        }
    }

    /// Construct a new rectangle with given `xmax` and no `xmin`
    ///
    /// Essentially, this is a halfspace
    pub fn new_only_xmax(xmax_: Vec<f64>) -> Rectangle {
        Rectangle {
            xmin: None,
            xmax: Some(xmax_),
        }
    }
}

impl Constraint for Rectangle {
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
