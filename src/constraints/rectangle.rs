use super::{Constraint, Rectangle};

impl Rectangle {
    ///
    /// Construct a new rectangle with given `xmin` and `xmax`
    pub fn new(xmin_: Vec<f64>, xmax_: Vec<f64>) -> Rectangle {
        Rectangle {
            xmin: Some(xmin_),
            xmax: Some(xmax_),
        }
    }
    ///
    /// Construct a new rectangle with given `xmin` and no `xmax`
    ///
    /// Essentially, this is a halfspace
    pub fn new_only_xmin(xmin_: Vec<f64>) -> Rectangle {
        Rectangle {
            xmin: Some(xmin_),
            xmax: None,
        }
    }

    ///
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
        if self.xmin.is_some() {
            x.iter_mut()
                .zip(self.xmin.as_ref().unwrap().iter())
                .for_each(|(x_, xmin_)| {
                    if *x_ < *xmin_ {
                        *x_ = *xmin_
                    };
                });
        }

        if self.xmax.is_some() {
            x.iter_mut()
                .zip(self.xmax.as_ref().unwrap().iter())
                .for_each(|(x_, xmax_)| {
                    if *x_ > *xmax_ {
                        *x_ = *xmax_
                    };
                });
        }
    }
}
