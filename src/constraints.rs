/// A set which can be used as a constraint
///
/// This trait defines an abstract function that allows to compute projections
/// on the set
pub trait Constraint {
    /// Projection onto the set
    ///
    /// The given vector `x` is updated with the projection on the set
    fn project(&self, x: &mut [f64]);
}

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

/// A Eucledian ball
pub struct Ball2 {
    centre: Option<Vec<f64>>,
    radius: f64,
}

impl Ball2 {
    ///
    /// Construct a new ball centered at the origin with given radius
    pub fn new_at_origin_with_radius(radius_: f64) -> Ball2 {
        assert!(radius_ > 0.0);
        Ball2 {
            centre: None,
            radius: radius_,
        }
    }

    ///
    /// Construct a new Eucledian ball with given centre and radius
    pub fn new(centre_: Vec<f64>, radius_: f64) -> Ball2 {
        assert!(radius_ > 0.0);
        Ball2 {
            centre: Some(centre_),
            radius: radius_,
        }
    }
}

impl Constraint for Ball2 {
    fn project(&self, x: &mut [f64]) {
        if self.centre.is_none() {
            let norm_x = crate::matrix_operations::norm2(x);
            if norm_x > self.radius {
                let norm_over_radius = norm_x / self.radius;
                x.iter_mut().for_each(|x_| *x_ /= norm_over_radius);
            }
        } else {
            let mut norm_difference = 0.0;
            x.iter()
                .zip(self.centre.as_ref().unwrap().iter())
                .for_each(|(a, b)| {
                    let diff_ = *a - *b;
                    norm_difference += diff_ * diff_
                });
            norm_difference = norm_difference.sqrt();

            if norm_difference > self.radius {
                x.iter_mut()
                    .zip(self.centre.as_ref().unwrap().iter())
                    .for_each(|(x, c)| {
                        *x = *c + (*x - *c) / norm_difference;
                    });
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rectangle_closed() {
        let xmin = vec![2.0; 5];
        let xmax = vec![4.5; 5];
        let rectangle = Rectangle::new(xmin, xmax);
        let mut x = [1.0, 2.0, 3.0, 4.0, 5.0];
        rectangle.project(&mut x);
        println!("x = {:?}", x);
    }

    #[test]
    fn rectangle_only_xmin() {
        let xmin = vec![2.0; 5];
        let rectangle = Rectangle::new_only_xmin(xmin);
        let mut x = [1.0, 2.0, 3.0, 4.0, 5.0];
        rectangle.project(&mut x);
        println!("x = {:?}", x);
    }

    #[test]
    fn ball_at_origin() {
        let radius = 1.0;
        let mut x = [1.0, 1.0];
        let ball = Ball2::new_at_origin_with_radius(radius);
        ball.project(&mut x);
        println!("x = {:?}", x);
    }

    #[test]
    fn ball_elsewhere() {
        let radius = 1.0;
        let centre = [1.0, 1.0];
        let mut x = [2.0, 2.0];
        let ball = Ball2::new(centre.to_vec(), radius);
        ball.project(&mut x);
        println!("x = {:?}", x);
    }
}
