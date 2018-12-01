pub trait Constraint {
    fn project(&self, x: &mut [f64]);
}

pub struct Rectangle {
    xmin: Option<Vec<f64>>,
    xmax: Option<Vec<f64>>,
}

impl Rectangle {
    pub fn new(xmin_: Vec<f64>, xmax_: Vec<f64>) -> Rectangle {
        Rectangle {
            xmin: Some(xmin_),
            xmax: Some(xmax_),
        }
    }

    pub fn new_only_xmin(xmin_: Vec<f64>) -> Rectangle {
        Rectangle {
            xmin: Some(xmin_),
            xmax: None,
        }
    }

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

pub struct Ball2 {
    centre: Option<Vec<f64>>,
    radius: f64,
}

impl Ball2 {
    pub fn new_at_origin_with_radius(radius_: f64) -> Ball2 {
        assert!(radius_ > 0.0);
        Ball2 {
            centre: None,
            radius: radius_,
        }
    }

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
                x.iter_mut().for_each(|x_| *x_ /= norm_x);
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
