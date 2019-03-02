use super::{Ball2, Constraint};

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
