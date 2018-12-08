//! Constraints and projections

mod ball2;
mod no_constraints;
mod rectangle;

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

/// The whole space, no constraints
pub struct NoConstraints {}

/// A Eucledian ball
pub struct Ball2 {
    centre: Option<Vec<f64>>,
    radius: f64,
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

/* ---------------------------------------------------------------------------- */
/*          TESTS                                                               */
/* ---------------------------------------------------------------------------- */
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

    #[test]
    fn no_constraints() {
        let mut x = [1.0, 2.0, 3.0];
        let whole_space = NoConstraints::new();
        whole_space.project(&mut x);
        assert_eq!([1., 2., 3.], x);
    }
}
