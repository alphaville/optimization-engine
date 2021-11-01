use super::Constraint;
use super::Simplex;

#[derive(Copy, Clone)]
/// Docs
pub struct Ball1<'a> {
    center: Option<&'a [f64]>,
    radius: f64,
    simplex: Simplex,
}

impl<'a> Ball1<'a> {
    /// Docs
    pub fn new(center: Option<&'a [f64]>, radius: f64) -> Self {
        assert!(radius > 0.0);
        let simplex = Simplex::new(radius);
        Ball1 {
            center,
            radius,
            simplex,
        }
    }

    fn project_on_ball1_centered_at_origin(&self, x: &mut [f64]) {
        if crate::matrix_operations::norm1(x) > self.radius {
            // u = |x| (copied)
            let mut u = vec![0.0; x.len()];
            u.iter_mut()
                .zip(x.iter())
                .for_each(|(ui, &xi)| *ui = f64::abs(xi));
            // u = P_simplex(u)
            self.simplex.project(&mut u);
            x.iter_mut()
                .zip(u.iter())
                .for_each(|(xi, &ui)| *xi = f64::signum(*xi) * ui);
        }
    }
}

impl<'a> Constraint for Ball1<'a> {
    fn project(&self, x: &mut [f64]) {
        if let Some(center) = &self.center {
            x.iter_mut()
                .zip(center.iter())
                .for_each(|(xi, &ci)| *xi -= ci);
            self.project_on_ball1_centered_at_origin(x);
            x.iter_mut()
                .zip(center.iter())
                .for_each(|(xi, &ci)| *xi += ci);
        } else {
            self.project_on_ball1_centered_at_origin(x);
        }
    }

    fn is_convex(&self) -> bool {
        true
    }
}
