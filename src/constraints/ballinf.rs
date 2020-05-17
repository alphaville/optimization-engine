use super::Constraint;

#[derive(Copy, Clone)]
/// An infinity ball defined as $B_\infty^r = \\{x\in\mathbb{R}^n {}:{} \Vert{}x{}\Vert_{\infty} \leq r\\}$,
/// where $\Vert{}\cdot{}\Vert_{\infty}$ is the infinity norm. The infinity ball centered at a point
/// $x_c$ is defined as $B_\infty^{x_c,r} = \\{x\in\mathbb{R}^n {}:{} \Vert{}x-x_c{}\Vert_{\infty} \leq r\\}$.
///
pub struct BallInf<'a> {
    center: Option<&'a [f64]>,
    radius: f64,
}

impl<'a> BallInf<'a> {
    /// Construct a new infinity-norm ball with given center and radius
    /// If no `center` is given, then it is assumed to be in the origin
    ///   
    pub fn new(center: Option<&'a [f64]>, radius: f64) -> Self {
        assert!(radius > 0.0);
        BallInf { center, radius }
    }
}

impl<'a> Constraint for BallInf<'a> {
    /// Computes the projection of a given vector `x` on the current infinity ball.
    ///
    ///
    /// The projection of a $v\in\mathbb{R}^{n}$ on $B_\infty^r$ is given by
    /// $\Pi_{B_\infty^r}(v) = z$ with
    ///
    /// $$
    /// z_i = \begin{cases}v_i,&\text{ if } |z_i| \leq r\\\\\mathrm{sng}(v_i)r,&\text{ otherwise}\end{cases}
    /// $$
    ///
    /// for all $i=1,\ldots, n$, where sgn is the sign function.
    ///
    /// The projection of $v\in\mathbb{R}^{n}$ on $B_\infty^{x_c,r}$ is given by
    /// $\Pi_{B_\infty^r}(v) = z$ with
    ///
    /// $$
    /// z_i = \begin{cases}v_i,&\text{ if } |z_i-x_{c, i}| \leq r\\\\x_{c,i} + \mathrm{sng}(v_i)r,&\text{ otherwise}\end{cases}
    /// $$
    ///
    /// for all $i=1,\ldots, n$.
    ///
    fn project(&self, x: &mut [f64]) {
        if let Some(center) = &self.center {
            x.iter_mut()
                .zip(center.iter())
                .filter(|(&mut xi, &ci)| (xi - ci).abs() > self.radius)
                .for_each(|(xi, ci)| *xi = ci + (*xi - ci).signum() * self.radius);
        } else {
            x.iter_mut()
                .filter(|xi| xi.abs() > self.radius)
                .for_each(|xi| *xi = xi.signum() * self.radius);
        }
    }

    fn is_convex(&self) -> bool {
        true
    }
}
