use crate::panoc::PANOCCache;

const DEFAULT_INITIAL_PENALTY: f64 = 10.0;

#[derive(Debug)]
pub struct AlmCache {
    /// PANOC cache for inner problems
    pub(crate) panoc_cache: PANOCCache,
    /// Lagrange multipliers (next)
    pub(crate) y_plus: Option<Vec<f64>>,
    /// Vector xi^nu = (c^nu, y^nu)
    pub(crate) xi: Option<Vec<f64>>,
    /// Infeasibility related to ALM-type constraints
    ///
    pub(crate) delta_y_norm: f64,
    pub(crate) delta_y_norm_plus: f64,
    pub(crate) f2_norm: f64,
    pub(crate) f2_norm_plus: f64,
    pub(crate) w_alm_aux: Option<Vec<f64>>,
    /// Infeasibility related to PM-type constraints
    ///
    /// w_pm = F2(u)
    pub(crate) w_pm: Option<Vec<f64>>,
    /// (Outer) iteration
    pub(crate) iteration: usize,
}

impl AlmCache {
    pub fn new(panoc_cache: PANOCCache, n1: usize, n2: usize) -> Self {
        AlmCache {
            panoc_cache,
            y_plus: if n1 > 0 { Some(vec![0.0; n1]) } else { None },
            // Allocate memory for xi = (c, y) if either n1 or n2 is nonzero
            xi: if n1 + n2 > 0 {
                let mut xi_init = vec![DEFAULT_INITIAL_PENALTY; 1];
                xi_init.append(&mut vec![0.0; n1]);
                Some(xi_init)
            } else {
                None
            },
            w_alm_aux: if n1 > 0 { Some(vec![0.0; n1]) } else { None },
            w_pm: if n2 > 0 { Some(vec![0.0; n2]) } else { None },
            iteration: 0,
            delta_y_norm: 0.0,
            delta_y_norm_plus: 0.0,
            f2_norm: 0.0,
            f2_norm_plus: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.panoc_cache.reset();
        self.iteration = 0;
        self.f2_norm = 0.0;
        self.f2_norm_plus = 0.0;
        self.delta_y_norm = 0.0;
        self.delta_y_norm_plus = 0.0;
    }
}
