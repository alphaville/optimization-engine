//! FBS Cache
//!
use std::num::NonZeroUsize;

/// Cache for the forward-backward splitting (FBS), or projected gradient, algorithm
///
/// This struct allocates memory needed for the FBS algorithm
pub struct FBSCache {
    pub(crate) work_gradient_u: Vec<f64>,
    pub(crate) work_u_previous: Vec<f64>,
    pub(crate) gamma: f64,
    pub(crate) tolerance: f64,
    pub(crate) norm_fpr: f64,
}

impl FBSCache {
    /// Construct a new instance of `FBSCache`
    ///
    /// ## Arguments
    ///
    /// - `gamma` parameter gamma of the algorithm
    /// - `tolerance` tolerance used for termination
    ///
    /// ## Memory allocation
    ///
    /// This method allocates new memory (which it owns, of course). You should avoid
    /// constructing instances of `FBSCache`  in a loop or in any way more than
    /// absolutely necessary
    ///
    /// If you need to call an optimizer more than once, perhaps with different
    /// parameters, then construct an `FBSCache` only once
    ///
    /// This method will allocate memory for `2*n + 3` floats
    ///
    /// ## Panics
    ///
    /// This method will panic if there is no available memory for the required allocation
    /// (capacity overflow)
    ///
    pub fn new(n: NonZeroUsize, gamma: f64, tolerance: f64) -> FBSCache {
        FBSCache {
            work_gradient_u: vec![0.0; n.get()],
            work_u_previous: vec![0.0; n.get()],
            gamma,
            tolerance,
            norm_fpr: std::f64::INFINITY,
        }
    }
}
