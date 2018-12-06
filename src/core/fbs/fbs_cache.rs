//! FBS Cache
//!
use super::FBSCache;

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
    pub fn new(n: usize, gamma: f64, tolerance: f64) -> FBSCache {
        FBSCache {
            work_gradient_u: vec![0.0; n],
            work_u_previous: vec![0.0; n],
            gamma: gamma,
            tolerance: tolerance,
            norm_fpr: std::f64::INFINITY,
        }
    }
}
