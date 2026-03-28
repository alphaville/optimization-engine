//! FBS Cache
//!
use num::Float;
use std::num::NonZeroUsize;

/// Cache for the forward-backward splitting (FBS), or projected gradient, algorithm
///
/// This struct allocates memory needed for the FBS algorithm
pub struct FBSCache<T = f64>
where
    T: Float,
{
    pub(crate) work_gradient_u: Vec<T>,
    pub(crate) work_u_previous: Vec<T>,
    pub(crate) gamma: T,
    pub(crate) tolerance: T,
    pub(crate) norm_fpr: T,
}

impl<T: Float> FBSCache<T> {
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
    pub fn new(n: NonZeroUsize, gamma: T, tolerance: T) -> FBSCache<T> {
        FBSCache {
            work_gradient_u: vec![T::zero(); n.get()],
            work_u_previous: vec![T::zero(); n.get()],
            gamma,
            tolerance,
            norm_fpr: T::infinity(),
        }
    }
}
