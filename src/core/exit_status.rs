/// Exit status of an algorithm (not algorithm specific)
///
///
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ExitStatus {
    /// The algorithm has converged
    ///
    /// All termination criteria are satisfied and the algorithm
    /// converged within the available time and number of iterations
    Converged,
    /// Failed to converge because the maximum number of iterations was reached
    NotConvergedIterations,
    /// Failed to converge because the maximum execution time was reached
    NotConvergedOutOfTime,
    /// The algorithm failed to converge for other reasons
    ///
    /// This exit code corresponds to highly exceptional circumstances; it may
    /// occur if a function computation fails
    Failure,
}
