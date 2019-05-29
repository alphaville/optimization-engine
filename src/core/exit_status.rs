#[derive(Debug, Clone, Copy)]
pub enum ExitStatus {
    Converged,
    NotConvergedIterations,
    NotConvergedTime,
}