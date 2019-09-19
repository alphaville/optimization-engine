//! PANOC super-fast algorithm

#![deny(missing_docs)]

mod panoc_cache;
mod panoc_engine;
mod panoc_optimizer;

pub use panoc_cache::PANOCCache;
pub use panoc_optimizer::PANOCOptimizer;

#[cfg(test)]
mod tests;
