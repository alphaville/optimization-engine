use crate::{continuation::ContinuationMode, panoc::*};

#[derive(Debug)]
pub struct HomotopyCache {
    pub(crate) panoc_cache: PANOCCache,
    /// indices of elements of c(u; p) on which to apply continuation
    pub(crate) idx: Vec<usize>,
    /// initial value of continuation
    pub(crate) from: Vec<f64>,
    /// fianl value of continuation
    pub(crate) to: Vec<f64>,
    /// transition mode of continuation
    pub(crate) transition_mode: Vec<ContinuationMode>,
}

impl HomotopyCache {
    pub fn new(panoc_cache: PANOCCache) -> Self {
        HomotopyCache {
            panoc_cache,
            idx: Vec::new(),
            from: Vec::new(),
            to: Vec::new(),
            transition_mode: Vec::new(),
        }
    }

    /// Adds a continuation directive
    ///
    /// ## Arguments
    ///
    /// - `idx`: index of the vector c(u; p) on which continuation should be
    ///   applied
    /// - `from`: starting value
    /// - `to`: target value
    /// - `transition`: transition type (see ContinuationMode)
    pub fn add_continuation(
        &mut self,
        idx: usize,
        from: f64,
        to: f64,
        transition: ContinuationMode,
    ) {
        self.idx.push(idx);
        self.from.push(from);
        self.to.push(to);
        self.transition_mode.push(transition);
    }

    /// Adds multiple continuation directives
    ///
    /// ## Arguments
    ///
    /// - `idx`: indices of the vector c(u; p) on which continuation should be
    ///   applied
    /// - `from`: starting values
    /// - `to`: target values
    /// - `transition`: transition types (see ContinuationMode)
    ///
    pub fn add_continuations(
        &mut self,
        idx: &[usize],
        from: &[f64],
        to: &[f64],
        transition: &[ContinuationMode],
    ) {
        self.idx.extend(idx);
        self.from.extend(from);
        self.to.extend(to);
        self.transition_mode.extend(transition);
    }
}
