use crate::{
    FreedomAnalysis, Id, NonLinearSystemError, Warning,
    datatypes::{
        inputs::{DatumCircle, DatumCircularArc, DatumDistance, DatumPoint},
        outputs::{Arc, Circle, Point},
    },
};

/// Data from a successful solved system.
#[derive(Debug)]
#[cfg_attr(not(feature = "unstable-exhaustive"), non_exhaustive)]
pub struct SolveOutcome {
    /// Which constraints couldn't be satisfied
    pub(crate) unsatisfied: Vec<usize>,
    /// Each variable's final value.
    pub(crate) final_values: Vec<f64>,
    /// How many iterations of Newton's method were required?
    pub(crate) iterations: usize,
    /// Anything that went wrong either in problem definition or during solving it.
    pub(crate) warnings: Vec<Warning>,
    /// What is the lowest priority that got solved?
    /// 0 is the highest priority. Larger numbers are lower priority.
    pub(crate) priority_solved: u32,
}

impl SolveOutcome {
    /// Which constraints couldn't be satisfied
    pub fn unsatisfied(&self) -> &[usize] {
        &self.unsatisfied
    }

    /// Each variable's final value.
    pub fn final_values(&self) -> &[f64] {
        &self.final_values
    }

    /// How many iterations of Newton's method were required?
    pub fn iterations(&self) -> usize {
        self.iterations
    }

    /// Anything that went wrong either in problem definition or during solving it.
    pub fn warnings(&self) -> &[Warning] {
        &self.warnings
    }

    /// What is the lowest priority that got solved?
    /// 0 is the highest priority. Larger numbers are lower priority.
    pub fn priority_solved(&self) -> u32 {
        self.priority_solved
    }

    /// Look up the solved value for this distance.
    fn final_value_scalar(&self, id: Id) -> f64 {
        self.final_values[id as usize]
    }

    /// Look up the solved value for this distance.
    pub fn final_value_distance(&self, distance: &DatumDistance) -> f64 {
        self.final_values[distance.id as usize]
    }

    /// Look up the solved values for this point.
    pub fn final_value_point(&self, point: &DatumPoint) -> Point {
        let x = self.final_value_scalar(point.id_x());
        let y = self.final_value_scalar(point.id_y());
        Point { x, y }
    }

    /// Look up the solved values for this arc.
    pub fn final_value_arc(&self, arc: &DatumCircularArc) -> Arc {
        let a = self.final_value_point(&arc.start);
        let b = self.final_value_point(&arc.end);
        let c = self.final_value_point(&arc.center);
        Arc { a, b, center: c }
    }

    /// Look up the solved values for this circle.
    pub fn final_value_circle(&self, circle: &DatumCircle) -> Circle {
        let center = self.final_value_point(&circle.center);
        let radius = self.final_value_distance(&circle.radius);
        Circle { center, radius }
    }

    /// Were all constraints satisfied?
    pub fn is_satisfied(&self) -> bool {
        self.unsatisfied.is_empty()
    }

    /// Were any constraints unsatisfied?
    pub fn is_unsatisfied(&self) -> bool {
        !self.is_satisfied()
    }
}

/// Just like [`SolveOutcome`] except it also contains the result of
/// expensive numeric analysis on the final solved system.
// This is just like `SolveOutcomeAnalysis<FreedomAnalysis>`,
// except it doesn't leak the private trait `Analysis`.
#[derive(Debug)]
pub struct SolveOutcomeFreedomAnalysis {
    /// Extra analysis for the system,
    /// which is probably expensive to compute.
    pub analysis: FreedomAnalysis,
    /// Other data.
    pub outcome: SolveOutcome,
}

impl AsRef<SolveOutcome> for SolveOutcomeFreedomAnalysis {
    fn as_ref(&self) -> &SolveOutcome {
        &self.outcome
    }
}

/// Returned when ezpz could not solve a system.
#[derive(Debug)]
#[cfg_attr(not(feature = "unstable-exhaustive"), non_exhaustive)]
pub struct FailureOutcome {
    /// The error that stopped the system from being solved.
    pub error: NonLinearSystemError,
    /// Other warnings which might have contributed,
    /// or might be suboptimal for other reasons.
    pub warnings: Vec<Warning>,
    /// Size of the system.
    pub num_vars: usize,
    /// Size of the system.
    pub num_eqs: usize,
}

impl FailureOutcome {
    /// The error that stopped the system from being solved.
    pub fn error(&self) -> &NonLinearSystemError {
        &self.error
    }

    /// Other warnings which might have contributed,
    /// or might be suboptimal for other reasons.
    pub fn warnings(&self) -> &[Warning] {
        &self.warnings
    }

    /// Size of the system.
    pub fn num_vars(&self) -> usize {
        self.num_vars
    }

    /// Size of the system.
    pub fn num_eqs(&self) -> usize {
        self.num_eqs
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_satisfied() {
        // Test the is_unsatisfied and is_satisfied getters
        // do what we expect.
        let so = SolveOutcome {
            unsatisfied: vec![0],
            final_values: vec![0.3],
            iterations: 1,
            warnings: Vec::new(),
            priority_solved: 0,
        };

        assert!(so.is_unsatisfied());
        assert!(!so.is_satisfied());
    }
}
