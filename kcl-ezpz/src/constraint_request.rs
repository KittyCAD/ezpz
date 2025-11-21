use crate::Constraint;

/// A constraint that EZPZ should solve for.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
pub struct ConstraintRequest {
    /// The constraint itself.
    pub constraint: Constraint,
    /// The constraint's priority.
    /// 0 is highest priority.
    /// Larger numbers are lower priority.
    pub priority: u32,
}

impl ConstraintRequest {
    /// Create a new constraint request.
    pub fn new(constraint: Constraint, priority: u32) -> Self {
        Self {
            constraint,
            priority,
        }
    }

    /// Create a new constraint request with the highest priority.
    pub fn highest_priority(constraint: Constraint) -> Self {
        Self::new(constraint, 0)
    }
}

impl From<ConstraintRequest> for Constraint {
    fn from(value: ConstraintRequest) -> Self {
        value.constraint
    }
}

impl AsRef<Constraint> for ConstraintRequest {
    fn as_ref(&self) -> &Constraint {
        &self.constraint
    }
}
