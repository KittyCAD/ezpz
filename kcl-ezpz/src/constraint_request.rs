use crate::Constraint;

/// A constraint that EZPZ should solve for.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
pub struct ConstraintRequest {
    /// The constraint itself.
    constraint: Constraint,

    /// The constraint's priority.
    /// 0 is highest priority.
    /// Larger numbers are lower priority.
    priority: u32,
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

    /// Get the underlying constraint.
    pub fn constraint(&self) -> &Constraint {
        &self.constraint
    }

    /// Get the underlying priority.
    pub fn priority(&self) -> u32 {
        self.priority
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

#[cfg(test)]
mod tests {
    use crate::tests::assert_nearly_eq;

    use super::*;

    fn demo_constraint() -> Constraint {
        Constraint::Fixed(42, 3.1)
    }

    #[test]
    fn builds_with_expected_priorities() {
        let constraint = demo_constraint();
        let custom = ConstraintRequest::new(constraint, 5);
        assert_eq!(custom.priority, 5);

        let highest = ConstraintRequest::highest_priority(custom.constraint);
        let lower = ConstraintRequest::new(custom.constraint, 40);
        assert!(highest.priority < lower.priority);
    }

    #[test]
    fn converts_back_to_constraint() {
        let constraint = demo_constraint();
        let req = ConstraintRequest::new(constraint, 1);

        let Constraint::Fixed(id, value) = Constraint::from(req) else {
            panic!();
        };
        assert_eq!(id, 42);
        assert_nearly_eq(value, 3.1);

        let req = ConstraintRequest::new(constraint, 1);
        assert!(matches!(req.as_ref(), Constraint::Fixed(_, _)));
    }
}
