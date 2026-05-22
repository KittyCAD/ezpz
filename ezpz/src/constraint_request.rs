use crate::Constraint;
use serde::{Deserialize, Serialize};

/// A constraint that EZPZ should solve for.
/// ```
/// use ezpz::{Constraint, ConstraintRequest};
/// let var = 2;
/// let constraint = Constraint::Fixed(var, 14.2);
/// let priority = 3;
/// let constraint_req = ConstraintRequest::new(constraint, priority);
/// ```
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
pub struct ConstraintRequest {
    /// The constraint itself.
    constraint: Constraint,

    /// The constraint's priority.
    /// 0 is highest priority.
    /// Larger numbers are lower priority.
    priority: u32,

    /// Multiplicative weight applied to this constraint's residual and Jacobian
    /// rows when assembled into the global system. Higher weights make the
    /// solver pull harder on this constraint relative to others within the same
    /// priority tier. Defaults to 1.0.
    weight: f64,
}

impl ConstraintRequest {
    /// Create a new constraint request.
    /// ```
    /// use ezpz::{Constraint, ConstraintRequest};
    /// let var = 2;
    /// let constraint = Constraint::Fixed(var, 14.2);
    /// let priority = 3;
    /// let constraint_req = ConstraintRequest::new(constraint, priority);
    /// ```
    pub fn new(constraint: Constraint, priority: u32) -> Self {
        Self {
            constraint,
            priority,
            weight: 1.0,
        }
    }

    /// Create a new constraint request with the highest priority.
    /// ```
    /// use ezpz::{Constraint, ConstraintRequest};
    /// let var = 2;
    /// let constraint = Constraint::Fixed(var, 14.2);
    /// let constraint_req = ConstraintRequest::highest_priority(constraint);
    /// ```
    pub fn highest_priority(constraint: Constraint) -> Self {
        Self::new(constraint, 0)
    }

    /// Override the default unit weight.
    /// ```
    /// use ezpz::{Constraint, ConstraintRequest};
    /// let req = ConstraintRequest::highest_priority(Constraint::Fixed(0, 1.0))
    ///     .with_weight(100.0);
    /// ```
    pub fn with_weight(mut self, weight: f64) -> Self {
        self.weight = weight;
        self
    }

    /// Get the underlying constraint.
    pub fn constraint(&self) -> &Constraint {
        &self.constraint
    }

    /// Get the underlying priority.
    pub fn priority(&self) -> u32 {
        self.priority
    }

    /// Get the assigned weight.
    pub fn weight(&self) -> f64 {
        self.weight
    }

    pub(crate) fn set_from_initial_values(&mut self, initial_values: &[f64]) {
        self.constraint.set_from_initial_values(initial_values);
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
