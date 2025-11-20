use crate::{
    Constraint,
    constraints::ConstraintEntry,
    datatypes::{Angle, AngleKind},
};

#[derive(Debug, Clone)]
#[cfg_attr(test, derive(PartialEq))]
pub struct Warning {
    pub about_constraint: Option<usize>,
    pub content: WarningContent,
}

#[derive(Debug, Clone)]
#[cfg_attr(test, derive(PartialEq))]
#[non_exhaustive]
pub enum WarningContent {
    Degenerate,
    ShouldBeParallel(Angle),
    ShouldBePerpendicular(Angle),
}

pub fn lint(constraints: &[ConstraintEntry]) -> Vec<Warning> {
    let mut warnings = Vec::default();
    for constraint in constraints.iter() {
        match constraint.constraint {
            Constraint::LinesAtAngle(_, _, AngleKind::Other(theta))
                if nearly_eq(theta.to_degrees(), 0.0)
                    || nearly_eq(theta.to_degrees(), 360.0)
                    || nearly_eq(theta.to_degrees(), 180.0) =>
            {
                warnings.push(Warning {
                    about_constraint: Some(constraint.id),
                    content: WarningContent::ShouldBeParallel(*theta),
                });
            }
            Constraint::LinesAtAngle(_, _, AngleKind::Other(theta))
                if nearly_eq(theta.to_degrees(), 90.0) || nearly_eq(theta.to_degrees(), -90.0) =>
            {
                warnings.push(Warning {
                    about_constraint: Some(constraint.id),
                    content: WarningContent::ShouldBePerpendicular(*theta),
                });
            }
            _ => {}
        }
    }
    warnings
}

impl std::fmt::Display for WarningContent {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            WarningContent::Degenerate => write!(
                f,
                "This geometry is degenerate, meaning two points are so close together that they practically overlap. This is probably unintentional, you probably should place your initial guesses further apart or choose different constraints."
            ),
            WarningContent::ShouldBeParallel(angle) => {
                write!(
                    f,
                    "Instead of constraining to {angle}, constrain to Parallel"
                )
            }
            WarningContent::ShouldBePerpendicular(angle) => {
                write!(
                    f,
                    "Instead of constraining to {angle}, constraint to Perpendicular"
                )
            }
        }
    }
}

fn nearly_eq(a: f64, b: f64) -> bool {
    (a - b).abs() < crate::EPSILON
}
