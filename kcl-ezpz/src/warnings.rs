use crate::{
    Constraint,
    constraints::ConstraintEntry,
    datatypes::{Angle, AngleKind},
};

/// Something bad that users should know about.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(test, derive(PartialEq))]
pub struct Warning {
    /// If this warning is about a particular constraint, which constraint?
    /// Refers to each constraint by ID.
    pub about_constraint: Option<usize>,
    /// What went wrong, or should be done differently.
    pub content: WarningContent,
}

/// What went wrong, or should be done differently.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(test, derive(PartialEq))]
#[cfg_attr(not(feature = "unstable-exhaustive"), non_exhaustive)]
pub enum WarningContent {
    /// The constraint was satisfied, but only by a degenerate solution,
    /// e.g. making a line where both points are the same.
    Degenerate,
    /// This constraint used a specific angle measurement, but
    /// it would be more accurate to use the Parallel constraint.
    ShouldBeParallel(Angle),
    /// This constraint used a specific angle measurement, but
    /// it would be more accurate to use the Perpendicular constraint.
    ShouldBePerpendicular(Angle),
}

pub(crate) fn lint(constraints: &[ConstraintEntry<'_>]) -> Vec<Warning> {
    let mut warnings = Vec::default();
    for constraint in constraints {
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
            Self::Degenerate => write!(
                f,
                "This geometry is degenerate, meaning two points are so close together that they practically overlap. This is probably unintentional, you probably should place your initial guesses further apart or choose different constraints."
            ),
            Self::ShouldBeParallel(angle) => {
                write!(
                    f,
                    "Instead of constraining to {angle}, constrain to Parallel"
                )
            }
            Self::ShouldBePerpendicular(angle) => {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        Constraint,
        constraints::ConstraintEntry,
        datatypes::{
            Angle, AngleKind,
            inputs::{DatumLineSegment, DatumPoint},
        },
    };

    fn make_lines(angle: Angle) -> Constraint {
        let line = DatumLineSegment {
            p0: DatumPoint::new_xy(0, 1),
            p1: DatumPoint::new_xy(2, 3),
        };
        Constraint::LinesAtAngle(line, line, AngleKind::Other(angle))
    }

    #[test]
    fn suggests_parallel_and_perpendicular() {
        let parallel = make_lines(Angle::from_degrees(360.00005));
        let perpendicular = make_lines(Angle::from_degrees(-90.0));
        let constraints = [
            ConstraintEntry {
                constraint: &parallel,
                id: 7,
                priority: 0,
            },
            ConstraintEntry {
                constraint: &perpendicular,
                id: 9,
                priority: 0,
            },
        ];

        let warnings = lint(&constraints);

        assert_eq!(
            warnings,
            vec![
                Warning {
                    about_constraint: Some(7),
                    content: WarningContent::ShouldBeParallel(Angle::from_degrees(360.00005))
                },
                Warning {
                    about_constraint: Some(9),
                    content: WarningContent::ShouldBePerpendicular(Angle::from_degrees(-90.0))
                }
            ]
        );
    }

    #[test]
    fn display_formats_are_human_friendly() {
        let degenerate = WarningContent::Degenerate.to_string();
        assert!(degenerate.contains("degenerate"));
        let parallel = WarningContent::ShouldBeParallel(Angle::from_degrees(0.0)).to_string();
        assert!(parallel.contains("Parallel"));
        let perpendicular =
            WarningContent::ShouldBePerpendicular(Angle::from_degrees(90.0)).to_string();
        assert!(perpendicular.contains("Perpendicular"));
    }
}
