//! Efficient Zoo Problem Zolver.
//! Solves 2D constraint systems.

pub use crate::constraints::Constraint;
use crate::datatypes::Angle;
pub use crate::solver::Config;
// Only public for now so that I can benchmark it.
// TODO: Replace this with an end-to-end benchmark,
// or find a different way to structure modules.
pub use crate::id::{Id, IdGenerator};
use crate::solver::Model;
use faer::sparse::CreationError;

/// Each kind of constraint we support.
mod constraints;
/// Geometric data (lines, points, etc).
pub mod datatypes;
/// IDs of various entities, points, scalars etc.
mod id;
/// Numeric solver using sparse matrices.
mod solver;
/// Unit tests
#[cfg(test)]
mod tests;
/// Parser for textual representation of these problems.
pub mod textual;
mod vector;

const EPSILON: f64 = 1e-4;

#[derive(thiserror::Error, Debug)]
#[non_exhaustive]
pub enum Error {
    #[error("{0}")]
    NonLinearSystemError(#[from] NonLinearSystemError),
    #[error("Solver error {0}")]
    Solver(Box<dyn std::error::Error>),
    #[error("No guess was given for point {label}")]
    MissingGuess { label: String },
    #[error("You gave a guess for points which weren't defined: {labels:?}")]
    UnusedGuesses { labels: Vec<String> },
    #[error("You referred to the point {label} but it was never defined")]
    UndefinedPoint { label: String },
}

#[derive(thiserror::Error, Debug)]
#[non_exhaustive]
pub enum NonLinearSystemError {
    #[error("ID {0} not found")]
    NotFound(Id),
    #[error(
        "There should be exactly 1 guess per variable, but you supplied {labels} variables and must {guesses} guesses"
    )]
    WrongNumberGuesses { labels: usize, guesses: usize },
    #[error(
        "Constraint {c} references variable {v} but no such variable appears in your initial guesses."
    )]
    MissingGuess { c: usize, v: Id },
    #[error("Could not create matrix: {error}")]
    FaerMatrix {
        #[from]
        error: CreationError,
    },
}

#[derive(Debug)]
pub struct SolveOutcome {
    pub final_values: Vec<f64>,
    pub iterations: usize,
    pub warnings: Vec<Warning>,
}

#[derive(Debug)]
#[cfg_attr(test, derive(PartialEq))]
pub struct Warning {
    pub about_constraint: Option<usize>,
    pub content: WarningContent,
}

#[derive(Debug)]
#[cfg_attr(test, derive(PartialEq))]
#[non_exhaustive]
pub enum WarningContent {
    Degenerate,
    ShouldBeParallel(Angle),
    ShouldBePerpendicular(Angle),
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

#[derive(Debug)]
pub struct FailureOutcome {
    pub error: Error,
    pub warnings: Vec<Warning>,
    pub num_vars: usize,
    pub num_eqs: usize,
}

/// Given some initial guesses, constrain them.
/// Returns the same variables in the same order, but constrained.
pub fn solve(
    constraints: &[Constraint],
    initial_guesses: Vec<(Id, f64)>,
    config: Config,
) -> Result<SolveOutcome, FailureOutcome> {
    let num_vars = initial_guesses.len();
    let num_eqs = constraints.iter().map(|c| c.residual_dim()).sum();
    let (all_variables, mut values): (Vec<Id>, Vec<f64>) = initial_guesses.into_iter().unzip();
    let mut warnings = lint(constraints);
    let initial_values = values.clone();

    let mut model = match Model::new(constraints, all_variables, initial_values, config) {
        Ok(o) => o,
        Err(e) => {
            return Err(FailureOutcome {
                error: e.into(),
                warnings,
                num_vars,
                num_eqs,
            });
        }
    };
    let outcome = newton_faer::solve(
        &mut model,
        &mut values,
        newton_faer::NewtonCfg::sparse().with_adaptive(true),
    )
    .map_err(|errs| Error::Solver(Box::new(errs.into_error())));
    warnings.extend(model.warnings.lock().unwrap().drain(..));
    let iterations = match outcome {
        Ok(o) => o,
        Err(e) => {
            return Err(FailureOutcome {
                error: e,
                warnings,
                num_vars,
                num_eqs,
            });
        }
    };

    Ok(SolveOutcome {
        final_values: values,
        iterations,
        warnings,
    })
}

fn nearly_eq(a: f64, b: f64) -> bool {
    (a - b).abs() < EPSILON
}

fn lint(constraints: &[Constraint]) -> Vec<Warning> {
    let mut warnings = Vec::default();
    for (i, constraint) in constraints.iter().enumerate() {
        match constraint {
            Constraint::LinesAtAngle(_, _, constraints::AngleKind::Other(theta))
                if nearly_eq(theta.to_degrees(), 0.0)
                    || nearly_eq(theta.to_degrees(), 360.0)
                    || nearly_eq(theta.to_degrees(), 180.0) =>
            {
                warnings.push(Warning {
                    about_constraint: Some(i),
                    content: WarningContent::ShouldBeParallel(*theta),
                });
            }
            Constraint::LinesAtAngle(_, _, constraints::AngleKind::Other(theta))
                if nearly_eq(theta.to_degrees(), 90.0) || nearly_eq(theta.to_degrees(), -90.0) =>
            {
                warnings.push(Warning {
                    about_constraint: Some(i),
                    content: WarningContent::ShouldBePerpendicular(*theta),
                });
            }
            _ => {}
        }
    }
    warnings
}
