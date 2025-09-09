//! Efficient Zoo Problem Zolver.
//! Solves 2D constraint systems.

pub use crate::constraints::Constraint;
pub use crate::solver::Config;
// Only public for now so that I can benchmark it.
// TODO: Replace this with an end-to-end benchmark,
// or find a different way to structure modules.
pub use crate::id::{Id, IdGenerator};
use crate::solver::Model;

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

const EPSILON: f64 = 1e-5;

#[derive(thiserror::Error, Debug)]
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
pub enum NonLinearSystemError {
    #[error("ID {0} not found")]
    NotFound(Id),
    #[error(
        "There should be exactly 1 guess per variable, but you supplied {labels} variables and must {guesses} guesses"
    )]
    WrongNumberGuesses { labels: usize, guesses: usize },
}

#[derive(Debug)]
pub struct SolveOutcome {
    pub final_values: Vec<f64>,
    pub iterations: usize,
    pub lints: Vec<Lint>,
}

#[derive(Debug)]
#[cfg_attr(test, derive(Eq, PartialEq))]
pub struct Lint {
    pub about_constraint: Option<usize>,
    pub content: String,
}

#[derive(Debug)]
pub struct FailureOutcome {
    pub error: Error,
    pub lints: Vec<Lint>,
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
    let lints = lint(constraints);
    let initial_values = values.clone();

    let mut model = match Model::new(constraints, all_variables, initial_values, config) {
        Ok(o) => o,
        Err(e) => {
            return Err(FailureOutcome {
                error: e.into(),
                lints,
                num_vars,
                num_eqs,
            });
        }
    };
    let iterations = match newton_faer::solve(
        &mut model,
        &mut values,
        newton_faer::NewtonCfg::sparse().with_adaptive(true),
    )
    .map_err(|errs| Error::Solver(Box::new(errs.into_error())))
    {
        Ok(o) => o,
        Err(e) => {
            return Err(FailureOutcome {
                error: e,
                lints,
                num_vars,
                num_eqs,
            });
        }
    };

    Ok(SolveOutcome {
        final_values: values,
        iterations,
        lints,
    })
}

fn nearly_eq(a: f64, b: f64) -> bool {
    (a - b).abs() < EPSILON
}

fn lint(constraints: &[Constraint]) -> Vec<Lint> {
    let mut lints = Vec::default();
    for (i, constraint) in constraints.iter().enumerate() {
        match constraint {
            Constraint::LinesAtAngle(_, _, constraints::AngleKind::Other(theta))
                if nearly_eq(theta.to_degrees(), 0.0)
                    || nearly_eq(theta.to_degrees(), 360.0)
                    || nearly_eq(theta.to_degrees(), 180.0) =>
            {
                lints.push(Lint {
                    about_constraint: Some(i),
                    content: content_for_angle(true, theta.to_degrees()),
                });
            }
            Constraint::LinesAtAngle(_, _, constraints::AngleKind::Other(theta))
                if nearly_eq(theta.to_degrees(), 90.0) || nearly_eq(theta.to_degrees(), -90.0) =>
            {
                lints.push(Lint {
                    about_constraint: Some(i),
                    content: content_for_angle(false, theta.to_degrees()),
                });
            }
            _ => {}
        }
    }
    lints
}

fn content_for_angle(is_parallel: bool, actual_degrees: f64) -> String {
    format!(
        "Suggest using AngleKind::{} instead of AngleKind::Other({}deg)",
        if is_parallel {
            "Parallel"
        } else {
            "Perpendicular"
        },
        actual_degrees
    )
}
