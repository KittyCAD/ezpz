pub use crate::constraints::Constraint;
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

const EPSILON: f64 = 0.001;

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
}

#[derive(Debug)]
pub struct SolveOutcome {
    pub final_values: Vec<f64>,
    pub iterations: usize,
    pub lints: Vec<Lint>,
}

#[derive(Debug)]
pub struct Lint {
    pub about_constraint: Option<usize>,
    pub content: String,
}

/// Given some initial guesses, constrain them.
/// Returns the same variables in the same order, but constrained.
pub fn solve(
    constraints: &[Constraint],
    initial_guesses: Vec<(Id, f64)>,
) -> Result<SolveOutcome, Error> {
    let (all_variables, mut final_values): (Vec<Id>, Vec<f64>) =
        initial_guesses.into_iter().unzip();
    let lints = lint(constraints);

    let mut model = Model::new(constraints, all_variables)?;
    let iterations = newton_faer::solve(
        &mut model,
        &mut final_values,
        newton_faer::NewtonCfg::sparse().with_adaptive(true),
    )
    .map_err(|errs| Error::Solver(Box::new(errs.into_error())))?;

    Ok(SolveOutcome {
        final_values,
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
                    content: format!(
                        "Suggest using AngleKind::Parallel instead of AngleKind::Other({})",
                        theta.to_degrees()
                    ),
                });
            }
            Constraint::LinesAtAngle(_, _, constraints::AngleKind::Other(theta))
                if nearly_eq(theta.to_degrees(), 90.0) || nearly_eq(theta.to_degrees(), -90.0) =>
            {
                lints.push(Lint {
                    about_constraint: Some(i),
                    content: format!(
                        "Suggest using AngleKind::Perpendicular instead of AngleKind::Other({})",
                        theta.to_degrees()
                    ),
                });
            }
            _ => {}
        }
    }
    lints
}
