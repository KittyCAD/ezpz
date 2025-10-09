//! Efficient Zoo Problem Zolver.
//! Solves 2D constraint systems.

pub use crate::constraints::Constraint;
pub use crate::solver::Config;
// Only public for now so that I can benchmark it.
// TODO: Replace this with an end-to-end benchmark,
// or find a different way to structure modules.
pub use crate::id::{Id, IdGenerator};
use crate::solver::Model;
use faer::sparse::CreationError;
pub use warnings::{Warning, WarningContent};

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
mod warnings;

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
    /// Which constraints couldn't be satisfied
    pub unsatisfied: Vec<usize>,
    /// Each variable's final value.
    pub final_values: Vec<f64>,
    /// How many iterations of Newton's method were required?
    pub iterations: usize,
    /// Anything that went wrong either in problem definition or during solving it.
    pub warnings: Vec<Warning>,
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
    let mut warnings = warnings::lint(constraints);
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

    let mut newton_faer_config = newton_faer::NewtonCfg::sparse().with_adaptive(true);
    newton_faer_config.max_iter = config.max_iterations;

    let outcome = newton_faer::solve(&mut model, &mut values, newton_faer_config)
        .map_err(|errs| Error::Solver(Box::new(errs.into_error())));
    let mut unsatisfied: Vec<usize> = Vec::new();
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
        unsatisfied,
        final_values: values,
        iterations,
        warnings,
    })
}
