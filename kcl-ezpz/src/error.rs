use faer::{
    linalg::svd::SvdError,
    sparse::{CreationError, FaerError, linalg::LuError},
};

use crate::Id;

/// All errors that could occur when solving a system.
#[derive(thiserror::Error, Debug)]
#[non_exhaustive]
pub enum Error {
    #[error("{0}")]
    NonLinearSystemError(#[from] NonLinearSystemError),
    #[error("No guess was given for point {label}")]
    MissingGuess { label: String },
    #[error("You gave a guess for points which weren't defined: {labels:?}")]
    UnusedGuesses { labels: Vec<String> },
    #[error("You referred to the point {label} but it was never defined")]
    UndefinedPoint { label: String },
}

/// Errors that could occur when running the core Newton-Gauss solve.
#[derive(thiserror::Error, Debug)]
#[non_exhaustive]
pub enum NonLinearSystemError {
    /// ID was not found.
    #[error("ID {0} not found")]
    NotFound(Id),
    /// There should be exactly 1 guess per variable, but you supplied the wrong number.
    #[error(
        "There should be exactly 1 guess per variable, but you supplied {labels} variables and must {guesses} guesses"
    )]
    WrongNumberGuesses { labels: usize, guesses: usize },
    /// Constraint references a variable that doesn't appear in the initial guesses.
    #[error(
        "Constraint {constraint_id} references variable {variable} but no such variable appears in your initial guesses."
    )]
    MissingGuess { constraint_id: usize, variable: Id },
    /// Faer: could not create a matrix.
    #[error("Could not create matrix: {error}")]
    FaerMatrix {
        #[from]
        error: CreationError,
    },
    /// Faer: general error.
    #[error("Something went wrong in faer: {error}")]
    Faer {
        #[from]
        error: FaerError,
    },
    /// Faer: could not solve the matrix in the Newton-Gauss loop.
    #[error("Something went wrong doing matrix solves in faer: {error}")]
    FaerSolve {
        #[from]
        error: LuError,
    },
    /// Faer: could not decompose Jacobian.
    #[error("Something went wrong doing SVD in faer")]
    FaerSvd(SvdError),
    /// Solver did not find a solution within the allowed number of iterations.
    /// Consider raising the iterations?
    #[error("Could not find a solution in the allowed number of iterations")]
    DidNotConverge,
    /// You provided an empty constraint system.
    #[error("Cannot solve an empty system")]
    EmptySystemNotAllowed,
}
