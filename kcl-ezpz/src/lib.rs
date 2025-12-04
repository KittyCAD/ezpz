//! Efficient Zoo Problem Zolver.
//! Solves 2D constraint systems.

use std::collections::HashSet;

pub use crate::constraint_request::ConstraintRequest;
pub use crate::constraints::Constraint;
use crate::constraints::ConstraintEntry;
pub use crate::solver::Config;
// Only public for now so that I can benchmark it.
// TODO: Replace this with an end-to-end benchmark,
// or find a different way to structure modules.
pub use crate::id::{Id, IdGenerator};
use crate::solver::Model;
use faer::sparse::CreationError;
pub use warnings::{Warning, WarningContent};

mod constraint_request;
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
        "Constraint {constraint_id} references variable {variable} but no such variable appears in your initial guesses."
    )]
    MissingGuess { constraint_id: usize, variable: Id },
    #[error("Could not create matrix: {error}")]
    FaerMatrix {
        #[from]
        error: CreationError,
    },
    #[error("Could not find a solution in the allowed number of iterations")]
    DidNotConverge,
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
    /// What is the lowest priority that got solved?
    /// 0 is the highest priority. Larger numbers are lower priority.
    pub priority_solved: u32,
}

impl SolveOutcome {
    /// Were all constraints satisfied?
    pub fn is_satisfied(&self) -> bool {
        self.unsatisfied.is_empty()
    }

    /// Were any constraints unsatisfied?
    pub fn is_unsatisfied(&self) -> bool {
        !self.is_satisfied()
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
pub fn solve_with_priority(
    reqs: &[ConstraintRequest],
    initial_guesses: Vec<(Id, f64)>,
    config: Config,
) -> Result<SolveOutcome, FailureOutcome> {
    // When there's no constraints, return early.
    // Use the initial guesses as the final values.
    if reqs.is_empty() {
        return Ok(SolveOutcome {
            unsatisfied: Vec::new(),
            final_values: initial_guesses
                .into_iter()
                .map(|(_id, guess)| guess)
                .collect(),
            iterations: 0,
            warnings: Vec::new(),
            priority_solved: 0,
        });
    }

    let reqs: Vec<_> = reqs
        .iter()
        .enumerate()
        .map(|(id, c)| ConstraintEntry {
            constraint: &c.constraint,
            priority: c.priority,
            id,
        })
        .collect();

    // Find all the priority levels, and put them into order from highest to lowest priority.
    let priorities: HashSet<_> = reqs.iter().map(|c| c.priority).collect();
    let mut priorities: Vec<_> = priorities.into_iter().collect();
    let lowest_priority = priorities.iter().min().copied().unwrap_or(0);
    priorities.sort();

    // Handle the case with 0 constraints.
    // (this gets used below, if the per-constraint loop never returns).
    let mut res = None;
    let total_constraints = reqs.len();

    // Try solving, starting with only the highest priority constraints,
    // adding more and more until we eventually either finish all constraints,
    // or cannot find a solution that satisfies all of them.
    let mut constraint_subset: Vec<ConstraintEntry> = Vec::with_capacity(total_constraints);

    for curr_max_priority in priorities {
        constraint_subset.clear();
        for req in &reqs {
            if req.priority <= curr_max_priority {
                constraint_subset.push(req.to_owned()); // Notice: this clones.
            }
        }
        let solve_res = solve_inner(
            constraint_subset.as_slice(),
            initial_guesses.clone(),
            config,
        );

        match solve_res {
            Ok(outcome) => {
                // If there were unsatisfied constraints, then there's no point trying to add more lower-priority constraints,
                // just return now.
                if outcome.is_unsatisfied() {
                    return Ok(res.unwrap_or(outcome));
                }
                // Otherwise, continue the loop again, adding higher-priority constraints.
                res = Some(outcome);
            }
            // If this constraint couldn't be solved,
            Err(e) => {
                // then return a previous solved system with fewer (higher-priority) constraints,
                // or if there was no such previous system, then this was the first run,
                // and we should just return the error.
                return res.ok_or(e);
            }
        }
    }
    Ok(res.unwrap_or(SolveOutcome {
        priority_solved: lowest_priority,
        unsatisfied: Vec::new(),
        final_values: initial_guesses
            .into_iter()
            .map(|(_id, guess)| guess)
            .collect(),
        iterations: 0,
        warnings: Vec::new(),
    }))
}

/// Solve, assuming all constraints are the same priority.
pub fn solve(
    constraints: &[Constraint],
    initial_guesses: Vec<(Id, f64)>,
    config: Config,
) -> Result<SolveOutcome, FailureOutcome> {
    let constraints: Vec<_> = constraints
        .iter()
        .enumerate()
        .map(|(id, c)| ConstraintEntry {
            constraint: c,
            id,
            priority: 0,
        })
        .collect();
    solve_inner(&constraints, initial_guesses, config)
}

fn solve_inner(
    constraints: &[ConstraintEntry],
    initial_guesses: Vec<(Id, f64)>,
    config: Config,
) -> Result<SolveOutcome, FailureOutcome> {
    let num_vars = initial_guesses.len();
    let num_eqs = constraints
        .iter()
        .map(|c| c.constraint.residual_dim())
        .sum();
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

    let mut unsatisfied: Vec<usize> = Vec::new();
    // let outcome = newton_faer::solve(&mut model, &mut values, newton_faer_config)
    //     .map_err(|errs| Error::Solver(Box::new(errs.into_error())));
    let outcome = model.run_newtons_method(&mut values, config);
    warnings.extend(model.warnings.lock().unwrap().drain(..));
    let iterations = match outcome {
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
    let cs: Vec<_> = constraints.iter().map(|c| c.constraint).collect();
    let layout = crate::solver::Layout::new(&Vec::new(), cs.as_slice(), config);
    for constraint in constraints.iter() {
        let mut residual0 = 0.0;
        let mut residual1 = 0.0;
        let mut degenerate = false;
        constraint.constraint.residual(
            &layout,
            &values,
            &mut residual0,
            &mut residual1,
            &mut degenerate,
        );
        let satisfied = match constraint.constraint.residual_dim() {
            1 => residual0.abs() < EPSILON,
            2 => residual0.abs() < EPSILON && residual1.abs() < EPSILON,
            other => unreachable!(
                "Unsupported number of residuals {other}, the `residual` method must be modified."
            ),
        };
        if !satisfied {
            unsatisfied.push(constraint.id);
        }
    }

    Ok(SolveOutcome {
        priority_solved: 0,
        unsatisfied,
        final_values: values,
        iterations,
        warnings,
    })
}
