//! Efficient Zoo Problem Zolver.
//! Solves 2D constraint systems.

use std::collections::HashSet;

pub use crate::analysis::FreedomAnalysis;
use crate::analysis::{Analysis, NoAnalysis, SolveOutcomeAnalysis};
pub use crate::constraint_request::ConstraintRequest;
pub use crate::constraints::Constraint;
use crate::constraints::ConstraintEntry;
pub use crate::error::*;
pub use crate::solver::Config;
// Only public for now so that I can benchmark it.
// TODO: Replace this with an end-to-end benchmark,
// or find a different way to structure modules.
pub use crate::id::{Id, IdGenerator};
use crate::solver::Model;
pub use warnings::{Warning, WarningContent};

mod analysis;
mod constraint_request;
/// Each kind of constraint we support.
mod constraints;
/// Geometric data (lines, points, etc).
pub mod datatypes;
mod error;
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

/// Data from a successful solved system.
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

/// Just like [`SolveOutcome`] except it also contains the result of
/// expensive numeric analysis on the final solved system.
// This is just like `SolveOutcomeAnalysis<FreedomAnalysis>`,
// except it doesn't leak the private trait `Analysis`.
#[derive(Debug)]
pub struct SolveOutcomeFreedomAnalysis {
    /// Extra analysis for the system,
    /// which is probably expensive to compute.
    pub analysis: FreedomAnalysis,
    /// Other data.
    pub outcome: SolveOutcome,
}

impl AsRef<SolveOutcome> for SolveOutcomeFreedomAnalysis {
    fn as_ref(&self) -> &SolveOutcome {
        &self.outcome
    }
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

/// Returned when ezpz could not solve a system.
#[derive(Debug)]
pub struct FailureOutcome {
    /// The error that stopped the system from being solved.
    pub error: NonLinearSystemError,
    /// Other warnings which might have contributed,
    /// or might be suboptimal for other reasons.
    pub warnings: Vec<Warning>,
    /// Size of the system.
    pub num_vars: usize,
    /// Size of the system.
    pub num_eqs: usize,
}

/// Given some initial guesses, constrain them.
/// Returns the same variables in the same order, but constrained.
pub fn solve_with_priority(
    reqs: &[ConstraintRequest],
    initial_guesses: Vec<(Id, f64)>,
    config: Config,
) -> Result<SolveOutcome, FailureOutcome> {
    let out = solve_with_priority_inner::<NoAnalysis>(reqs, initial_guesses, config)?;
    Ok(out.outcome)
}

/// Just like [`solve_with_priority`] except it also does some expensive analysis steps
/// at the end. This lets it calculate helpful data for the user, like degrees of freedom.
/// Should not be called on every iteration of a system when you change the initial values!
/// Just call this when you change the constraint structure.
pub fn solve_with_priority_analysis(
    reqs: &[ConstraintRequest],
    initial_guesses: Vec<(Id, f64)>,
    config: Config,
) -> Result<SolveOutcomeFreedomAnalysis, FailureOutcome> {
    let out = solve_with_priority_inner::<FreedomAnalysis>(reqs, initial_guesses, config)?;
    Ok(SolveOutcomeFreedomAnalysis {
        analysis: out.analysis,
        outcome: out.outcome,
    })
}

/// Given some initial guesses, constrain them.
/// Returns the same variables in the same order, but constrained.
pub(crate) fn solve_with_priority_inner<A: Analysis>(
    reqs: &[ConstraintRequest],
    initial_guesses: Vec<(Id, f64)>,
    config: Config,
) -> Result<SolveOutcomeAnalysis<A>, FailureOutcome> {
    // When there's no constraints, return early.
    // Use the initial guesses as the final values.
    if reqs.is_empty() {
        return Ok(SolveOutcomeAnalysis {
            analysis: A::no_constraints(),
            outcome: SolveOutcome {
                unsatisfied: Vec::new(),
                final_values: initial_guesses
                    .into_iter()
                    .map(|(_id, guess)| guess)
                    .collect(),
                iterations: 0,
                warnings: Vec::new(),
                priority_solved: 0,
            },
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
    let mut constraint_subset: Vec<ConstraintEntry<'_>> = Vec::with_capacity(total_constraints);

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
                if outcome.outcome.is_unsatisfied() {
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
    // The unwrap default value is used when
    // there were 0 constraints.
    Ok(res.unwrap_or(SolveOutcomeAnalysis {
        analysis: A::no_constraints(),
        outcome: SolveOutcome {
            unsatisfied: Vec::new(),
            final_values: initial_guesses
                .into_iter()
                .map(|(_id, guess)| guess)
                .collect(),
            iterations: 0,
            warnings: Vec::new(),
            priority_solved: lowest_priority,
        },
    }))
}

fn solve_inner<A: Analysis>(
    constraints: &[ConstraintEntry<'_>],
    initial_guesses: Vec<(Id, f64)>,
    config: Config,
) -> Result<SolveOutcomeAnalysis<A>, FailureOutcome> {
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
        Err(error) => {
            return Err(FailureOutcome {
                error,
                warnings,
                num_vars,
                num_eqs,
            });
        }
    };

    let mut unsatisfied: Vec<usize> = Vec::new();
    let outcome = model.solve_gauss_newton(&mut values, config);
    warnings.extend(model.warnings.lock().unwrap().drain(..));
    let success = match outcome {
        Ok(o) => o,
        Err(error) => {
            return Err(FailureOutcome {
                error,
                warnings,
                num_vars,
                num_eqs,
            });
        }
    };
    let cs: Vec<_> = constraints.iter().map(|c| c.constraint).collect();
    let layout = solver::Layout::new(&Vec::new(), cs.as_slice(), config);
    for constraint in constraints {
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
    let analysis = match A::analyze(model) {
        Ok(o) => o,
        Err(error) => {
            return Err(FailureOutcome {
                error,
                warnings,
                num_vars,
                num_eqs,
            });
        }
    };

    Ok(SolveOutcomeAnalysis {
        outcome: SolveOutcome {
            priority_solved: 0,
            unsatisfied,
            final_values: values,
            iterations: success.iterations,
            warnings,
        },
        analysis,
    })
}
