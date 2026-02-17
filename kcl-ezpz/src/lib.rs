#![doc = include_str!("../README.md")]

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
pub use solve_outcome::{FailureOutcome, SolveOutcome, SolveOutcomeFreedomAnalysis};
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
/// Residual field visualization (optional).
#[cfg(feature = "residual-viz")]
pub mod residual_viz;
mod solve_outcome;
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

/// Given some initial guesses, constrain them.
/// Returns the same variables in the same order, but constrained.
/// ```
/// use kcl_ezpz::{Config, solve, Constraint, ConstraintRequest, datatypes::inputs::DatumPoint, IdGenerator};
///
/// // Define the geometry.
/// let mut ids = IdGenerator::default();
/// let p = DatumPoint::new(&mut ids);
/// let q = DatumPoint::new(&mut ids);
///
/// // Define constraints on the geometry.
/// let requests = [
///     // Fix P to the origin
///     ConstraintRequest::highest_priority(Constraint::Fixed(p.id_x(), 0.0)),
///     ConstraintRequest::highest_priority(Constraint::Fixed(p.id_y(), 0.0)),
///     // P and Q should be 4 units apart.
///     ConstraintRequest::highest_priority(Constraint::Distance(p, q, 4.0)),
/// ];
///
/// // Provide some initial guesses to the solver for their locations.
/// let initial_guesses = vec![
///     (p.id_x(), 0.0),
///     (p.id_y(), -0.02),
///     (q.id_x(), 4.39),
///     (q.id_y(), 4.38),
/// ];
///
/// // Run the solver!
/// let _outcome = solve(
///     &requests,
///     initial_guesses,
///     // You can customize the config, but for this example, we'll just use the default.
///     Config::default(),
/// );
/// ```
pub fn solve(
    reqs: &[ConstraintRequest],
    initial_guesses: Vec<(Id, f64)>,
    config: Config,
) -> Result<SolveOutcome, FailureOutcome> {
    let out = solve_with_priority_inner::<NoAnalysis>(reqs, initial_guesses, config)?;
    Ok(out.outcome)
}

/// Just like [`solve`] except it also does some expensive analysis steps
/// at the end. This lets it calculate helpful data for the user, like degrees of freedom.
/// Should not be called on every iteration of a system when you change the initial values!
/// Just call this when you change the constraint structure.
/// ```
/// use kcl_ezpz::{Config, solve_analysis, Constraint, ConstraintRequest, datatypes::inputs::DatumPoint, IdGenerator};
///
/// // Define the geometry.
/// let mut ids = IdGenerator::default();
/// let p = DatumPoint::new(&mut ids);
/// let q = DatumPoint::new(&mut ids);
///
/// // Define constraints on the geometry.
/// let requests = [
///     // Fix P to the origin
///     ConstraintRequest::highest_priority(Constraint::Fixed(p.id_x(), 0.0)),
///     ConstraintRequest::highest_priority(Constraint::Fixed(p.id_y(), 0.0)),
///     // P and Q should be 4 units apart.
///     ConstraintRequest::highest_priority(Constraint::Distance(p, q, 4.0)),
/// ];
///
/// // Provide some initial guesses to the solver for their locations.
/// let initial_guesses = vec![
///     (p.id_x(), 0.0),
///     (p.id_y(), -0.02),
///     (q.id_x(), 4.39),
///     (q.id_y(), 4.38),
/// ];
///
/// // Run the solver!
/// let solver_res = solve_analysis(
///     &requests,
///     initial_guesses,
///     Config::default(),
/// );
/// let analysis = solver_res.unwrap().analysis;
/// let underconstrained_vars = analysis.underconstrained();
/// // P is fully constrained, because it's completely fixed to the origin.
/// assert!(!underconstrained_vars.contains(&p.id_x()));
/// assert!(!underconstrained_vars.contains(&p.id_y()));
/// // Q is underconstrained. It has to be 4 units away from Q, but there's many
/// // possible positions for Q. It could be at (4, 0), (-4, 0), etc etc.
/// assert!(underconstrained_vars.contains(&q.id_x()));
/// assert!(underconstrained_vars.contains(&q.id_y()));
/// ```
pub fn solve_analysis(
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
            constraint: c.constraint(),
            priority: c.priority(),
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
        let mut residual2 = 0.0;
        let mut degenerate = false;
        constraint.constraint.residual(
            &layout,
            &values,
            &mut residual0,
            &mut residual1,
            &mut residual2,
            &mut degenerate,
        );
        let satisfied = is_satisfied(
            constraint.constraint.residual_dim(),
            [residual0, residual1, residual2],
        );
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

    let lowest_priority = constraints
        .iter()
        .map(|c| c.priority)
        .max()
        .unwrap_or_default();
    Ok(SolveOutcomeAnalysis {
        outcome: SolveOutcome {
            priority_solved: lowest_priority,
            unsatisfied,
            final_values: values,
            iterations: success.iterations,
            warnings,
        },
        analysis,
    })
}

fn is_satisfied(residual_dim: usize, residuals: [f64; 3]) -> bool {
    let sat0 = residuals[0].abs() < EPSILON;
    let sat1 = residuals[1].abs() < EPSILON;
    let sat2 = residuals[2].abs() < EPSILON;
    match residual_dim {
        1 => sat0,
        2 => sat0 && sat1,
        3 => sat0 && sat1 && sat2,
        other => unreachable!(
            "Unsupported number of residuals {other}, the `residual` method must be modified."
        ),
    }
}

#[cfg(test)]
mod basic_tests {
    use super::*;

    #[test]
    fn test_is_satisfied_0() {
        let actual = is_satisfied(1, [1e-8, 44.0, 44.0]);
        let expected = true;
        assert_eq!(actual, expected);
    }

    #[test]
    fn test_is_satisfied_1() {
        let actual = is_satisfied(2, [1e-8, 1e-8, 44.0]);
        let expected = true;
        assert_eq!(actual, expected);
    }

    #[test]
    fn test_is_satisfied_2() {
        let actual = is_satisfied(3, [1e-8, 1e-8, 1e-8]);
        let expected = true;
        assert_eq!(actual, expected);
    }

    #[test]
    fn test_is_unsatisfied_0() {
        let actual = is_satisfied(1, [44.0, 44.0, 44.0]);
        let expected = false;
        assert_eq!(actual, expected);
    }

    #[test]
    fn test_is_unsatisfied_1() {
        let actual = is_satisfied(2, [1e-8, 44.0, 44.0]);
        let expected = false;
        assert_eq!(actual, expected);
    }

    #[test]
    fn test_is_unsatisfied_2() {
        let actual = is_satisfied(3, [44.0, 1e-8, 1e-8]);
        let expected = false;
        assert_eq!(actual, expected);
    }
}
