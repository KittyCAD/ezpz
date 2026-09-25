use crate::{NonLinearSystemError, SolveOutcomeFreedomAnalysis, solver::Model};

pub(crate) trait Analysis: Sized {
    fn analyze(model: Model<'_>) -> Result<Self, NonLinearSystemError>;
    fn no_constraints() -> Self;
}

#[derive(Default, Debug)]
pub(crate) struct NoAnalysis;

impl Analysis for NoAnalysis {
    #[mutants::skip]
    fn analyze(_: Model<'_>) -> Result<Self, NonLinearSystemError> {
        Ok(Self)
    }

    #[mutants::skip]
    fn no_constraints() -> Self {
        Self
    }
}

/// Results from analyzing the freedom of each variable.
/// Created from [`crate::solve_analysis`].
#[derive(Default, Debug)]
#[cfg_attr(not(feature = "unstable-exhaustive"), non_exhaustive)]
pub struct FreedomAnalysis {
    /// These variables are underconstrained, and the user could (probably should)
    /// add more constraints so that their positions are properly specified and don't
    /// depend on the initial guesses.
    underconstrained: Vec<crate::Id>,
    /// Constraints with an equation that is linearly dependent on earlier constraints'
    /// at the final solution, identified by their index in the request list.
    redundant: Vec<usize>,
    /// Rank of the Jacobian at the final solution.
    rank: usize,
    /// Number of equations (Jacobian rows) in the system.
    num_equations: usize,
}

impl Analysis for FreedomAnalysis {
    fn analyze(model: Model<'_>) -> Result<Self, NonLinearSystemError> {
        model.freedom_analysis()
    }

    #[mutants::skip]
    fn no_constraints() -> Self {
        Self::default()
    }
}

impl FreedomAnalysis {
    pub(crate) fn new(
        underconstrained: Vec<crate::Id>,
        redundant: Vec<usize>,
        rank: usize,
        num_equations: usize,
    ) -> Self {
        Self {
            underconstrained,
            redundant,
            rank,
            num_equations,
        }
    }

    /// Is any variable in the system underconstrained?
    pub fn is_underconstrained(&self) -> bool {
        !self.underconstrained.is_empty()
    }

    /// These variables are underconstrained, and the user could (probably should)
    /// add more constraints so that their positions are properly specified and don't
    /// depend on the initial guesses.
    pub fn underconstrained(&self) -> &[crate::Id] {
        &self.underconstrained
    }

    /// Just like [`FreedomAnalysis::underconstrained`] except it consumes the struct to take ownership.
    pub fn into_underconstrained(self) -> Vec<crate::Id> {
        self.underconstrained
    }

    /// Does any constraint repeat what other constraints already say?
    /// Equivalent to `self.rank() < self.num_equations()`.
    pub fn has_redundant_constraints(&self) -> bool {
        !self.redundant.is_empty()
    }

    /// Constraints which repeat, at least in part, what earlier constraints say:
    /// at the final solution, one or more of their equations is linearly dependent
    /// on the equations before it. Each is the constraint's index in the request
    /// list, the same numbering as [`crate::SolveOutcome::unsatisfied`].
    ///
    /// When two constraints say the same thing, the later one in the request list is
    /// reported. If the system is unsatisfied, a constraint listed here may be in
    /// conflict with earlier ones rather than merely repeating them.
    pub fn redundant(&self) -> &[usize] {
        &self.redundant
    }

    /// Rank of the Jacobian at the final solution: how many of the system's
    /// equations are independent.
    pub fn rank(&self) -> usize {
        self.rank
    }

    /// Number of equations in the system. Most constraints contribute one;
    /// some, like [`crate::Constraint::PointsCoincident`], contribute more.
    pub fn num_equations(&self) -> usize {
        self.num_equations
    }
}

impl From<FreedomAnalysis> for Vec<crate::Id> {
    fn from(analysis: FreedomAnalysis) -> Vec<crate::Id> {
        analysis.into_underconstrained()
    }
}

#[derive(Debug)]
pub(crate) struct SolveOutcomeAnalysis<A> {
    /// Extra analysis for the system.
    pub analysis: A,
    /// Other data.
    pub outcome: crate::SolveOutcome,
}

impl From<SolveOutcomeFreedomAnalysis> for SolveOutcomeAnalysis<FreedomAnalysis> {
    fn from(value: SolveOutcomeFreedomAnalysis) -> Self {
        Self {
            analysis: value.analysis,
            outcome: value.outcome,
        }
    }
}

impl From<SolveOutcomeAnalysis<FreedomAnalysis>> for SolveOutcomeFreedomAnalysis {
    fn from(value: SolveOutcomeAnalysis<FreedomAnalysis>) -> Self {
        Self {
            analysis: value.analysis,
            outcome: value.outcome,
        }
    }
}
