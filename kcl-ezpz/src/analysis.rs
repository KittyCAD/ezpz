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
#[derive(Default, Debug)]
pub struct FreedomAnalysis {
    /// These variables are underconstrained, and the user could (probably should)
    /// add more constraints so that their positions are properly specified and don't
    /// depend on the initial guesses.
    pub underconstrained: Vec<crate::Id>,
}

impl Analysis for FreedomAnalysis {
    fn analyze(model: Model<'_>) -> Result<Self, NonLinearSystemError> {
        model.freedom_analysis()
    }

    #[mutants::skip]
    fn no_constraints() -> Self {
        Self {
            underconstrained: Vec::new(),
        }
    }
}

impl FreedomAnalysis {
    /// Is any variable in the system underconstrained?
    pub fn is_underconstrained(&self) -> bool {
        !self.underconstrained.is_empty()
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
