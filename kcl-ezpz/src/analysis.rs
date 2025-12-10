use crate::{NonLinearSystemError, SolveOutcomeFreedomAnalysis, solver::Model};

pub(crate) trait Analysis: Sized {
    fn analyze(model: Model<'_>) -> Result<Self, NonLinearSystemError>;
    fn no_constraints() -> Self;
}

#[derive(Default, Debug)]
pub(crate) struct NoAnalysis;

impl Analysis for NoAnalysis {
    fn analyze(_: Model<'_>) -> Result<Self, NonLinearSystemError> {
        Ok(Self)
    }

    fn no_constraints() -> Self {
        Self
    }
}

#[derive(Default, Debug)]
pub struct FreedomAnalysis {
    pub is_underconstrained: bool,
}

impl Analysis for FreedomAnalysis {
    fn analyze(model: Model<'_>) -> Result<Self, NonLinearSystemError> {
        let is_underconstrained = model.is_underconstrained()?;
        Ok(Self {
            is_underconstrained,
        })
    }

    fn no_constraints() -> Self {
        Self {
            is_underconstrained: true,
        }
    }
}

#[derive(Debug)]
pub struct SolveOutcomeAnalysis<A> {
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
