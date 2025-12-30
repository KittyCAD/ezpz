use crate::datatypes::{AngleKind, inputs::DatumLineSegment};

use super::Constraint;

impl Constraint {
    /// Constrain these lines to be parallel.
    pub fn lines_parallel([l0, l1]: [DatumLineSegment; 2]) -> Self {
        // TODO: Check if all points are unique.
        // Our math can't handle a common point just yet.
        Self::LinesAtAngle(l0, l1, AngleKind::Parallel)
    }

    /// Constrain these lines to be perpendicular.
    pub fn lines_perpendicular([l0, l1]: [DatumLineSegment; 2]) -> Self {
        Self::LinesAtAngle(l0, l1, AngleKind::Perpendicular)
    }
}
