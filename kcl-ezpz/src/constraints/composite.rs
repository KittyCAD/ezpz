use crate::datatypes::{
    AngleKind,
    inputs::{DatumCircle, DatumCircularArc, DatumLineSegment, DatumPoint},
};

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

    /// Constrains this point to bisect this arc.
    pub fn point_bisects_arc(arc: DatumCircularArc, point: DatumPoint) -> [Self; 2] {
        // To make a point bisect an arc, just:
        // - ensure the point is on the arc, with PointArcCoincident
        // - draw a line from the arc's center to that point, and
        //   make sure the arc's start and end points are symmetric
        //   across that line.
        let center_to_point = DatumLineSegment {
            p0: arc.center,
            p1: point,
        };
        [
            Constraint::PointArcCoincident(arc, point),
            Constraint::Symmetric(center_to_point, arc.start, arc.end),
        ]
    }

    /// Ensure the arc starts at the line's `p1`, and is tangential
    /// (i.e. moves smoothly from the line into an arc).
    /// Note: this does not add an arc constraint, you probably want to
    /// use [`Self::Arc`] too.
    pub fn tangential_arc(arc: DatumCircularArc, line: DatumLineSegment) -> [Self; 2] {
        [
            Self::PointsCoincident(line.p1, arc.start),
            Self::lines_perpendicular([
                line,
                DatumLineSegment {
                    p0: arc.center,
                    p1: arc.start,
                },
            ]),
        ]
    }

    /// Constrains these two lines to be parallel, and to have the given perpendicular distance.
    pub fn parallel_lines_distance(lines: [DatumLineSegment; 2], distance: f64) -> [Self; 2] {
        [
            Constraint::lines_parallel(lines),
            Constraint::PointLineDistance(lines[0].p0, lines[1], distance),
        ]
    }

    /// Constraints a circle and a circular arc to have the same center and radius.
    pub fn circle_arc_coincident(circle: DatumCircle, arc: DatumCircularArc) -> [Self; 2] {
        [
            Constraint::PointsCoincident(circle.center, arc.center),
            Constraint::LinesEqualLength(
                DatumLineSegment {
                    p0: arc.center,
                    p1: arc.start,
                },
                DatumLineSegment {
                    p0: arc.center,
                    p1: arc.end,
                },
            ),
        ]
    }
}
