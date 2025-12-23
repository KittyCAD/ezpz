use crate::datatypes::Angle;

use super::{Component, Label};

#[derive(Debug)]
pub(crate) enum Instruction {
    DeclarePoint(DeclarePoint),
    DeclareCircle(DeclareCircle),
    DeclareArc(DeclareArc),
    FixPointComponent(FixPointComponent),
    Vertical(Vertical),
    Horizontal(Horizontal),
    Distance(Distance),
    Parallel(Parallel),
    Perpendicular(Perpendicular),
    AngleLine(AngleLine),
    PointsCoincident(PointsCoincident),
    PointArcCoincident(PointArcCoincident),
    Midpoint(Midpoint),
    Symmetric(Symmetric),
    CircleRadius(CircleRadius),
    Tangent(Tangent),
    ArcRadius(ArcRadius),
    FixCenterPointComponent(FixCenterPointComponent),
    LinesEqualLength(LinesEqualLength),
    IsArc(IsArc),
    PointLineDistance(PointLineDistance),
    Line(Line),
    ArcLength(ArcLength),
}

#[derive(Debug)]
pub struct Distance {
    pub label: (Label, Label),
    pub distance: f64,
}

#[derive(Debug)]
pub struct Parallel {
    pub line0: (Label, Label),
    pub line1: (Label, Label),
}

#[derive(Debug)]
pub struct CircleRadius {
    pub circle: Label,
    pub radius: f64,
}

#[derive(Debug)]
pub struct Tangent {
    pub circle: Label,
    pub line_p0: Label,
    pub line_p1: Label,
}

#[derive(Debug)]
pub struct ArcRadius {
    pub arc_label: Label,
    pub radius: f64,
}

#[derive(Debug)]
pub struct LinesEqualLength {
    pub line0: (Label, Label),
    pub line1: (Label, Label),
}

#[derive(Debug)]
pub struct IsArc {
    pub arc_label: Label,
}

#[derive(Debug)]
pub struct Line {
    pub p0: Label,
    pub p1: Label,
}

#[derive(Debug)]
pub struct Perpendicular {
    pub line0: (Label, Label),
    pub line1: (Label, Label),
}

#[derive(Debug)]
pub struct AngleLine {
    pub line0: (Label, Label),
    pub line1: (Label, Label),
    pub angle: Angle,
}

#[derive(Debug)]
pub struct PointsCoincident {
    pub point0: Label,
    pub point1: Label,
}

#[derive(Debug)]
pub struct PointArcCoincident {
    pub point: Label,
    pub arc: Label,
}

#[derive(Debug)]
pub struct Midpoint {
    pub point0: Label,
    pub point1: Label,
    pub mp: Label,
}

#[derive(Debug)]
pub struct PointLineDistance {
    pub point: Label,
    pub line_p0: Label,
    pub line_p1: Label,
    pub distance: f64,
}

#[derive(Debug)]
pub struct ArcLength {
    pub arc: Label,
    pub distance: f64,
}

#[derive(Debug)]
pub struct Symmetric {
    /// Be symmetric across this line.
    pub line: (Label, Label),
    pub p0: Label,
    pub p1: Label,
}

#[derive(Debug)]
pub struct Vertical {
    pub label: (Label, Label),
}

#[derive(Debug)]
pub struct Horizontal {
    pub label: (Label, Label),
}

#[derive(Debug)]
pub struct DeclarePoint {
    pub label: Label,
}

#[derive(Debug)]
pub struct DeclareCircle {
    pub label: Label,
}

#[derive(Debug)]
pub struct DeclareArc {
    pub label: Label,
}

#[derive(Debug)]
pub struct FixPointComponent {
    pub point: Label,
    pub component: Component,
    pub value: f64,
}

#[derive(Debug)]
pub struct FixCenterPointComponent {
    pub object: Label,
    pub center_component: Component,
    pub value: f64,
}
