use crate::datatypes::Angle;

use super::{Component, Label};

#[derive(Debug)]
pub enum Instruction {
    DeclarePoint(DeclarePoint),
    DeclareCircle(DeclareCircle),
    FixPointComponent(FixPointComponent),
    Vertical(Vertical),
    Horizontal(Horizontal),
    Distance(Distance),
    Parallel(Parallel),
    Perpendicular(Perpendicular),
    AngleLine(AngleLine),
    PointsCoincident(PointsCoincident),
    CircleRadius(CircleRadius),
    Tangent(Tangent),
    FixCenterPointComponent(FixCenterPointComponent),
    LinesEqualLength(LinesEqualLength),
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
pub struct LinesEqualLength {
    pub line0: (Label, Label),
    pub line1: (Label, Label),
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
pub struct FixPointComponent {
    pub point: Label,
    pub component: Component,
    pub value: f64,
}

#[derive(Debug)]
pub struct FixCenterPointComponent {
    pub circle: Label,
    pub center_component: Component,
    pub value: f64,
}
