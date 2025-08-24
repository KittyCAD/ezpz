use super::{Component, Label};

#[derive(Debug)]
pub enum Instruction {
    DeclarePoint(DeclarePoint),
    FixPointComponent(FixPointComponent),
    Vertical(Vertical),
    Horizontal(Horizontal),
    Distance(Distance),
    Parallel(Parallel),
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
pub struct FixPointComponent {
    pub point: Label,
    pub component: Component,
    pub value: f64,
}
