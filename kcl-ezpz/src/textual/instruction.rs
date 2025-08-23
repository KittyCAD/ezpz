use super::{Component, Label};

#[derive(Debug)]
pub enum Instruction {
    DeclarePoint(DeclarePoint),
    FixPointComponent(FixPointComponent),
    Vertical(Vertical),
    Horizontal(Horizontal),
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
