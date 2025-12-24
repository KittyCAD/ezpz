mod executor;
mod geometry_variables;
mod instruction;
mod parser;

use std::str::FromStr;

pub use executor::ConstraintSystem;
pub use executor::Outcome;
pub use executor::OutcomeAnalysis;
use instruction::Instruction;
use winnow::Parser;

use crate::datatypes::outputs::Point;
use crate::textual::parser::parse_problem;

#[allow(missing_docs)]
#[derive(Debug, PartialEq)]
pub struct PointGuess {
    pub point: Label,
    pub guess: Point,
}

#[allow(missing_docs)]
#[derive(Debug, PartialEq)]
pub struct ScalarGuess {
    pub scalar: Label,
    pub guess: f64,
}

#[allow(missing_docs)]
#[derive(Debug)]
pub struct Problem {
    instructions: Vec<Instruction>,
    inner_points: Vec<Label>,
    inner_circles: Vec<Label>,
    inner_arcs: Vec<Label>,
    inner_lines: Vec<(Label, Label)>,
    point_guesses: Vec<PointGuess>,
    scalar_guesses: Vec<ScalarGuess>,
}

impl FromStr for Problem {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        parse_problem.parse(s).map_err(|e| e.to_string())
    }
}

/// The label of a variable being solved for in the system.
/// E.g. `p.x` or `p.y` or `arc.center`.
#[derive(Debug, Eq, PartialEq, Clone, Hash)]
pub struct Label(String);

impl From<&str> for Label {
    fn from(value: &str) -> Self {
        Self(value.to_owned())
    }
}

impl From<Label> for String {
    fn from(value: Label) -> Self {
        value.0
    }
}

impl PartialEq<&str> for Label {
    fn eq(&self, other: &&str) -> bool {
        &self.0 == other
    }
}

impl PartialEq<String> for Label {
    fn eq(&self, other: &String) -> bool {
        &self.0 == other
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_label() {
        let l = Label("x".to_owned());
        assert_eq!(l, "x");
        assert_eq!(l, "x".to_owned());
        let l2 = Label::from("x");
        assert_eq!(l, l2);
    }

    #[test]
    fn test_point_str() {
        let p = Point { x: 1.0, y: 2.0 };
        assert_eq!(p.to_string(), "(1,2)");
    }
}
