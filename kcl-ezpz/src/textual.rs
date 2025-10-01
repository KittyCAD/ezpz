mod executor;
mod geometry_variables;
mod instruction;
mod parser;

use std::str::FromStr;

pub use executor::Outcome;
use instruction::Instruction;
use winnow::Parser;

use crate::{textual::parser::parse_problem, vector::V};

#[derive(Debug, PartialEq)]
pub struct PointGuess {
    pub point: Label,
    pub guess: Point,
}

#[derive(Debug, PartialEq)]
pub struct ScalarGuess {
    pub scalar: Label,
    pub guess: f64,
}

#[derive(Debug)]
pub struct Problem {
    pub instructions: Vec<Instruction>,
    pub inner_points: Vec<Label>,
    pub inner_circles: Vec<Label>,
    pub inner_arcs: Vec<Label>,
    pub point_guesses: Vec<PointGuess>,
    pub scalar_guesses: Vec<ScalarGuess>,
}

impl FromStr for Problem {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        parse_problem.parse(s).map_err(|e| e.to_string())
    }
}

#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub struct Circle {
    pub radius: f64,
    pub center: Point,
}

#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub struct Arc {
    pub a: Point,
    pub b: Point,
    pub center: Point,
}

impl std::fmt::Display for Point {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "({},{})", self.x, self.y)
    }
}

impl Point {
    #[allow(dead_code)]
    pub(crate) fn euclidean_distance(&self, r: Point) -> f64 {
        V::new(self.x, self.y).euclidean_distance(V::new(r.x, r.y))
    }
}

#[derive(Debug)]
pub enum Component {
    X,
    Y,
}

#[derive(Debug, Eq, PartialEq, Clone, Hash)]
pub struct Label(String);

impl From<&str> for Label {
    fn from(value: &str) -> Self {
        Self(value.to_owned())
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

impl Problem {
    pub fn points(&self) -> &[Label] {
        &self.inner_points
    }
    pub fn circles(&self) -> &[Label] {
        &self.inner_circles
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
