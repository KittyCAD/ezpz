mod executor;
mod instruction;
mod parser;

use instruction::Instruction;

#[derive(Debug, PartialEq)]
pub struct PointGuess {
    pub point: Label,
    pub guess: Point,
}

#[derive(Debug)]
pub struct Problem {
    pub instructions: Vec<Instruction>,
    pub inner_points: Vec<Label>,
    pub point_guesses: Vec<PointGuess>,
}

#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub struct Point {
    pub x: f64,
    pub y: f64,
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

impl PartialEq<str> for Label {
    fn eq(&self, other: &str) -> bool {
        self.0 == other
    }
}

impl Problem {
    pub fn points(&self) -> &[Label] {
        &self.inner_points
    }
}
