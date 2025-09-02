mod executor;
mod instruction;
mod parser;

pub use executor::Outcome;
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

impl std::fmt::Display for Point {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "({},{})", self.x, self.y)
    }
}

impl Point {
    #[allow(dead_code)]
    pub(crate) fn euclidean_distance(&self, r: Point) -> f64 {
        crate::constraints::euclidean_distance((self.x, self.y), (r.x, r.y))
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
