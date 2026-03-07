//! The final solved values of various geometry.

/// A 2D point that ezpz solved for, i.e. found values for all its variables.
#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub struct Point {
    #[allow(missing_docs)]
    pub x: f64,
    #[allow(missing_docs)]
    pub y: f64,
}

/// Points can be easily converted to/from an (x, y) pair.
impl From<(f64, f64)> for Point {
    fn from((x, y): (f64, f64)) -> Self {
        Self { x, y }
    }
}

/// Points can be easily converted to/from an (x, y) pair.
impl From<Point> for (f64, f64) {
    fn from(Point { x, y }: Point) -> Self {
        (x, y)
    }
}

/// A 2D circle that ezpz solved for, i.e. found values for all its variables.
#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub struct Circle {
    /// Radius of the circle.
    pub radius: f64,
    /// Center of the circle.
    pub center: Point,
}

/// A 2D circular arc that ezpz solved for, i.e. found values for all its variables.
#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub struct Arc {
    /// A point at one end of the arc.
    /// This doesn't specifically mean the start or end or anything.
    pub a: Point,
    /// A point at one end of the arc.
    /// This doesn't specifically mean the start or end or anything.
    pub b: Point,
    /// Center of the arc.
    pub center: Point,
}

impl std::fmt::Display for Point {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "({},{})", self.x, self.y)
    }
}

impl Point {
    /// Euclidean distance between two points.
    pub fn euclidean_distance(&self, r: Point) -> f64 {
        use crate::vector::V;
        V::new(self.x, self.y).euclidean_distance(V::new(r.x, r.y))
    }
}

/// Component of a 2D point.
#[derive(Clone, Copy, Eq, PartialEq, Debug)]
pub enum Component {
    /// Horizontal (X) component.
    X,
    /// Vertical (Y) component.
    Y,
}
