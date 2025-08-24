use kittycad_modeling_cmds::shared::Angle;
use libm::{cos, sin};

use crate::{IdGenerator, id::Id};

/// 1D distance.
pub type Distance = f64;

pub struct DatumDistance {
    pub id: Id,
}

impl DatumDistance {
    pub fn new(id: Id) -> Self {
        Self { id }
    }
}

/// 2D point.
#[derive(Clone, Copy, Debug)]
pub struct DatumPoint {
    pub(crate) x_id: Id,
    pub(crate) y_id: Id,
}

impl DatumPoint {
    pub fn new(id_generator: &mut IdGenerator) -> Self {
        Self {
            x_id: id_generator.next_id(),
            y_id: id_generator.next_id(),
        }
    }
}

impl DatumPoint {
    /// Id for the X component of the point.
    pub fn id_x(&self) -> Id {
        self.x_id
    }

    /// Id for the Y component of the point.
    pub fn id_y(&self) -> Id {
        self.y_id
    }
}

/// Line of infinite length.
#[derive(Clone, Copy, Debug)]
pub struct DatumLine {
    // Unusual representation of a line using two parameters, theta and A
    theta: Angle,
    a: f64,
}

impl DatumLine {
    /// Get gradient of the line dx/dy.
    pub fn direction(&self) -> f64 {
        let dx = cos(self.theta.to_radians());
        let dy = sin(self.theta.to_radians());
        dx / dy
    }

    /// Get an arbitrary point on this line.
    pub fn some_point(&self) -> kittycad_modeling_cmds::shared::Point2d<f64> {
        let x = -self.a * sin(self.theta.to_radians());
        let y = self.a * cos(self.theta.to_radians());
        kittycad_modeling_cmds::shared::Point2d { x, y }
    }
}

/// Finite segment of a line.
#[derive(Clone, Copy, Debug)]
pub struct LineSegment {
    pub p0: DatumPoint,
    pub p1: DatumPoint,
}

impl LineSegment {
    pub fn new(p0: DatumPoint, p1: DatumPoint) -> Self {
        Self { p0, p1 }
    }
}

/// A circle.
#[allow(dead_code)]
pub struct Circle {
    pub center: DatumPoint,
    pub radius: DatumDistance,
}

/// Arc on the perimeter of a circle.
#[allow(dead_code)]
pub struct CircularArc {
    /// Center of the circle
    pub center: DatumPoint,
    /// Lies on the arc.
    /// Distance(A,center) == Distance(B,center)
    pub a: DatumPoint,
    /// Lies on the arc.
    /// Distance(A,center) == Distance(B,center)
    pub b: DatumPoint,
}
