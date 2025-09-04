use kittycad_modeling_cmds::shared::Angle;
use libm::{cos, sin};

use crate::{IdGenerator, id::Id};

#[derive(Clone, Copy, Debug)]
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

    /// Get all IDs of all variables, i.e. p0.x, p0.y, p1.x, p1.y
    pub fn all_variables(&self) -> [Id; 4] {
        [
            self.p0.id_x(),
            self.p0.id_y(),
            self.p1.id_x(),
            self.p1.id_y(),
        ]
    }
}

/// A circle.
#[derive(Clone, Copy, Debug)]
pub struct Circle {
    pub center: DatumPoint,
    pub radius: DatumDistance,
}

impl Circle {
    /// Get all IDs of all variables, i.e. center components and radius.
    pub fn all_variables(&self) -> [Id; 3] {
        [self.center.id_x(), self.center.id_y(), self.radius.id]
    }
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
