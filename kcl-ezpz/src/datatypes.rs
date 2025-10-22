use libm::{cos, sin};

use crate::{IdGenerator, id::Id};

pub trait Datum {
    fn all_variables(&self) -> impl IntoIterator<Item = Id>;
}

#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
pub struct Angle {
    val: f64,
    degrees: bool,
}

impl std::fmt::Display for Angle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.degrees {
            write!(f, "{}deg", self.val)
        } else {
            write!(f, "{}rad", self.val)
        }
    }
}

impl Angle {
    pub fn from_degrees(degrees: f64) -> Self {
        Self {
            val: degrees,
            degrees: true,
        }
    }

    pub fn from_radians(radians: f64) -> Self {
        Self {
            val: radians,
            degrees: false,
        }
    }

    pub fn to_degrees(self) -> f64 {
        if self.degrees {
            self.val
        } else {
            self.val.to_degrees()
        }
    }

    pub fn to_radians(self) -> f64 {
        if self.degrees {
            self.val.to_radians()
        } else {
            self.val
        }
    }
}

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
pub struct DatumDistance {
    pub id: Id,
}

impl DatumDistance {
    pub fn new(id: Id) -> Self {
        Self { id }
    }
}

impl Datum for DatumDistance {
    fn all_variables(&self) -> impl IntoIterator<Item = Id> {
        [self.id]
    }
}

/// 2D point.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
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

    /// Id for the X component of the point.
    #[inline(always)]
    pub fn id_x(&self) -> Id {
        self.x_id
    }

    /// Id for the Y component of the point.
    #[inline(always)]
    pub fn id_y(&self) -> Id {
        self.y_id
    }
}

impl Datum for DatumPoint {
    fn all_variables(&self) -> impl IntoIterator<Item = Id> {
        [self.id_x(), self.id_y()]
    }
}

/// Line of infinite length.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
pub struct DatumLine {
    // Unusual representation of a line using two parameters, theta and A
    theta: Angle,
    #[allow(dead_code)]
    a: f64,
}

impl DatumLine {
    /// Get gradient of the line dx/dy.
    pub fn direction(&self) -> f64 {
        let dx = cos(self.theta.to_radians());
        let dy = sin(self.theta.to_radians());
        dx / dy
    }
}

/// Finite segment of a line.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
pub struct LineSegment {
    pub p0: DatumPoint,
    pub p1: DatumPoint,
}

impl LineSegment {
    pub fn new(p0: DatumPoint, p1: DatumPoint) -> Self {
        Self { p0, p1 }
    }
}

impl Datum for LineSegment {
    fn all_variables(&self) -> impl IntoIterator<Item = Id> {
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
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
pub struct Circle {
    pub center: DatumPoint,
    pub radius: DatumDistance,
}

impl Datum for Circle {
    /// Get all IDs of all variables, i.e. center components and radius.
    fn all_variables(&self) -> impl IntoIterator<Item = Id> {
        [self.center.id_x(), self.center.id_y(), self.radius.id]
    }
}

/// Arc on the perimeter of a circle.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
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

impl Datum for CircularArc {
    fn all_variables(&self) -> impl IntoIterator<Item = Id> {
        [
            self.a.id_x(),
            self.a.id_y(),
            self.b.id_x(),
            self.b.id_y(),
            self.center.id_x(),
            self.center.id_y(),
        ]
    }
}
