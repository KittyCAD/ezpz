//! Geometric entities that can be constrained and solved by ezpz.

use crate::{Id, IdGenerator};

pub(crate) trait Datum {
    fn all_variables(&self) -> impl IntoIterator<Item = Id>;
}

/// A distance that can be determined by the constraint solver.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
pub struct DatumDistance {
    /// ID of the variable for this distance.
    pub id: Id,
}

impl DatumDistance {
    /// Create a new `DatumDistance`.
    pub fn new(id: Id) -> Self {
        Self { id }
    }
}

impl Datum for DatumDistance {
    fn all_variables(&self) -> impl IntoIterator<Item = Id> {
        [self.id]
    }
}

/// 2D point, whose position can be determined by the constraint solver.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
pub struct DatumPoint {
    /// ID of the variable for this point's X component.
    pub x_id: Id,
    /// ID of the variable for this point's Y component.
    pub y_id: Id,
}

impl DatumPoint {
    /// Create a new `DatumPoint` from an ID generator.
    pub fn new(id_generator: &mut IdGenerator) -> Self {
        Self {
            x_id: id_generator.next_id(),
            y_id: id_generator.next_id(),
        }
    }

    /// Create a new `DatumPoint` with these specific IDs.
    pub fn new_xy(x: Id, y: Id) -> Self {
        Self { x_id: x, y_id: y }
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

/// Finite segment of a line.
/// It has two points, one at each end, and those points
/// can be determined by the constraint solver.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
pub struct DatumLineSegment {
    /// Point for one end of this line.
    pub p0: DatumPoint,
    /// Point for the other end of this line.
    pub p1: DatumPoint,
}

impl DatumLineSegment {
    /// Create a new `LineSegment`.
    pub fn new(p0: DatumPoint, p1: DatumPoint) -> Self {
        Self { p0, p1 }
    }
}

impl Datum for DatumLineSegment {
    fn all_variables(&self) -> impl IntoIterator<Item = Id> {
        [
            self.p0.id_x(),
            self.p0.id_y(),
            self.p1.id_x(),
            self.p1.id_y(),
        ]
    }
}

/// A circle, whose radius and position can be determined by the constraint solver.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
pub struct DatumCircle {
    /// Center of the circle.
    pub center: DatumPoint,
    /// Radius distance of the circle.
    pub radius: DatumDistance,
}

impl Datum for DatumCircle {
    /// Get all IDs of all variables, i.e. center components and radius.
    fn all_variables(&self) -> impl IntoIterator<Item = Id> {
        [self.center.id_x(), self.center.id_y(), self.radius.id]
    }
}

/// Arc on the perimeter of a circle.
/// The arc's start, end and center can be determined by the constraint solver.
/// The arc always goes counter-clockwise from start to end.
/// To get a clockwise arc, swap start and end.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
pub struct DatumCircularArc {
    /// Center of the circle
    pub center: DatumPoint,
    /// Start point of the arc.
    /// Distance(start, center) == Distance(end, center)
    pub start: DatumPoint,
    /// End point of the arc.
    /// Distance(start, center) == Distance(end, center)
    pub end: DatumPoint,
}

impl Datum for DatumCircularArc {
    fn all_variables(&self) -> impl IntoIterator<Item = Id> {
        [
            self.start.id_x(),
            self.start.id_y(),
            self.end.id_x(),
            self.end.id_y(),
            self.center.id_x(),
            self.center.id_y(),
        ]
    }
}
