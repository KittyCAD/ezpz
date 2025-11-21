use crate::{IdGenerator, id::Id};

pub trait Datum {
    fn all_variables(&self) -> impl IntoIterator<Item = Id>;
}

/// Possible angles, with specific descriptors for special angles
/// like parallel or perpendicular.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
pub enum AngleKind {
    Parallel,
    Perpendicular,
    Other(Angle),
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
    pub x_id: Id,
    pub y_id: Id,
}

impl DatumPoint {
    pub fn new(id_generator: &mut IdGenerator) -> Self {
        Self {
            x_id: id_generator.next_id(),
            y_id: id_generator.next_id(),
        }
    }

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

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn angle_conversions_and_display() {
        let deg = Angle::from_degrees(180.0);
        assert!((deg.to_radians() - PI).abs() < 1e-12);
        assert_eq!(deg.to_string(), "180deg");

        let rad = Angle::from_radians(PI);
        assert!((rad.to_degrees() - 180.0).abs() < 1e-12);
        assert_eq!(rad.to_string(), format!("{PI}rad"));
    }

    #[test]
    fn datum_collects_all_variables() {
        let mut ids = IdGenerator::default();
        let p0 = DatumPoint::new(&mut ids);
        let p1 = DatumPoint::new(&mut ids);
        let line = LineSegment::new(p0, p1);
        assert_eq!(
            line.all_variables().into_iter().collect::<Vec<_>>(),
            vec![0, 1, 2, 3]
        );

        let circle = Circle {
            center: p0,
            radius: DatumDistance::new(ids.next_id()),
        };
        assert_eq!(
            circle.all_variables().into_iter().collect::<Vec<_>>(),
            vec![0, 1, 4]
        );

        let arc = CircularArc {
            center: p0,
            a: p1,
            b: DatumPoint::new_xy(6, 7),
        };
        assert_eq!(
            arc.all_variables().into_iter().collect::<Vec<_>>(),
            vec![2, 3, 6, 7, 0, 1]
        );
    }
}
