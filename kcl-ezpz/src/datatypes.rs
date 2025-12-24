pub mod inputs;
pub mod outputs;

/// Possible angles, with specific descriptors for special angles
/// like parallel or perpendicular.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
#[cfg_attr(not(feature = "unstable-exhaustive"), non_exhaustive)]
pub enum AngleKind {
    /// The two lines should be parallel to each other.
    Parallel,
    /// The two lines should be perpendicular to each other.
    Perpendicular,
    /// The two lines should meet at this angle.
    Other(Angle),
}

/// A measurement of a particular angle, could be degrees or radians.
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
    /// Create an angle of this many degrees.
    pub fn from_degrees(degrees: f64) -> Self {
        Self {
            val: degrees,
            degrees: true,
        }
    }

    /// Create an angle of this many radians.
    pub fn from_radians(radians: f64) -> Self {
        Self {
            val: radians,
            degrees: false,
        }
    }

    /// How large is this angle, in degrees?
    pub fn to_degrees(self) -> f64 {
        if self.degrees {
            self.val
        } else {
            self.val.to_degrees()
        }
    }

    /// How large is this angle, in radians?
    pub fn to_radians(self) -> f64 {
        if self.degrees {
            self.val.to_radians()
        } else {
            self.val
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::IdGenerator;

    use super::*;
    use inputs::*;
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
        let line = DatumLineSegment::new(p0, p1);
        assert_eq!(
            line.all_variables().into_iter().collect::<Vec<_>>(),
            vec![0, 1, 2, 3]
        );

        let circle = DatumCircle {
            center: p0,
            radius: DatumDistance::new(ids.next_id()),
        };
        assert_eq!(
            circle.all_variables().into_iter().collect::<Vec<_>>(),
            vec![0, 1, 4]
        );

        let arc = DatumCircularArc {
            center: p0,
            start: p1,
            end: DatumPoint::new_xy(6, 7),
        };
        assert_eq!(
            arc.all_variables().into_iter().collect::<Vec<_>>(),
            vec![2, 3, 6, 7, 0, 1]
        );
    }
}
