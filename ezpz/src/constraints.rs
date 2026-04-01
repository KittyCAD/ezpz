use crate::{
    EPSILON,
    datatypes::inputs::*,
    datatypes::*,
    id::Id,
    solver::Layout,
    vector::{Rotation2, V},
};
use std::f64::consts::PI;

/// Constructors for constraints which are a composition of
/// existing constraints.
mod composite;

#[derive(Clone, Copy, Debug)]
pub(crate) struct ConstraintEntry<'c> {
    /// The constraint itself.
    pub constraint: &'c Constraint,
    /// The constraint's ID.
    pub id: usize,
    /// The constraint's priority. 0 is highest, larger numbers are lower.
    pub priority: u32,
}

impl AsRef<Constraint> for ConstraintEntry<'_> {
    fn as_ref(&self) -> &Constraint {
        self.constraint
    }
}

/// Each geometric constraint we support.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
#[cfg_attr(not(feature = "unstable-exhaustive"), non_exhaustive)]
pub enum Constraint {
    /// This line must be tangent to the circle
    /// (i.e. touches its perimeter in exactly one place)
    LineTangentToCircle(DatumLineSegment, DatumCircle),
    /// These two circles should be tangent.
    /// This could include internal tangency (where one circle is inside the other),
    /// or external (where they're adjacent).
    CircleTangentToCircle(DatumCircle, DatumCircle),
    /// These two points should be a given distance apart.
    Distance(DatumPoint, DatumPoint, f64),
    /// These two points should have distance equal to the given variable.
    DistanceVar(DatumPoint, DatumPoint, DatumDistance),
    /// These two points should be a given vertical distance apart.
    VerticalDistance(DatumPoint, DatumPoint, f64),
    /// These two points should be a given horizontal distance apart.
    HorizontalDistance(DatumPoint, DatumPoint, f64),
    /// These two points have the same Y value.
    Vertical(DatumLineSegment),
    /// These two points have the same X value.
    Horizontal(DatumLineSegment),
    /// These lines meet at this angle.
    LinesAtAngle(DatumLineSegment, DatumLineSegment, AngleKind),
    /// Some scalar value is fixed.
    Fixed(Id, f64),
    /// These two scalar values are the same.
    /// E.g. set two circles to have the same radius.
    ScalarEqual(Id, Id),
    /// These two points must coincide.
    PointsCoincident(DatumPoint, DatumPoint),
    /// Constraint radius of a circle
    CircleRadius(DatumCircle, f64),
    /// These lines should be the same distance.
    LinesEqualLength(DatumLineSegment, DatumLineSegment),
    /// The arc should have the given radius.
    ArcRadius(DatumCircularArc, f64),
    /// These 3 points should form an arc,
    /// i.e. `a` and `b` should be equidistant from `center`.
    Arc(DatumCircularArc),
    /// The given point should be the midpoint along the given line.
    Midpoint(DatumLineSegment, DatumPoint),
    /// The given point should be the given (perpendicular, i.e. minimum Euclidean) distance away from the line.
    PointLineDistance(DatumPoint, DatumLineSegment, f64),
    /// The given point should be the given (vertical) distance away from the line.
    VerticalPointLineDistance(DatumPoint, DatumLineSegment, f64),
    /// The given point should be the given (horizontal) distance away from the line.
    HorizontalPointLineDistance(DatumPoint, DatumLineSegment, f64),
    /// These two points should be symmetric across the given line.
    Symmetric(DatumLineSegment, DatumPoint, DatumPoint),
    /// This point should lie on this arc.
    PointArcCoincident(DatumCircularArc, DatumPoint),
    /// The arc should have this length.
    ArcLength(DatumCircularArc, f64),
    /// The arc should span this angle.
    ArcAngle(DatumCircularArc, Angle),
}

/// Describes one value in one row of the Jacobian matrix.
#[derive(Clone, Copy)]
pub(crate) struct JacobianVar {
    /// Which variable are we talking about?
    /// Corresponds to one column in the row.
    pub id: Id,
    /// What value is its partial derivative?
    pub partial_derivative: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PointArcCoincidentPart {
    Interior,
    Start,
    End,
}

#[cfg(feature = "dbg-jac")]
impl std::fmt::Debug for JacobianVar {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "∂ col={} pd={:.3}", self.id, self.partial_derivative)
    }
}

impl Constraint {
    /// Extend `out` with the primitive variable IDs that this constraint's
    /// residual equations depend on.
    ///
    /// "Dependent" means changing one of the emitted IDs can change this
    /// constraint's residual value. This is intentionally narrower than
    /// [`Constraint::extend_associated_variable_ids`], which reports every ID
    /// structurally present inside the attached geometry. For example,
    /// [`Constraint::HorizontalDistance`] only emits the two X-component IDs
    /// because its residual does not depend on either point's Y component, and
    /// [`Constraint::CircleRadius`] only emits the radius ID.
    ///
    /// The output collection is owned by the caller so this API does not
    /// allocate. Callers that need deduplication can pass a set-like type.
    pub fn extend_dependent_variable_ids(&self, out: &mut impl Extend<Id>) {
        match self {
            Constraint::LineTangentToCircle(line, circle) => {
                out.extend(line.all_variables());
                out.extend(circle.all_variables());
            }
            Constraint::CircleTangentToCircle(circle0, circle1) => {
                out.extend(circle0.all_variables());
                out.extend(circle1.all_variables());
            }
            Constraint::Distance(p0, p1, _dist) => {
                out.extend(p0.all_variables());
                out.extend(p1.all_variables());
            }
            Constraint::DistanceVar(p0, p1, d) => {
                out.extend(p0.all_variables());
                out.extend(p1.all_variables());
                out.extend(d.all_variables());
            }
            Constraint::VerticalDistance(p0, p1, _dist) => out.extend([p0.id_y(), p1.id_y()]),
            Constraint::HorizontalDistance(p0, p1, _dist) => out.extend([p0.id_x(), p1.id_x()]),
            Constraint::Vertical(line) => out.extend([line.p0.id_x(), line.p1.id_x()]),
            Constraint::Horizontal(line) => out.extend([line.p0.id_y(), line.p1.id_y()]),
            Constraint::LinesAtAngle(line0, line1, _angle) => {
                out.extend(line0.all_variables());
                out.extend(line1.all_variables());
            }
            Constraint::Fixed(id, _scalar) => out.extend([*id]),
            Constraint::ScalarEqual(x, y) => out.extend([*x, *y]),
            Constraint::PointsCoincident(p0, p1) => {
                out.extend(p0.all_variables());
                out.extend(p1.all_variables());
            }
            Constraint::CircleRadius(circle, _radius) => out.extend([circle.radius.id]),
            Constraint::LinesEqualLength(line0, line1) => {
                out.extend(line0.all_variables());
                out.extend(line1.all_variables());
            }
            Constraint::ArcRadius(arc, _radius) => out.extend(arc.all_variables()),
            Constraint::Arc(arc) => out.extend(arc.all_variables()),
            Constraint::Midpoint(line, point) => {
                out.extend([line.p0.id_x(), line.p1.id_x(), point.id_x()]);
                out.extend([line.p0.id_y(), line.p1.id_y(), point.id_y()]);
            }
            Constraint::PointLineDistance(point, line, _distance) => {
                out.extend(point.all_variables());
                out.extend(line.all_variables());
            }
            Constraint::VerticalPointLineDistance(point, line, _distance) => {
                out.extend(line.all_variables());
                out.extend(point.all_variables());
            }
            Constraint::HorizontalPointLineDistance(point, line, _distance) => {
                out.extend(line.all_variables());
                out.extend(point.all_variables());
            }
            Constraint::Symmetric(line, a, b) => {
                out.extend(line.all_variables());
                out.extend(a.all_variables());
                out.extend(b.all_variables());
            }
            Constraint::PointArcCoincident(circular_arc, point) => {
                out.extend(circular_arc.all_variables());
                out.extend(point.all_variables());
            }
            Constraint::ArcLength(circular_arc, _dist) => out.extend(circular_arc.all_variables()),
            Constraint::ArcAngle(circular_arc, _angle) => out.extend(circular_arc.all_variables()),
        }
    }

    /// Extend `out` with the primitive variable IDs associated with the
    /// geometry attached to this constraint.
    ///
    /// "Associated" means every variable ID belonging to the datums mentioned
    /// by this constraint, even if some of those IDs do not affect the residual
    /// directly. This is intentionally broader than
    /// [`Constraint::extend_dependent_variable_ids`], which only reports IDs
    /// that can change the residual itself. For example,
    /// [`Constraint::HorizontalDistance`] emits both points' X and Y IDs
    /// because both full points are associated with the constraint, and
    /// [`Constraint::CircleRadius`] emits the circle center IDs as well as the
    /// radius ID because all three belong to the associated circle datum.
    ///
    /// The output collection is owned by the caller so this API does not
    /// allocate. Callers that need deduplication can pass a set-like type.
    pub fn extend_associated_variable_ids(&self, out: &mut impl Extend<Id>) {
        match self {
            Constraint::LineTangentToCircle(line, circle) => {
                out.extend(line.all_variables());
                out.extend(circle.all_variables());
            }
            Constraint::CircleTangentToCircle(circle0, circle1) => {
                out.extend(circle0.all_variables());
                out.extend(circle1.all_variables());
            }
            Constraint::Distance(p0, p1, _dist) => {
                out.extend(p0.all_variables());
                out.extend(p1.all_variables());
            }
            Constraint::DistanceVar(p0, p1, d) => {
                out.extend(p0.all_variables());
                out.extend(p1.all_variables());
                out.extend(d.all_variables());
            }
            Constraint::VerticalDistance(p0, p1, _dist)
            | Constraint::HorizontalDistance(p0, p1, _dist) => {
                out.extend(p0.all_variables());
                out.extend(p1.all_variables());
            }
            Constraint::Vertical(line) | Constraint::Horizontal(line) => {
                out.extend(line.all_variables());
            }
            Constraint::LinesAtAngle(line0, line1, _angle) => {
                out.extend(line0.all_variables());
                out.extend(line1.all_variables());
            }
            Constraint::Fixed(id, _scalar) => out.extend([*id]),
            Constraint::ScalarEqual(x, y) => out.extend([*x, *y]),
            Constraint::PointsCoincident(p0, p1) => {
                out.extend(p0.all_variables());
                out.extend(p1.all_variables());
            }
            Constraint::CircleRadius(circle, _radius) => out.extend(circle.all_variables()),
            Constraint::LinesEqualLength(line0, line1) => {
                out.extend(line0.all_variables());
                out.extend(line1.all_variables());
            }
            Constraint::ArcRadius(arc, _radius) => out.extend(arc.all_variables()),
            Constraint::Arc(arc) => out.extend(arc.all_variables()),
            Constraint::Midpoint(line, point) => {
                out.extend(line.all_variables());
                out.extend(point.all_variables());
            }
            Constraint::PointLineDistance(point, line, _distance) => {
                out.extend(point.all_variables());
                out.extend(line.all_variables());
            }
            Constraint::VerticalPointLineDistance(point, line, _distance) => {
                out.extend(line.all_variables());
                out.extend(point.all_variables());
            }
            Constraint::HorizontalPointLineDistance(point, line, _distance) => {
                out.extend(line.all_variables());
                out.extend(point.all_variables());
            }
            Constraint::Symmetric(line, a, b) => {
                out.extend(line.all_variables());
                out.extend(a.all_variables());
                out.extend(b.all_variables());
            }
            Constraint::PointArcCoincident(circular_arc, point) => {
                out.extend(circular_arc.all_variables());
                out.extend(point.all_variables());
            }
            Constraint::ArcLength(circular_arc, _dist) => out.extend(circular_arc.all_variables()),
            Constraint::ArcAngle(circular_arc, _angle) => out.extend(circular_arc.all_variables()),
        }
    }

    /// For each row of the Jacobian matrix, which variables are involved in them?
    pub(crate) fn nonzeroes(&self, row0: &mut Vec<Id>, row1: &mut Vec<Id>, _row2: &mut Vec<Id>) {
        match self {
            Constraint::LineTangentToCircle(line, circle) => {
                row0.extend(line.all_variables());
                row0.extend(circle.all_variables());
            }
            Constraint::CircleTangentToCircle(circle0, circle1) => {
                row0.extend(circle0.all_variables());
                row0.extend(circle1.all_variables());
            }
            Constraint::Distance(p0, p1, _dist) => {
                row0.extend(p0.all_variables());
                row0.extend(p1.all_variables());
            }
            Constraint::DistanceVar(p0, p1, d) => {
                row0.extend(p0.all_variables());
                row0.extend(p1.all_variables());
                row0.extend(d.all_variables());
            }
            Constraint::VerticalDistance(p0, p1, _dist) => {
                row0.extend([p0.id_y(), p1.id_y()]);
            }
            Constraint::HorizontalDistance(p0, p1, _dist) => {
                row0.extend([p0.id_x(), p1.id_x()]);
            }
            Constraint::Vertical(line) => row0.extend([line.p0.id_x(), line.p1.id_x()]),
            Constraint::Horizontal(line) => row0.extend([line.p0.id_y(), line.p1.id_y()]),
            Constraint::LinesAtAngle(line0, line1, _angle) => {
                row0.extend(line0.all_variables());
                row0.extend(line1.all_variables());
            }
            Constraint::Fixed(id, _scalar) => row0.push(*id),
            Constraint::ScalarEqual(x, y) => row0.extend([x, y]),
            Constraint::PointsCoincident(p0, p1) => {
                row0.push(p0.id_x());
                row0.push(p1.id_x());
                row1.push(p0.id_y());
                row1.push(p1.id_y());
            }
            Constraint::CircleRadius(circle, _radius) => row0.extend([circle.radius.id]),
            Constraint::LinesEqualLength(line0, line1) => {
                row0.extend(line0.all_variables());
                row0.extend(line1.all_variables());
            }
            Constraint::ArcRadius(arc, radius) => {
                // This is really just equivalent to 2 constraints,
                // distance(center, start) and distance(center, end).
                let constraints = (
                    Constraint::Distance(arc.center, arc.start, *radius),
                    Constraint::Distance(arc.center, arc.end, *radius),
                );
                constraints.0.nonzeroes(row0, row1, _row2);
                constraints.1.nonzeroes(row1, row0, _row2);
            }
            Constraint::Arc(arc) => {
                row0.extend(arc.all_variables());
            }
            Constraint::Midpoint(line, point) => {
                row0.extend(&[line.p0.id_x(), line.p1.id_x(), point.id_x()]);
                row1.extend(&[line.p0.id_y(), line.p1.id_y(), point.id_y()]);
            }
            Constraint::PointLineDistance(point, line, _distance) => {
                row0.extend(point.all_variables());
                row0.extend(line.all_variables());
            }
            Constraint::VerticalPointLineDistance(point, line, _distance) => {
                row0.extend(line.all_variables());
                row0.extend(point.all_variables());
            }
            Constraint::HorizontalPointLineDistance(point, line, _distance) => {
                row0.extend(line.all_variables());
                row0.extend(point.all_variables());
            }
            Constraint::Symmetric(line, a, b) => {
                // Equation: rej(A - P, Q - P) + rej(B - P, Q - P) = 0
                row0.extend(line.all_variables());
                row0.extend(a.all_variables());
                row0.extend(b.all_variables());
                row1.extend(line.all_variables());
                row1.extend(a.all_variables());
                row1.extend(b.all_variables());
            }
            Constraint::PointArcCoincident(circular_arc, point) => {
                row0.extend(circular_arc.all_variables());
                row0.extend(point.all_variables());
                row1.extend(circular_arc.all_variables());
                row1.extend(point.all_variables());
            }
            Constraint::ArcLength(circular_arc, _dist) => {
                row0.extend(circular_arc.all_variables());
                row1.extend(circular_arc.all_variables());
            }
            Constraint::ArcAngle(circular_arc, angle) => Constraint::LinesAtAngle(
                DatumLineSegment {
                    p0: circular_arc.center,
                    p1: circular_arc.start,
                },
                DatumLineSegment {
                    p0: circular_arc.center,
                    p1: circular_arc.end,
                },
                AngleKind::Other(*angle),
            )
            .nonzeroes(row0, row1, _row2),
        }
    }

    /// How close is this constraint to being satisfied?
    /// For performance reasons (avoiding allocations), this doesn't return a `Vec<f64>`,
    /// instead it takes one as a mutable argument and writes out all residuals to that.
    /// Most constraints have a residual measured as a single number (scalar),
    /// but some constraints have two residuals (e.g. one for the X axis and one for the Y axis).
    /// That's why there's two possible residuals to calculate (and therefore, two &mut residual to write into).
    pub(crate) fn residual(
        &self,
        layout: &Layout,
        current_assignments: &[f64],
        residual0: &mut f64,
        residual1: &mut f64,
        _residual2: &mut f64,
        degenerate: &mut bool,
    ) {
        match self {
            Constraint::LineTangentToCircle(line, circle) => {
                // Get current state of the entities.
                let p0_x = current_assignments[layout.index_of(line.p0.id_x())];
                let p0_y = current_assignments[layout.index_of(line.p0.id_y())];
                let p0 = V::new(p0_x, p0_y);
                let p1_x = current_assignments[layout.index_of(line.p1.id_x())];
                let p1_y = current_assignments[layout.index_of(line.p1.id_y())];
                let p1 = V::new(p1_x, p1_y);
                let center_x = current_assignments[layout.index_of(circle.center.id_x())];
                let center_y = current_assignments[layout.index_of(circle.center.id_y())];
                let radius = current_assignments[layout.index_of(circle.radius.id)];
                let circle_center = V::new(center_x, center_y);

                // Calculate the unsigned distance from the circle's center to the line.
                let u = p1 - p0;
                let mag_u = u.magnitude();

                // Handle degenerate line case
                if mag_u <= EPSILON {
                    // TODO: Could revert to point circle constraint here
                    *residual0 = 0.0;
                    *degenerate = true;
                    return;
                }

                let v = circle_center - p0;
                let abs_cross_uv = u.cross_2d(v).abs();
                let cen_dist = abs_cross_uv / mag_u;

                // Handle ambiguous case where circle center is on the line
                if cen_dist <= EPSILON {
                    *residual0 = 0.0;
                    *degenerate = true;
                    return;
                }

                *residual0 = cen_dist - radius;
            }
            Constraint::CircleTangentToCircle(circle_a, circle_b) => {
                // Get current state of the entities.
                let ax = current_assignments[layout.index_of(circle_a.center.id_x())];
                let ay = current_assignments[layout.index_of(circle_a.center.id_y())];
                let ar = current_assignments[layout.index_of(circle_a.radius.id)];
                let bx = current_assignments[layout.index_of(circle_b.center.id_x())];
                let by = current_assignments[layout.index_of(circle_b.center.id_y())];
                let br = current_assignments[layout.index_of(circle_b.radius.id)];

                // Evaluate residuals for both internal and external tangency
                // See https://github.com/KittyCAD/ezpz-sympy/pull/1
                let residual_internal =
                    -((ax - bx).powi(2) + (ay - by).powi(2)).sqrt() + (ar - br).abs();
                let residual_external = ar + br - ((ax - bx).powi(2) + (ay - by).powi(2)).sqrt();
                let is_internal =
                    (((ax - bx).powi(2) + (ay - by).powi(2)).sqrt() - (ar - br).abs()).abs()
                        < (ar + br - ((ax - bx).powi(2) + (ay - by).powi(2)).sqrt()).abs();

                // Use whichever kind of tangency has a smaller residual.
                let smaller_residual = if is_internal {
                    residual_internal
                } else {
                    residual_external
                };
                *residual0 = smaller_residual;
            }
            Constraint::Distance(p0, p1, expected_distance) => {
                let p0_x = current_assignments[layout.index_of(p0.id_x())];
                let p0_y = current_assignments[layout.index_of(p0.id_y())];
                let p0 = V::new(p0_x, p0_y);
                let p1_x = current_assignments[layout.index_of(p1.id_x())];
                let p1_y = current_assignments[layout.index_of(p1.id_y())];
                let p1 = V::new(p1_x, p1_y);
                let actual_distance = p0.euclidean_distance(p1);
                *residual0 = actual_distance - expected_distance;
            }
            Constraint::DistanceVar(p, q, d) => {
                let px = current_assignments[layout.index_of(p.id_x())];
                let py = current_assignments[layout.index_of(p.id_y())];
                let qx = current_assignments[layout.index_of(q.id_x())];
                let qy = current_assignments[layout.index_of(q.id_y())];
                let d = current_assignments[layout.index_of(d.id)];
                let residual = -d + ((px - qx).powi(2) + (py - qy).powi(2)).sqrt();
                *residual0 = residual;
            }
            Constraint::VerticalDistance(p0, p1, expected_distance) => {
                let p0_y = current_assignments[layout.index_of(p0.id_y())];
                let p1_y = current_assignments[layout.index_of(p1.id_y())];
                // Residual:
                // p0.y - p1.y = d
                // p0.y - p1.y - d = 0
                *residual0 = (p0_y - p1_y) - expected_distance;
            }
            Constraint::HorizontalDistance(p0, p1, expected_distance) => {
                let p0_x = current_assignments[layout.index_of(p0.id_x())];
                let p1_x = current_assignments[layout.index_of(p1.id_x())];
                *residual0 = (p0_x - p1_x) - expected_distance;
            }
            Constraint::Vertical(line) => {
                let p0_x = current_assignments[layout.index_of(line.p0.id_x())];
                let p1_x = current_assignments[layout.index_of(line.p1.id_x())];
                *residual0 = p0_x - p1_x;
            }
            Constraint::Horizontal(line) => {
                let p0_y = current_assignments[layout.index_of(line.p0.id_y())];
                let p1_y = current_assignments[layout.index_of(line.p1.id_y())];
                *residual0 = p0_y - p1_y;
            }
            Constraint::Fixed(id, expected) => {
                let actual = current_assignments[layout.index_of(*id)];
                *residual0 = actual - expected;
            }
            Constraint::ScalarEqual(x, y) => {
                // Residual equation R: x-y=0
                let vx = current_assignments[layout.index_of(*x)];
                let vy = current_assignments[layout.index_of(*y)];
                *residual0 = vx - vy;
            }
            Constraint::LinesAtAngle(line0, line1, expected_angle) => {
                let x0 = current_assignments[layout.index_of(line0.p0.id_x())];
                let y0 = current_assignments[layout.index_of(line0.p0.id_y())];
                let x1 = current_assignments[layout.index_of(line0.p1.id_x())];
                let y1 = current_assignments[layout.index_of(line0.p1.id_y())];
                let x2 = current_assignments[layout.index_of(line1.p0.id_x())];
                let y2 = current_assignments[layout.index_of(line1.p0.id_y())];
                let x3 = current_assignments[layout.index_of(line1.p1.id_x())];
                let y3 = current_assignments[layout.index_of(line1.p1.id_y())];

                let u = V::new(x1 - x0, y1 - y0);
                let v = V::new(x3 - x2, y3 - y2);

                let rot = rotation_for_angle_kind(*expected_angle);
                *residual0 = u.cross_2d(rot.apply(v));
            }
            Constraint::PointsCoincident(p0, p1) => {
                let p0_x = current_assignments[layout.index_of(p0.id_x())];
                let p0_y = current_assignments[layout.index_of(p0.id_y())];
                let p1_x = current_assignments[layout.index_of(p1.id_x())];
                let p1_y = current_assignments[layout.index_of(p1.id_y())];
                *residual0 = p0_x - p1_x;
                *residual1 = p0_y - p1_y;
            }
            Constraint::CircleRadius(circle, expected_radius) => {
                let actual_radius = current_assignments[layout.index_of(circle.radius.id)];
                *residual0 = actual_radius - *expected_radius;
            }
            Constraint::LinesEqualLength(line0, line1) => {
                let (l0, l1) = get_line_ends(current_assignments, line0, line1, layout);
                let len0 = l0.0.euclidean_distance(l0.1);
                let len1 = l1.0.euclidean_distance(l1.1);
                *residual0 = len0 - len1;
            }
            Constraint::ArcRadius(arc, radius) => {
                // This is really just equivalent to 2 constraints,
                // distance(center, start) and distance(center, end).
                let constraints = (
                    Constraint::Distance(arc.center, arc.start, *radius),
                    Constraint::Distance(arc.center, arc.end, *radius),
                );
                constraints.0.residual(
                    layout,
                    current_assignments,
                    residual0,
                    residual1,
                    _residual2,
                    degenerate,
                );
                constraints.1.residual(
                    layout,
                    current_assignments,
                    residual1,
                    residual0,
                    _residual2,
                    degenerate,
                );
            }
            Constraint::Arc(arc) => {
                let start_x = current_assignments[layout.index_of(arc.start.id_x())];
                let start_y = current_assignments[layout.index_of(arc.start.id_y())];
                let end_x = current_assignments[layout.index_of(arc.end.id_x())];
                let end_y = current_assignments[layout.index_of(arc.end.id_y())];
                let cx = current_assignments[layout.index_of(arc.center.id_x())];
                let cy = current_assignments[layout.index_of(arc.center.id_y())];
                // For numerical stability and simpler derivatives, we compare the squared
                // distances. The residual is zero if the distances are equal.
                // R = distance(center, start)² - distance(center, end)²
                let dist0_sq = (start_x - cx).powi(2) + (start_y - cy).powi(2);
                let dist1_sq = (end_x - cx).powi(2) + (end_y - cy).powi(2);

                *residual0 = dist0_sq - dist1_sq;
            }
            Constraint::Midpoint(line, point) => {
                let p = line.p0;
                let q = line.p1;
                let px = current_assignments[layout.index_of(p.id_x())];
                let py = current_assignments[layout.index_of(p.id_y())];
                let qx = current_assignments[layout.index_of(q.id_x())];
                let qy = current_assignments[layout.index_of(q.id_y())];
                let ax = current_assignments[layout.index_of(point.id_x())];
                let ay = current_assignments[layout.index_of(point.id_y())];
                // Equation:
                //   ax = (px + qx)/2,
                // ∴ ax - px/2 - qx/2 = 0
                *residual0 = ax - px / 2.0 - qx / 2.0;
                *residual1 = ay - py / 2.0 - qy / 2.0;
            }
            Constraint::PointLineDistance(point, line, target_distance) => {
                // Equation:
                //
                // Given a line in format Ax + By + C = 0,
                // and a point (px, py), then the actual distance is
                //
                // (A.px + B.py + C)  /  sqrt(A^2 + B^2)
                //
                // Note that we use a signed direction, so there's no absolute value
                // of the numerator, as you'd usually see. This stops the solver
                // from randomly flipping which side of the line the point is on.
                let px = current_assignments[layout.index_of(point.id_x())];
                let py = current_assignments[layout.index_of(point.id_y())];
                let (a, b, c) = equation_of_line(current_assignments, line, layout);

                // The above equation is a division, so make sure not to divide by zero.
                let denominator = libm::hypot(a, b);
                let is_invalid = denominator < EPSILON;
                if is_invalid {
                    *residual0 = 0.0;
                    *degenerate = true;
                    return;
                }
                let actual_distance = (a * px + b * py + c) / denominator;

                // Residual is then easy to calculate, it's just the gap between actual and target.
                let residual = actual_distance - target_distance;
                *residual0 = residual;
            }
            Constraint::VerticalPointLineDistance(point, line, desired_distance) => {
                // See notebook:
                // https://github.com/KittyCAD/ezpz-sympy/blob/main/main.py
                // Residual (scaled to avoid dividing by dx):
                // dx = qx - px
                // dy = qy - py
                // r = (ay - py - desired) * dx - dy * (ax - px)
                let ax = current_assignments[layout.index_of(point.id_x())];
                let ay = current_assignments[layout.index_of(point.id_y())];
                let px = current_assignments[layout.index_of(line.p0.id_x())];
                let py = current_assignments[layout.index_of(line.p0.id_y())];
                let qx = current_assignments[layout.index_of(line.p1.id_x())];
                let qy = current_assignments[layout.index_of(line.p1.id_y())];
                let dx = qx - px;
                let dy = qy - py;
                if dx.abs() < EPSILON || (dx * dx + dy * dy) < EPSILON {
                    // vertical or zero-length line
                    *degenerate = true;
                    return;
                }
                let residual = (ay - py - desired_distance) * dx - dy * (ax - px);
                *residual0 = residual;
            }
            Constraint::HorizontalPointLineDistance(point, line, d) => {
                // See notebook:
                // https://github.com/KittyCAD/ezpz-sympy/blob/main/main.py
                // Residual:
                // m = (qy-py)/(qx-px)
                // actual = ay - (m * (ax - px) + py)
                // residual = actual - desired_distance
                let ax = current_assignments[layout.index_of(point.id_x())];
                let ay = current_assignments[layout.index_of(point.id_y())];
                let px = current_assignments[layout.index_of(line.p0.id_x())];
                let py = current_assignments[layout.index_of(line.p0.id_y())];
                let qx = current_assignments[layout.index_of(line.p1.id_x())];
                let qy = current_assignments[layout.index_of(line.p1.id_y())];
                let dx = qx - px;
                let dy = qy - py;
                if dy.abs() < EPSILON || (dx * dx + dy * dy) < EPSILON {
                    // horizontal or zero-length line
                    *degenerate = true;
                    return;
                }
                let residual = ax - d - px - (ay - py) * (-px + qx) * (-py + qy).recip();
                *residual0 = residual;
            }
            Constraint::Symmetric(line, a, b) => {
                // Equation: reflect(a - p, q - p) - b + p
                // See notebook:
                // <https://colab.research.google.com/drive/17L_Lq-yTJOaLhDd2R0OtEe4Rwkr5RHsj#scrollTo=HpAraZ0OhKBW>

                let ax = current_assignments[layout.index_of(a.id_x())];
                let ay = current_assignments[layout.index_of(a.id_y())];
                let bx = current_assignments[layout.index_of(b.id_x())];
                let by = current_assignments[layout.index_of(b.id_y())];
                let px = current_assignments[layout.index_of(line.p0.id_x())];
                let py = current_assignments[layout.index_of(line.p0.id_y())];
                let qx = current_assignments[layout.index_of(line.p1.id_x())];
                let qy = current_assignments[layout.index_of(line.p1.id_y())];

                let a = V::new(ax, ay);
                let b = V::new(bx, by);
                let p = V::new(px, py);
                let q = V::new(qx, qy);

                let residual = (a - p).reflect(q - p) - b + p;
                *residual0 = residual.x;
                *residual1 = residual.y;
            }
            Constraint::PointArcCoincident(circular_arc, point) => {
                let cx = current_assignments[layout.index_of(circular_arc.center.id_x())];
                let cy = current_assignments[layout.index_of(circular_arc.center.id_y())];
                let c = V::new(cx, cy);

                let sx = current_assignments[layout.index_of(circular_arc.start.id_x())];
                let sy = current_assignments[layout.index_of(circular_arc.start.id_y())];
                let s = V::new(sx, sy) - c;

                let ex = current_assignments[layout.index_of(circular_arc.end.id_x())];
                let ey = current_assignments[layout.index_of(circular_arc.end.id_y())];
                let e = V::new(ex, ey) - c;

                let px = current_assignments[layout.index_of(point.id_x())];
                let py = current_assignments[layout.index_of(point.id_y())];
                let p = V::new(px, py) - c;

                let r = s.magnitude();
                let r_e = e.magnitude();
                let r_p = p.magnitude();
                if r < EPSILON || r_e < EPSILON || r_p < EPSILON {
                    *residual0 = 0.0;
                    *residual1 = 0.0;
                    *degenerate = true;
                    return;
                }

                let e_proj = e * (r / r_e);

                match classify_point_arc_coincident(s, e_proj, p) {
                    PointArcCoincidentPart::Interior => {
                        // Point is closest to arc interior
                        let f = p * (r / r_p - 1.0);
                        *residual0 = f.x;
                        *residual1 = f.y;
                    }
                    PointArcCoincidentPart::End => {
                        // Point is closest to arc end
                        let f = e_proj - p;
                        *residual0 = f.x;
                        *residual1 = f.y;
                    }
                    PointArcCoincidentPart::Start => {
                        // Point is closest to arc start
                        let f = s - p;
                        *residual0 = f.x;
                        *residual1 = f.y;
                    }
                }
            }
            Constraint::ArcLength(circular_arc, d) => {
                // Residual math, see ezpz-sympy for notebook.
                // u = a - c
                // v = b - c
                // ux = u[0]
                // uy = u[1]
                // vx = v[0]
                // vy = v[1]
                //
                // r = u.norm()
                //
                // cos_theta = u.dot(v) / (r**2)
                // sin_theta = ux * vy - uy * vx
                // # Target angle
                // alpha = d / r
                //
                // # Residuals
                // res0 = cos_theta - sp.cos(alpha)
                // res1 = sin_theta - sp.sin(alpha)

                let cx = current_assignments[layout.index_of(circular_arc.center.id_x())];
                let cy = current_assignments[layout.index_of(circular_arc.center.id_y())];
                let ax = current_assignments[layout.index_of(circular_arc.start.id_x())];
                let ay = current_assignments[layout.index_of(circular_arc.start.id_y())];
                let bx = current_assignments[layout.index_of(circular_arc.end.id_x())];
                let by = current_assignments[layout.index_of(circular_arc.end.id_y())];
                let dx = ax - cx;
                let dy = ay - cy;
                let r2 = dx * dx + dy * dy;
                if r2 < EPSILON {
                    *residual0 = 0.0;
                    *residual1 = 0.0;
                    *degenerate = true;
                    return;
                }
                let res0 = ((ax - cx) * (bx - cx) + (ay - cy) * (by - cy))
                    * ((ax - cx).powi(2) + (ay - cy).powi(2)).recip()
                    - libm::cos(d * ((ax - cx).powi(2) + (ay - cy).powi(2)).sqrt().recip());
                let res1 = ((ax - cx) * (by - cy) - (ay - cy) * (bx - cx))
                    * ((ax - cx).powi(2) + (ay - cy).powi(2)).recip()
                    - libm::sin(d * ((ax - cx).powi(2) + (ay - cy).powi(2)).sqrt().recip());

                *residual0 = res0;
                *residual1 = res1;
            }
            Constraint::ArcAngle(circular_arc, angle) => Constraint::LinesAtAngle(
                DatumLineSegment {
                    p0: circular_arc.center,
                    p1: circular_arc.start,
                },
                DatumLineSegment {
                    p0: circular_arc.center,
                    p1: circular_arc.end,
                },
                AngleKind::Other(*angle),
            )
            .residual(
                layout,
                current_assignments,
                residual0,
                residual1,
                _residual2,
                degenerate,
            ),
        }
    }

    /// How many equations does this constraint correspond to?
    /// Each equation is a residual function (a measure of error)
    pub(crate) fn residual_dim(&self) -> usize {
        match self {
            Constraint::LineTangentToCircle(..) => 1,
            Constraint::CircleTangentToCircle(..) => 1,
            Constraint::Distance(..) => 1,
            Constraint::DistanceVar(..) => 1,
            Constraint::VerticalDistance(..) => 1,
            Constraint::HorizontalDistance(..) => 1,
            Constraint::Vertical(..) => 1,
            Constraint::Horizontal(..) => 1,
            Constraint::Fixed(..) => 1,
            Constraint::ScalarEqual(_, _) => 1,
            Constraint::LinesAtAngle(..) => 1,
            Constraint::PointsCoincident(..) => 2,
            Constraint::CircleRadius(..) => 1,
            Constraint::LinesEqualLength(..) => 1,
            Constraint::ArcRadius(..) => 2,
            Constraint::Arc(..) => 1,
            Constraint::Midpoint(..) => 2,
            Constraint::PointLineDistance(..) => 1,
            Constraint::VerticalPointLineDistance(..) => 1,
            Constraint::HorizontalPointLineDistance(..) => 1,
            Constraint::Symmetric(..) => 2,
            Constraint::PointArcCoincident(..) => 2,
            Constraint::ArcLength(..) => 2,
            Constraint::ArcAngle(circular_arc, angle) => Constraint::LinesAtAngle(
                DatumLineSegment {
                    p0: circular_arc.center,
                    p1: circular_arc.start,
                },
                DatumLineSegment {
                    p0: circular_arc.center,
                    p1: circular_arc.end,
                },
                AngleKind::Other(*angle),
            )
            .residual_dim(),
        }
    }

    /// Used to construct part of a Jacobian matrix.
    /// For performance reasons (avoiding allocations), this doesn't return a
    /// `Vec<JacobianVar>` for each Jacobian row, instead takes the output rows as
    /// mutable arguments and writes out all nonzero variables for each row to
    /// one of them.
    pub(crate) fn jacobian_rows(
        &self,
        layout: &Layout,
        current_assignments: &[f64],
        row0: &mut Vec<JacobianVar>,
        row1: &mut Vec<JacobianVar>,
        _row2: &mut Vec<JacobianVar>,
        degenerate: &mut bool,
    ) {
        match self {
            Constraint::LineTangentToCircle(line, circle) => {
                // Residual: R = abs(cross(u, v)) / |u| - r
                // where u = p1 - p0 and v = c - p0.
                let x0 = current_assignments[layout.index_of(line.p0.id_x())];
                let y0 = current_assignments[layout.index_of(line.p0.id_y())];
                let p0 = V::new(x0, y0);
                let x1 = current_assignments[layout.index_of(line.p1.id_x())];
                let y1 = current_assignments[layout.index_of(line.p1.id_y())];
                let p1 = V::new(x1, y1);
                let xc = current_assignments[layout.index_of(circle.center.id_x())];
                let yc = current_assignments[layout.index_of(circle.center.id_y())];
                let center = V::new(xc, yc);

                let u = p1 - p0;
                let mag_u = u.magnitude();

                // Handle degenerate line case
                if mag_u <= EPSILON {
                    // TODO: Could revert to point circle constraint here
                    *degenerate = true;
                    return;
                }

                let v = center - p0;
                let cross_uv = u.cross_2d(v);
                let abs_cross_uv = cross_uv.abs();
                let cen_dist = abs_cross_uv / mag_u;

                // Handle ambiguous case where circle center is on the line
                if cen_dist <= EPSILON {
                    *degenerate = true;
                    return;
                }

                let mag_u_cubed = mag_u * mag_u * mag_u;
                let sign_cross_uv = cross_uv.signum();

                let dr_du_x = -(u.x * abs_cross_uv) / mag_u_cubed + (v.y * sign_cross_uv) / mag_u;
                let dr_du_y = -(u.y * abs_cross_uv) / mag_u_cubed - (v.x * sign_cross_uv) / mag_u;
                let dr_dv_x = -(u.y * sign_cross_uv) / mag_u;
                let dr_dv_y = (u.x * sign_cross_uv) / mag_u;

                let dr_dx0 = -(dr_du_x + dr_dv_x);
                let dr_dy0 = -(dr_du_y + dr_dv_y);
                let dr_dx1 = dr_du_x;
                let dr_dy1 = dr_du_y;
                let dr_dxc = dr_dv_x;
                let dr_dyc = dr_dv_y;
                let dr_dr = -1.0;

                let jvars = [
                    JacobianVar {
                        id: line.p0.id_x(),
                        partial_derivative: dr_dx0,
                    },
                    JacobianVar {
                        id: line.p0.id_y(),
                        partial_derivative: dr_dy0,
                    },
                    JacobianVar {
                        id: line.p1.id_x(),
                        partial_derivative: dr_dx1,
                    },
                    JacobianVar {
                        id: line.p1.id_y(),
                        partial_derivative: dr_dy1,
                    },
                    JacobianVar {
                        id: circle.center.id_x(),
                        partial_derivative: dr_dxc,
                    },
                    JacobianVar {
                        id: circle.center.id_y(),
                        partial_derivative: dr_dyc,
                    },
                    JacobianVar {
                        id: circle.radius.id,
                        partial_derivative: dr_dr,
                    },
                ];
                row0.extend(jvars.as_slice());
            }
            Constraint::CircleTangentToCircle(circle_a, circle_b) => {
                // Get current state of the entities.
                let ax_id = circle_a.center.id_x();
                let ay_id = circle_a.center.id_y();
                let ar_id = circle_a.radius.id;
                let bx_id = circle_b.center.id_x();
                let by_id = circle_b.center.id_y();
                let br_id = circle_b.radius.id;
                let ax = current_assignments[layout.index_of(ax_id)];
                let ay = current_assignments[layout.index_of(ay_id)];
                let ar = current_assignments[layout.index_of(ar_id)];
                let bx = current_assignments[layout.index_of(bx_id)];
                let by = current_assignments[layout.index_of(by_id)];
                let br = current_assignments[layout.index_of(br_id)];

                // Is the current state of affairs closer to internal or external tangency?
                // We should steer the system towards whichever is closer.
                let is_internal =
                    (((ax - bx).powi(2) + (ay - by).powi(2)).sqrt() - (ar - br).abs()).abs()
                        < (ar + br - ((ax - bx).powi(2) + (ay - by).powi(2)).sqrt()).abs();

                // Evaluate residuals for both internal and external tangency
                // See https://github.com/KittyCAD/ezpz-sympy/pull/1
                let pd_ax = (-ax + bx) * ((ax - bx).powi(2) + (ay - by).powi(2)).sqrt().recip();
                let pd_ay = (-ay + by) * ((ax - bx).powi(2) + (ay - by).powi(2)).sqrt().recip();
                let pd_bx = -(-ax + bx) * ((ax - bx).powi(2) + (ay - by).powi(2)).sqrt().recip();
                let pd_by = -(-ay + by) * ((ax - bx).powi(2) + (ay - by).powi(2)).sqrt().recip();
                // The partial derivative of the radius depends on whether we're steering towards
                // internal/external tangency. Because the internal tangency residual has an
                // absolute value, we don't differentiate it directly, we just check whether
                // the sign is positive or negative (i.e. which radius is larger) and differentiate
                // that specific equation for that specific case.
                let pd_ar_internal_ra_greater = 1.0;
                let pd_br_internal_ra_greater = -1.0;
                let pd_ar_internal_rb_greater = -1.0;
                let pd_br_internal_rb_greater = 1.0;
                let pd_ar_external = 1.0;
                let pd_br_external = 1.0;
                let (pd_ar, pd_br) = if is_internal {
                    if ar > br {
                        (pd_ar_internal_ra_greater, pd_br_internal_ra_greater)
                    } else {
                        (pd_ar_internal_rb_greater, pd_br_internal_rb_greater)
                    }
                } else {
                    (pd_ar_external, pd_br_external)
                };

                // Done!
                let pds = [
                    JacobianVar {
                        id: ax_id,
                        partial_derivative: pd_ax,
                    },
                    JacobianVar {
                        id: ay_id,
                        partial_derivative: pd_ay,
                    },
                    JacobianVar {
                        id: ar_id,
                        partial_derivative: pd_ar,
                    },
                    JacobianVar {
                        id: bx_id,
                        partial_derivative: pd_bx,
                    },
                    JacobianVar {
                        id: by_id,
                        partial_derivative: pd_by,
                    },
                    JacobianVar {
                        id: br_id,
                        partial_derivative: pd_br,
                    },
                ];
                row0.extend(pds.as_slice());
            }
            Constraint::Distance(p0, p1, _expected_distance) => {
                // Residual: R = sqrt((x1-x2)**2 + (y1-y2)**2) - d
                // ∂R/∂x0 = (x0 - x1) / sqrt((x0 - x1)**2 + (y0 - y1)**2)
                // ∂R/∂y0 = (y0 - y1) / sqrt((x0 - x1)**2 + (y0 - y1)**2)
                // ∂R/∂x1 = (-x0 + x1)/ sqrt((x0 - x1)**2 + (y0 - y1)**2)
                // ∂R/∂y1 = (-y0 + y1)/ sqrt((x0 - x1)**2 + (y0 - y1)**2)

                // Derivatives wrt p0 and p2's X/Y coordinates.
                let x0 = current_assignments[layout.index_of(p0.id_x())];
                let y0 = current_assignments[layout.index_of(p0.id_y())];
                let x1 = current_assignments[layout.index_of(p1.id_x())];
                let y1 = current_assignments[layout.index_of(p1.id_y())];

                let dist = V::new(x0, y0).euclidean_distance(V::new(x1, y1));
                if dist < EPSILON {
                    *degenerate = true;
                    return;
                }
                let dr_dx0 = (x0 - x1) / dist;
                let dr_dy0 = (y0 - y1) / dist;
                let dr_dx1 = (-x0 + x1) / dist;
                let dr_dy1 = (-y0 + y1) / dist;

                row0.extend(
                    [
                        JacobianVar {
                            id: p0.id_x(),
                            partial_derivative: dr_dx0,
                        },
                        JacobianVar {
                            id: p0.id_y(),
                            partial_derivative: dr_dy0,
                        },
                        JacobianVar {
                            id: p1.id_x(),
                            partial_derivative: dr_dx1,
                        },
                        JacobianVar {
                            id: p1.id_y(),
                            partial_derivative: dr_dy1,
                        },
                    ]
                    .as_slice(),
                );
            }
            Constraint::DistanceVar(p, q, d) => {
                let px = current_assignments[layout.index_of(p.id_x())];
                let py = current_assignments[layout.index_of(p.id_y())];
                let qx = current_assignments[layout.index_of(q.id_x())];
                let qy = current_assignments[layout.index_of(q.id_y())];
                /* Derivative math, from ezpz-sympy:
                residual = norm(p - q) - d
                df_dp = normalized(p - q)
                df_dq = -df_dp
                df_dd = -1
                */
                let dist = V::new(px, py).euclidean_distance(V::new(qx, qy));
                if dist < EPSILON {
                    *degenerate = true;
                    return;
                }
                let df_dpx = (px - qx) * dist.recip();
                let df_dpy = (py - qy) * dist.recip();
                let df_dqx = -(px - qx) * dist.recip();
                let df_dqy = -(py - qy) * dist.recip();
                let df_dd = -1.0;
                row0.extend(
                    [
                        JacobianVar {
                            id: p.id_x(),
                            partial_derivative: df_dpx,
                        },
                        JacobianVar {
                            id: p.id_y(),
                            partial_derivative: df_dpy,
                        },
                        JacobianVar {
                            id: q.id_x(),
                            partial_derivative: df_dqx,
                        },
                        JacobianVar {
                            id: q.id_y(),
                            partial_derivative: df_dqy,
                        },
                        JacobianVar {
                            id: d.id,
                            partial_derivative: df_dd,
                        },
                    ]
                    .as_slice(),
                );
            }
            Constraint::VerticalDistance(p0, p1, _expected_distance) => {
                // Residual: p0y - p1y - d = 0
                // ∂R/∂y0 = 1
                // ∂R/∂y1 = -1
                row0.extend(
                    [
                        JacobianVar {
                            id: p0.id_y(),
                            partial_derivative: 1.0,
                        },
                        JacobianVar {
                            id: p1.id_y(),
                            partial_derivative: -1.0,
                        },
                    ]
                    .as_slice(),
                );
            }
            Constraint::HorizontalDistance(p0, p1, _expected_distance) => {
                // Residual: p0x - p1x - d = 0
                // ∂R/∂x0 = 1
                // ∂R/∂x1 = -1
                row0.extend(
                    [
                        JacobianVar {
                            id: p0.id_x(),
                            partial_derivative: 1.0,
                        },
                        JacobianVar {
                            id: p1.id_x(),
                            partial_derivative: -1.0,
                        },
                    ]
                    .as_slice(),
                );
            }
            Constraint::Vertical(line) => {
                // Residual: R = x0 - x1
                // ∂R/∂x for p0 and p1.
                let dr_dx0 = 1.0;
                let dr_dx1 = -1.0;

                // Get the 'x' variable ID for the line's points.
                let p0_x_id = line.p0.id_x();
                let p1_x_id = line.p1.id_x();

                row0.extend(
                    [
                        JacobianVar {
                            id: p0_x_id,
                            partial_derivative: dr_dx0,
                        },
                        JacobianVar {
                            id: p1_x_id,
                            partial_derivative: dr_dx1,
                        },
                    ]
                    .as_slice(),
                );
            }
            Constraint::Horizontal(line) => {
                // Residual: R = y1 - y2
                // ∂R/∂y for p0 and p1.
                let dr_dy0 = 1.0;
                let dr_dy1 = -1.0;

                // Get the 'y' variable ID for the line's points.
                let p0_y_id = line.p0.id_y();
                let p1_y_id = line.p1.id_y();

                row0.extend(
                    [
                        JacobianVar {
                            id: p0_y_id,
                            partial_derivative: dr_dy0,
                        },
                        JacobianVar {
                            id: p1_y_id,
                            partial_derivative: dr_dy1,
                        },
                    ]
                    .as_slice(),
                );
            }
            Constraint::Fixed(id, _expected) => {
                row0.extend(
                    [JacobianVar {
                        id: *id,
                        partial_derivative: 1.0,
                    }]
                    .as_slice(),
                );
            }
            Constraint::ScalarEqual(x, y) => {
                // Residual equation R: x-y=0
                // dR/dx: 1
                // dR/dy: -1
                row0.push(JacobianVar {
                    id: *x,
                    partial_derivative: 1.0,
                });
                row0.push(JacobianVar {
                    id: *y,
                    partial_derivative: -1.0,
                });
            }
            Constraint::LinesAtAngle(line0, line1, expected_angle) => {
                let x0 = current_assignments[layout.index_of(line0.p0.id_x())];
                let y0 = current_assignments[layout.index_of(line0.p0.id_y())];
                let x1 = current_assignments[layout.index_of(line0.p1.id_x())];
                let y1 = current_assignments[layout.index_of(line0.p1.id_y())];
                let x2 = current_assignments[layout.index_of(line1.p0.id_x())];
                let y2 = current_assignments[layout.index_of(line1.p0.id_y())];
                let x3 = current_assignments[layout.index_of(line1.p1.id_x())];
                let y3 = current_assignments[layout.index_of(line1.p1.id_y())];

                let u = V::new(x1 - x0, y1 - y0);
                let v = V::new(x3 - x2, y3 - y2);

                let rot = rotation_for_angle_kind(*expected_angle);
                let df_du = rot.apply(v).perp_cw();
                let df_dv = rot.inverse().apply(u).perp_ccw();

                let pds = PartialDerivatives4Points {
                    x0: -df_du.x,
                    y0: -df_du.y,
                    x1: df_du.x,
                    y1: df_du.y,
                    x2: -df_dv.x,
                    y2: -df_dv.y,
                    x3: df_dv.x,
                    y3: df_dv.y,
                };

                let jvars = pds.jvars(line0, line1);
                row0.extend(jvars.as_slice());
            }
            Constraint::LinesEqualLength(line0, line1) => {
                // Get all points
                let x0 = current_assignments[layout.index_of(line0.p0.id_x())];
                let y0 = current_assignments[layout.index_of(line0.p0.id_y())];
                let x1 = current_assignments[layout.index_of(line0.p1.id_x())];
                let y1 = current_assignments[layout.index_of(line0.p1.id_y())];
                let l0 = (V::new(x0, y0), V::new(x1, y1));
                let x2 = current_assignments[layout.index_of(line1.p0.id_x())];
                let y2 = current_assignments[layout.index_of(line1.p0.id_y())];
                let x3 = current_assignments[layout.index_of(line1.p1.id_x())];
                let y3 = current_assignments[layout.index_of(line1.p1.id_y())];
                let l1 = (V::new(x2, y2), V::new(x3, y3));

                // Calculate lengths of each line.
                let len0 = l0.0.euclidean_distance(l0.1);
                let len1 = l1.0.euclidean_distance(l1.1);

                // Avoid division by 0
                if len0 < EPSILON || len1 < EPSILON {
                    *degenerate = true;
                    return;
                }

                // Calculate derivatives.
                let pds = PartialDerivatives4Points {
                    x0: (x0 - x1) / len0,
                    y0: (y0 - y1) / len0,
                    x1: (-x0 + x1) / len0,
                    y1: (-y0 + y1) / len0,
                    x2: (-x2 + x3) / len1,
                    y2: (-y2 + y3) / len1,
                    x3: (x2 - x3) / len1,
                    y3: (y2 - y3) / len1,
                };
                let jvars = pds.jvars(line0, line1);
                row0.extend(jvars.as_slice());
            }
            Constraint::PointsCoincident(p0, p1) => {
                // Residuals:
                // R0 = x0 - x1,
                // R1 = y0 - y1.
                //
                // For R0 = x0 - x1:
                // ∂R0/∂x0 = 1
                // ∂R0/∂y0 = 0
                // ∂R0/∂x1 = -1
                // ∂R0/∂y1 = 0
                //
                // For R1 = y0 - y1:
                // ∂R1/∂x0 = 0
                // ∂R1/∂y0 = 1
                // ∂R1/∂x1 = 0
                // ∂R1/∂y1 = -1

                let dr0_dx0 = 1.0;
                // dr0_dy0 = 0.0
                let dr0_dx1 = -1.0;
                // dr0_dy1 = 0.0

                // dr1_dx0 = 0.0
                let dr1_dy0 = 1.0;
                // dr1_dx1 = 0.0
                let dr1_dy1 = -1.0;

                // We only care about nonzero derivs here.
                row0.extend([
                    JacobianVar {
                        id: p0.id_x(),
                        partial_derivative: dr0_dx0,
                    },
                    JacobianVar {
                        id: p1.id_x(),
                        partial_derivative: dr0_dx1,
                    },
                ]);
                row1.extend([
                    JacobianVar {
                        id: p0.id_y(),
                        partial_derivative: dr1_dy0,
                    },
                    JacobianVar {
                        id: p1.id_y(),
                        partial_derivative: dr1_dy1,
                    },
                ]);
            }
            Constraint::CircleRadius(circle, _expected_radius) => {
                // Residual is R = r_expected - r_actual
                // Only partial derivative which is nonzero is ∂R/∂r_current, which is 1.
                row0.push(JacobianVar {
                    id: circle.radius.id,
                    partial_derivative: 1.0,
                });
            }
            Constraint::ArcRadius(arc, radius) => {
                // This is really just equivalent to 2 constraints,
                // distance(center, start) and distance(center, end).
                let constraints = (
                    Constraint::Distance(arc.center, arc.start, *radius),
                    Constraint::Distance(arc.center, arc.end, *radius),
                );
                constraints.0.jacobian_rows(
                    layout,
                    current_assignments,
                    row0,
                    row1,
                    _row2,
                    degenerate,
                );
                constraints.1.jacobian_rows(
                    layout,
                    current_assignments,
                    row1,
                    row0,
                    _row2,
                    degenerate,
                );
            }
            Constraint::Arc(arc) => {
                // Residual: R = (x_start-xc)²+(y_start-yc)² - (x_end-xc)²-(y_end-yc)² + CCW_constraint
                // The partial derivatives for distance part:
                // ∂R/∂x_start = 2*(x_start-xc)
                // ∂R/∂y_start = 2*(y_start-yc)
                // ∂R/∂x_end = -2*(x_end-xc)
                // ∂R/∂y_end = -2*(y_end-yc)
                // ∂R/∂xc = 2*(x_end-x_start)
                // ∂R/∂yc = 2*(y_end-y_start)
                // Plus derivatives for CCW constraint when cross < 0

                let start_x = current_assignments[layout.index_of(arc.start.id_x())];
                let start_y = current_assignments[layout.index_of(arc.start.id_y())];
                let end_x = current_assignments[layout.index_of(arc.end.id_x())];
                let end_y = current_assignments[layout.index_of(arc.end.id_y())];
                let cx = current_assignments[layout.index_of(arc.center.id_x())];
                let cy = current_assignments[layout.index_of(arc.center.id_y())];

                // TODO: Handle degenerate case here

                // Calculate derivative values for distance constraint.
                let dx_start = (start_x - cx) * 2.0;
                let dy_start = (start_y - cy) * 2.0;
                let dx_end = (end_x - cx) * -2.0;
                let dy_end = (end_y - cy) * -2.0;
                let dx_c = (end_x - start_x) * 2.0;
                let dy_c = (end_y - start_y) * 2.0;

                row0.extend([
                    JacobianVar {
                        id: arc.start.id_x(),
                        partial_derivative: dx_start,
                    },
                    JacobianVar {
                        id: arc.start.id_y(),
                        partial_derivative: dy_start,
                    },
                    JacobianVar {
                        id: arc.end.id_x(),
                        partial_derivative: dx_end,
                    },
                    JacobianVar {
                        id: arc.end.id_y(),
                        partial_derivative: dy_end,
                    },
                    JacobianVar {
                        id: arc.center.id_x(),
                        partial_derivative: dx_c,
                    },
                    JacobianVar {
                        id: arc.center.id_y(),
                        partial_derivative: dy_c,
                    },
                ]);
            }
            Constraint::Midpoint(line, point) => {
                let p = line.p0;
                let q = line.p1;
                // Equation:
                // (note that a = the midpoint)
                //   ax = (px + qx)/2,
                // ∴ ax - px/2 - qx/2 = 0
                //
                // This has partial derivatives:
                //   ∂R/∂ ax =  1
                //   ∂R/∂ px = -0.5
                //   ∂R/∂ qx = -0.5
                //   ∂R/∂ ay =  1
                //   ∂R/∂ py = -0.5
                //   ∂R/∂ qy = -0.5
                row0.extend([
                    JacobianVar {
                        id: point.id_x(),
                        partial_derivative: 1.0,
                    },
                    JacobianVar {
                        id: p.id_x(),
                        partial_derivative: -0.5,
                    },
                    JacobianVar {
                        id: q.id_x(),
                        partial_derivative: -0.5,
                    },
                ]);
                row1.extend([
                    JacobianVar {
                        id: point.id_y(),
                        partial_derivative: 1.0,
                    },
                    JacobianVar {
                        id: p.id_y(),
                        partial_derivative: -0.5,
                    },
                    JacobianVar {
                        id: q.id_y(),
                        partial_derivative: -0.5,
                    },
                ]);
            }
            Constraint::PointLineDistance(point, line, _distance) => {
                // Equation:
                //
                // Given a line in format Ax + By + C = 0,
                // and a point (px, py), then the actual distance is
                //
                // (A.px + B.py + C)  /  sqrt(A^2 + B^2)
                //
                // Note that we use a signed direction, so there's no absolute value
                // of the numerator, as you'd usually see. This stops the solver
                // from randomly flipping which side of the line the point is on.
                let px = current_assignments[layout.index_of(point.id_x())];
                let py = current_assignments[layout.index_of(point.id_y())];
                let p0x = current_assignments[layout.index_of(line.p0.id_x())];
                let p0y = current_assignments[layout.index_of(line.p0.id_y())];
                let p1x = current_assignments[layout.index_of(line.p1.id_x())];
                let p1y = current_assignments[layout.index_of(line.p1.id_y())];

                let partial_derivatives = pds_for_point_line(
                    *point,
                    line,
                    PointLineVars {
                        px,
                        py,
                        p0x,
                        p0y,
                        p1x,
                        p1y,
                    },
                );

                row0.extend(partial_derivatives);
            }
            Constraint::VerticalPointLineDistance(point, line, _distance) => {
                // See notebook:
                // https://github.com/KittyCAD/ezpz-sympy/blob/main/main.py
                let id_ax = point.id_x();
                let id_ay = point.id_y();
                let id_px = line.p0.id_x();
                let id_py = line.p0.id_y();
                let id_qx = line.p1.id_x();
                let id_qy = line.p1.id_y();
                let ax = current_assignments[layout.index_of(id_ax)];
                let ay = current_assignments[layout.index_of(id_ay)];
                let px = current_assignments[layout.index_of(id_px)];
                let py = current_assignments[layout.index_of(id_py)];
                let qx = current_assignments[layout.index_of(id_qx)];
                let qy = current_assignments[layout.index_of(id_qy)];
                let dx = qx - px;
                let dy = qy - py;
                if dx.abs() < EPSILON || (dx * dx + dy * dy) < EPSILON {
                    // vertical or zero-length line
                    *degenerate = true;
                    return;
                }
                // Residual is scaled by dx: r = (ay - py - d) * dx - dy * (ax - px)
                // Partial derivatives for the scaled residual:
                let dax = -dy;
                let day = dx;
                let dpx = qy - ay;
                let dpy = ax - qx;
                let dqx = ay - py;
                let dqy = -(ax - px);
                row0.extend([
                    JacobianVar {
                        id: id_ax,
                        partial_derivative: dax,
                    },
                    JacobianVar {
                        id: id_ay,
                        partial_derivative: day,
                    },
                    JacobianVar {
                        id: id_px,
                        partial_derivative: dpx,
                    },
                    JacobianVar {
                        id: id_py,
                        partial_derivative: dpy,
                    },
                    JacobianVar {
                        id: id_qx,
                        partial_derivative: dqx,
                    },
                    JacobianVar {
                        id: id_qy,
                        partial_derivative: dqy,
                    },
                ]);
            }
            Constraint::HorizontalPointLineDistance(point, line, _distance) => {
                // See notebook:
                // https://github.com/KittyCAD/ezpz-sympy/blob/main/main.py
                let id_ax = point.id_x();
                let id_ay = point.id_y();
                let id_px = line.p0.id_x();
                let id_py = line.p0.id_y();
                let id_qx = line.p1.id_x();
                let id_qy = line.p1.id_y();
                // let ax = current_assignments[layout.index_of(id_ax)];
                let ay = current_assignments[layout.index_of(id_ay)];
                let px = current_assignments[layout.index_of(id_px)];
                let py = current_assignments[layout.index_of(id_py)];
                let qx = current_assignments[layout.index_of(id_qx)];
                let qy = current_assignments[layout.index_of(id_qy)];
                let dx = qx - px;
                let dy = qy - py;
                if dy.abs() < EPSILON || (dx * dx + dy * dy) < EPSILON {
                    // vertical or zero-length line
                    *degenerate = true;
                    return;
                }
                let dpx = (-ay + qy) * (py - qy).recip();
                let dpy = (ay - qy) * (px - qx) * (py - qy).powi(-2);
                let dqx = (ay - py) * (py - qy).recip();
                let dqy = -(ay - py) * (px - qx) * (py - qy).powi(-2);
                let dax = 1.0;
                let day = (-px + qx) * (py - qy).recip();
                row0.extend([
                    JacobianVar {
                        id: id_ax,
                        partial_derivative: dax,
                    },
                    JacobianVar {
                        id: id_ay,
                        partial_derivative: day,
                    },
                    JacobianVar {
                        id: id_px,
                        partial_derivative: dpx,
                    },
                    JacobianVar {
                        id: id_py,
                        partial_derivative: dpy,
                    },
                    JacobianVar {
                        id: id_qx,
                        partial_derivative: dqx,
                    },
                    JacobianVar {
                        id: id_qy,
                        partial_derivative: dqy,
                    },
                ]);
            }
            Constraint::Symmetric(line, a, b) => {
                let id_px = line.p0.id_x();
                let id_py = line.p0.id_y();
                let id_qx = line.p1.id_x();
                let id_qy = line.p1.id_y();
                let id_ax = a.id_x();
                let id_ay = a.id_y();
                let id_bx = b.id_x();
                let id_by = b.id_y();

                let values = SymmetricVars {
                    px: current_assignments[layout.index_of(id_px)],
                    py: current_assignments[layout.index_of(id_py)],
                    qx: current_assignments[layout.index_of(id_qx)],
                    qy: current_assignments[layout.index_of(id_qy)],
                    ax: current_assignments[layout.index_of(a.id_x())],
                    ay: current_assignments[layout.index_of(a.id_y())],
                };
                let Some(pds) = pds_from_symmetric(values) else {
                    *degenerate = true;
                    return;
                };

                row0.extend([
                    JacobianVar {
                        id: id_px,
                        partial_derivative: pds.dpx[0],
                    },
                    JacobianVar {
                        id: id_py,
                        partial_derivative: pds.dpy[0],
                    },
                    JacobianVar {
                        id: id_qx,
                        partial_derivative: pds.dqx[0],
                    },
                    JacobianVar {
                        id: id_qy,
                        partial_derivative: pds.dqy[0],
                    },
                    JacobianVar {
                        id: id_ax,
                        partial_derivative: pds.dax[0],
                    },
                    JacobianVar {
                        id: id_ay,
                        partial_derivative: pds.day[0],
                    },
                    JacobianVar {
                        id: id_bx,
                        partial_derivative: pds.dbx[0],
                    },
                    JacobianVar {
                        id: id_by,
                        partial_derivative: pds.dby[0],
                    },
                ]);
                row1.extend([
                    JacobianVar {
                        id: id_px,
                        partial_derivative: pds.dpx[1],
                    },
                    JacobianVar {
                        id: id_py,
                        partial_derivative: pds.dpy[1],
                    },
                    JacobianVar {
                        id: id_qx,
                        partial_derivative: pds.dqx[1],
                    },
                    JacobianVar {
                        id: id_qy,
                        partial_derivative: pds.dqy[1],
                    },
                    JacobianVar {
                        id: id_ax,
                        partial_derivative: pds.dax[1],
                    },
                    JacobianVar {
                        id: id_ay,
                        partial_derivative: pds.day[1],
                    },
                    JacobianVar {
                        id: id_bx,
                        partial_derivative: pds.dbx[1],
                    },
                    JacobianVar {
                        id: id_by,
                        partial_derivative: pds.dby[1],
                    },
                ]);
            }
            Constraint::PointArcCoincident(circular_arc, point) => {
                let id_cx = circular_arc.center.id_x();
                let id_cy = circular_arc.center.id_y();
                let cx = current_assignments[layout.index_of(id_cx)];
                let cy = current_assignments[layout.index_of(id_cy)];
                let c = V::new(cx, cy);

                let id_sx = circular_arc.start.id_x();
                let id_sy = circular_arc.start.id_y();
                let sx = current_assignments[layout.index_of(id_sx)];
                let sy = current_assignments[layout.index_of(id_sy)];
                let s = V::new(sx, sy) - c;

                let id_ex = circular_arc.end.id_x();
                let id_ey = circular_arc.end.id_y();
                let ex = current_assignments[layout.index_of(id_ex)];
                let ey = current_assignments[layout.index_of(id_ey)];
                let e = V::new(ex, ey) - c;

                let id_px = point.id_x();
                let id_py = point.id_y();
                let px = current_assignments[layout.index_of(id_px)];
                let py = current_assignments[layout.index_of(id_py)];
                let p = V::new(px, py) - c;

                let r = s.magnitude();
                let r_e = e.magnitude();
                let r_p = p.magnitude();
                if r < EPSILON || r_e < EPSILON || r_p < EPSILON {
                    *degenerate = true;
                    return;
                }

                let u_s = s * r.recip();
                let u_e = e * r_e.recip();
                let e_proj = e * (r / r_e);

                let (j_s, j_e, j_p) = match classify_point_arc_coincident(s, e_proj, p) {
                    PointArcCoincidentPart::Interior => {
                        // Point is inside arc
                        // Residual: f = p * (r/‖p‖ - 1)
                        // ∂f/∂s = u_p u_sᵀ
                        // ∂f/∂e = 0
                        // ∂f/∂p = (r/r_p - 1)I - (r/r_p) u_p u_pᵀ
                        let u_p = p * r_p.recip();
                        let r_over_rp = r / r_p;
                        (
                            [
                                [u_p.x * u_s.x, u_p.y * u_s.x],
                                [u_p.x * u_s.y, u_p.y * u_s.y],
                            ],
                            [[0.0, 0.0], [0.0, 0.0]],
                            [
                                [
                                    (r_over_rp - 1.0) - r_over_rp * u_p.x * u_p.x,
                                    -r_over_rp * u_p.y * u_p.x,
                                ],
                                [
                                    -r_over_rp * u_p.x * u_p.y,
                                    (r_over_rp - 1.0) - r_over_rp * u_p.y * u_p.y,
                                ],
                            ],
                        )
                    }
                    PointArcCoincidentPart::End => {
                        // Point is closer to arc end
                        // Residual: f = (r/r_e) * e - p
                        // ∂f/∂s = u_e u_sᵀ
                        // ∂f/∂e = (r/r_e)(I - u_e u_eᵀ)
                        // ∂f/∂p = -I
                        let r_over_re = r / r_e;
                        (
                            [
                                [u_e.x * u_s.x, u_e.y * u_s.x],
                                [u_e.x * u_s.y, u_e.y * u_s.y],
                            ],
                            [
                                [
                                    r_over_re * (1.0 - u_e.x * u_e.x),
                                    -r_over_re * u_e.y * u_e.x,
                                ],
                                [
                                    -r_over_re * u_e.x * u_e.y,
                                    r_over_re * (1.0 - u_e.y * u_e.y),
                                ],
                            ],
                            [[-1.0, 0.0], [0.0, -1.0]],
                        )
                    }
                    PointArcCoincidentPart::Start => {
                        // Point is closer to arc start
                        // Residual: f = s - p
                        // ∂f/∂s = I
                        // ∂f/∂e = 0
                        // ∂f/∂p = -I
                        (
                            [[1.0, 0.0], [0.0, 1.0]],
                            [[0.0, 0.0], [0.0, 0.0]],
                            [[-1.0, 0.0], [0.0, -1.0]],
                        )
                    }
                };

                // ∂f/∂c = -(∂f/∂s + ∂f/∂e + ∂f/∂p)
                let j_o = [
                    [
                        -(j_s[0][0] + j_e[0][0] + j_p[0][0]),
                        -(j_s[0][1] + j_e[0][1] + j_p[0][1]),
                    ],
                    [
                        -(j_s[1][0] + j_e[1][0] + j_p[1][0]),
                        -(j_s[1][1] + j_e[1][1] + j_p[1][1]),
                    ],
                ];

                row0.extend([
                    JacobianVar {
                        id: id_cx,
                        partial_derivative: j_o[0][0],
                    },
                    JacobianVar {
                        id: id_cy,
                        partial_derivative: j_o[1][0],
                    },
                    JacobianVar {
                        id: id_sx,
                        partial_derivative: j_s[0][0],
                    },
                    JacobianVar {
                        id: id_sy,
                        partial_derivative: j_s[1][0],
                    },
                    JacobianVar {
                        id: id_ex,
                        partial_derivative: j_e[0][0],
                    },
                    JacobianVar {
                        id: id_ey,
                        partial_derivative: j_e[1][0],
                    },
                    JacobianVar {
                        id: id_px,
                        partial_derivative: j_p[0][0],
                    },
                    JacobianVar {
                        id: id_py,
                        partial_derivative: j_p[1][0],
                    },
                ]);
                row1.extend([
                    JacobianVar {
                        id: id_cx,
                        partial_derivative: j_o[0][1],
                    },
                    JacobianVar {
                        id: id_cy,
                        partial_derivative: j_o[1][1],
                    },
                    JacobianVar {
                        id: id_sx,
                        partial_derivative: j_s[0][1],
                    },
                    JacobianVar {
                        id: id_sy,
                        partial_derivative: j_s[1][1],
                    },
                    JacobianVar {
                        id: id_ex,
                        partial_derivative: j_e[0][1],
                    },
                    JacobianVar {
                        id: id_ey,
                        partial_derivative: j_e[1][1],
                    },
                    JacobianVar {
                        id: id_px,
                        partial_derivative: j_p[0][1],
                    },
                    JacobianVar {
                        id: id_py,
                        partial_derivative: j_p[1][1],
                    },
                ]);
            }
            Constraint::ArcLength(circular_arc, d) => {
                // First, get all the variables.
                let id_cx = circular_arc.center.id_x();
                let id_cy = circular_arc.center.id_y();
                let id_ax = circular_arc.start.id_x();
                let id_ay = circular_arc.start.id_y();
                let id_bx = circular_arc.end.id_x();
                let id_by = circular_arc.end.id_y();
                let cx = current_assignments[layout.index_of(id_cx)];
                let cy = current_assignments[layout.index_of(id_cy)];
                let ax = current_assignments[layout.index_of(id_ax)];
                let ay = current_assignments[layout.index_of(id_ay)];
                let bx = current_assignments[layout.index_of(id_bx)];
                let by = current_assignments[layout.index_of(id_by)];
                let dx = ax - cx;
                let dy = ay - cy;
                let r2 = dx * dx + dy * dy;
                if r2 < EPSILON {
                    *degenerate = true;
                    return;
                }

                // Then calculate the partial derivatives.
                // Taken from SymPy, see ezpz-sympy.
                let r0dax = ((bx - cx) * ((ax - cx).powi(2) + (ay - cy).powi(2)).powf(7_f64 / 2.0)
                    - 2.0
                        * (ax - cx)
                        * ((ax - cx) * (bx - cx) + (ay - cy) * (by - cy))
                        * ((ax - cx).powi(2) + (ay - cy).powi(2)).powf(5_f64 / 2.0)
                    - d * (ax - cx)
                        * ((ax - cx).powi(2) + (ay - cy).powi(2)).powi(3)
                        * libm::sin(d * ((ax - cx).powi(2) + (ay - cy).powi(2)).sqrt().recip()))
                    / ((ax - cx).powi(2) + (ay - cy).powi(2)).powf(9_f64 / 2.0);
                let r0day = ((by - cy) * ((ax - cx).powi(2) + (ay - cy).powi(2)).powf(7_f64 / 2.0)
                    - 2.0
                        * (ay - cy)
                        * ((ax - cx) * (bx - cx) + (ay - cy) * (by - cy))
                        * ((ax - cx).powi(2) + (ay - cy).powi(2)).powf(5_f64 / 2.0)
                    - d * (ay - cy)
                        * ((ax - cx).powi(2) + (ay - cy).powi(2)).powi(3)
                        * libm::sin(d * ((ax - cx).powi(2) + (ay - cy).powi(2)).sqrt().recip()))
                    / ((ax - cx).powi(2) + (ay - cy).powi(2)).powf(9_f64 / 2.0);
                let r0dbx = (ax - cx) * ((ax - cx).powi(2) + (ay - cy).powi(2)).recip();
                let r0dby = (ay - cy) * ((ax - cx).powi(2) + (ay - cy).powi(2)).recip();
                let r0dcx = (((ax - cx).powi(2) + (ay - cy).powi(2)).powf(7_f64 / 2.0)
                    * (-ax - bx + 2.0 * cx)
                    + 2.0
                        * (ax - cx)
                        * ((ax - cx) * (bx - cx) + (ay - cy) * (by - cy))
                        * ((ax - cx).powi(2) + (ay - cy).powi(2)).powf(5_f64 / 2.0)
                    + d * (ax - cx)
                        * ((ax - cx).powi(2) + (ay - cy).powi(2)).powi(3)
                        * libm::sin(d * ((ax - cx).powi(2) + (ay - cy).powi(2)).sqrt().recip()))
                    / ((ax - cx).powi(2) + (ay - cy).powi(2)).powf(9_f64 / 2.0);
                let r0dcy = (((ax - cx).powi(2) + (ay - cy).powi(2)).powf(7_f64 / 2.0)
                    * (-ay - by + 2.0 * cy)
                    + 2.0
                        * (ay - cy)
                        * ((ax - cx) * (bx - cx) + (ay - cy) * (by - cy))
                        * ((ax - cx).powi(2) + (ay - cy).powi(2)).powf(5_f64 / 2.0)
                    + d * (ay - cy)
                        * ((ax - cx).powi(2) + (ay - cy).powi(2)).powi(3)
                        * libm::sin(d * ((ax - cx).powi(2) + (ay - cy).powi(2)).sqrt().recip()))
                    / ((ax - cx).powi(2) + (ay - cy).powi(2)).powf(9_f64 / 2.0);
                row0.extend([
                    JacobianVar {
                        id: id_ax,
                        partial_derivative: r0dax,
                    },
                    JacobianVar {
                        id: id_ay,
                        partial_derivative: r0day,
                    },
                    JacobianVar {
                        id: id_bx,
                        partial_derivative: r0dbx,
                    },
                    JacobianVar {
                        id: id_by,
                        partial_derivative: r0dby,
                    },
                    JacobianVar {
                        id: id_cx,
                        partial_derivative: r0dcx,
                    },
                    JacobianVar {
                        id: id_cy,
                        partial_derivative: r0dcy,
                    },
                ]);
                let r1dax = ((by - cy) * ((ax - cx).powi(2) + (ay - cy).powi(2)).powf(7_f64 / 2.0)
                    - 2.0
                        * (ax - cx)
                        * ((ax - cx) * (by - cy) - (ay - cy) * (bx - cx))
                        * ((ax - cx).powi(2) + (ay - cy).powi(2)).powf(5_f64 / 2.0)
                    + d * (ax - cx)
                        * ((ax - cx).powi(2) + (ay - cy).powi(2)).powi(3)
                        * libm::cos(d * ((ax - cx).powi(2) + (ay - cy).powi(2)).sqrt().recip()))
                    / ((ax - cx).powi(2) + (ay - cy).powi(2)).powf(9_f64 / 2.0);
                let r1day = ((-bx + cx)
                    * ((ax - cx).powi(2) + (ay - cy).powi(2)).powf(7_f64 / 2.0)
                    - 2.0
                        * (ay - cy)
                        * ((ax - cx) * (by - cy) - (ay - cy) * (bx - cx))
                        * ((ax - cx).powi(2) + (ay - cy).powi(2)).powf(5_f64 / 2.0)
                    + d * (ay - cy)
                        * ((ax - cx).powi(2) + (ay - cy).powi(2)).powi(3)
                        * libm::cos(d * ((ax - cx).powi(2) + (ay - cy).powi(2)).sqrt().recip()))
                    / ((ax - cx).powi(2) + (ay - cy).powi(2)).powf(9_f64 / 2.0);
                let r1dbx = (-ay + cy) * ((ax - cx).powi(2) + (ay - cy).powi(2)).recip();
                let r1dby = (ax - cx) * ((ax - cx).powi(2) + (ay - cy).powi(2)).recip();
                let r1dcx = ((ay - by) * ((ax - cx).powi(2) + (ay - cy).powi(2)).powf(7_f64 / 2.0)
                    + 2.0
                        * (ax - cx)
                        * ((ax - cx) * (by - cy) - (ay - cy) * (bx - cx))
                        * ((ax - cx).powi(2) + (ay - cy).powi(2)).powf(5_f64 / 2.0)
                    - d * (ax - cx)
                        * ((ax - cx).powi(2) + (ay - cy).powi(2)).powi(3)
                        * libm::cos(d * ((ax - cx).powi(2) + (ay - cy).powi(2)).sqrt().recip()))
                    / ((ax - cx).powi(2) + (ay - cy).powi(2)).powf(9_f64 / 2.0);
                let r1dcy = ((-ax + bx)
                    * ((ax - cx).powi(2) + (ay - cy).powi(2)).powf(7_f64 / 2.0)
                    + 2.0
                        * (ay - cy)
                        * ((ax - cx) * (by - cy) - (ay - cy) * (bx - cx))
                        * ((ax - cx).powi(2) + (ay - cy).powi(2)).powf(5_f64 / 2.0)
                    - d * (ay - cy)
                        * ((ax - cx).powi(2) + (ay - cy).powi(2)).powi(3)
                        * libm::cos(d * ((ax - cx).powi(2) + (ay - cy).powi(2)).sqrt().recip()))
                    / ((ax - cx).powi(2) + (ay - cy).powi(2)).powf(9_f64 / 2.0);
                row1.extend([
                    JacobianVar {
                        id: id_ax,
                        partial_derivative: r1dax,
                    },
                    JacobianVar {
                        id: id_ay,
                        partial_derivative: r1day,
                    },
                    JacobianVar {
                        id: id_bx,
                        partial_derivative: r1dbx,
                    },
                    JacobianVar {
                        id: id_by,
                        partial_derivative: r1dby,
                    },
                    JacobianVar {
                        id: id_cx,
                        partial_derivative: r1dcx,
                    },
                    JacobianVar {
                        id: id_cy,
                        partial_derivative: r1dcy,
                    },
                ]);
            }
            Constraint::ArcAngle(circular_arc, angle) => Constraint::LinesAtAngle(
                DatumLineSegment {
                    p0: circular_arc.center,
                    p1: circular_arc.start,
                },
                DatumLineSegment {
                    p0: circular_arc.center,
                    p1: circular_arc.end,
                },
                AngleKind::Other(*angle),
            )
            .jacobian_rows(layout, current_assignments, row0, row1, _row2, degenerate),
        }
    }

    /// Human-readable constraint name, useful for debugging.
    #[mutants::skip]
    pub fn constraint_kind(&self) -> &'static str {
        match self {
            Constraint::LineTangentToCircle(..) => "LineTangentToCircle",
            Constraint::CircleTangentToCircle(..) => "CircleTangentToCircle",
            Constraint::Distance(..) => "Distance",
            Constraint::DistanceVar(..) => "DistanceVar",
            Constraint::VerticalDistance(..) => "VerticalDistance",
            Constraint::HorizontalDistance(..) => "HorizontalDistance",
            Constraint::Vertical(..) => "Vertical",
            Constraint::Horizontal(..) => "Horizontal",
            Constraint::Fixed(..) => "Fixed",
            Constraint::LinesAtAngle(..) => "LinesAtAngle",
            Constraint::PointsCoincident(..) => "PointsCoincident",
            Constraint::CircleRadius(..) => "CircleRadius",
            Constraint::LinesEqualLength(..) => "LinesEqualLength",
            Constraint::ArcRadius(..) => "ArcRadius",
            Constraint::Arc(..) => "Arc",
            Constraint::Midpoint(..) => "Midpoint",
            Constraint::PointLineDistance(..) => "PointLineDistance",
            Constraint::VerticalPointLineDistance(_point, _line, _distance) => {
                "VerticalPointLineDistance"
            }
            Constraint::HorizontalPointLineDistance(_point, _line, _distance) => {
                "HorizontalPointLineDistance"
            }
            Constraint::Symmetric(..) => "Symmetric",
            Constraint::ScalarEqual(..) => "ScalarEqual",
            Constraint::PointArcCoincident(..) => "PointArcCoincident",
            Constraint::ArcLength(..) => "ArcLength",
            Constraint::ArcAngle(..) => "ArcAngle",
        }
    }
}

struct PointLineVars {
    px: f64,
    py: f64,
    p0x: f64,
    p0y: f64,
    p1x: f64,
    p1y: f64,
}

struct SymmetricPds {
    dpx: [f64; 2],
    dpy: [f64; 2],
    dqx: [f64; 2],
    dqy: [f64; 2],
    dax: [f64; 2],
    day: [f64; 2],
    dbx: [f64; 2],
    dby: [f64; 2],
}

struct SymmetricVars {
    px: f64,
    py: f64,
    qx: f64,
    qy: f64,
    ax: f64,
    ay: f64,
}

fn pds_from_symmetric(
    SymmetricVars {
        px,
        py,
        qx,
        qy,
        ax,
        ay,
    }: SymmetricVars,
) -> Option<SymmetricPds> {
    // See sympy notebook:
    // <https://colab.research.google.com/drive/17L_Lq-yTJOaLhDd2R0OtEe4Rwkr5RHsj#scrollTo=HpAraZ0OhKBW>
    // Common terms that appear in the derivatives a lot.
    let dx = px - qx;
    let dy = py - qy;
    let dx2 = dx * dx;
    let dy2 = dy * dy;
    let r = dx2 + dy2;
    let r2 = r.powi(2);
    // Avoid div-by-zero
    if r2 < EPSILON {
        return None;
    }

    let p_x = px;
    let p_y = py;
    let q_x = qx;
    let q_y = qy;
    let a_x = ax;
    let a_y = ay;

    let sx = a_x - p_x;
    let sy = a_y - p_y;
    let dot = sx * dx + sy * dy;

    let dpx = [
        (-4.0 * dx2 * dot
            + 2.0 * r2
            + 2.0 * r * (sx * dx + sy * dy + dx * (a_x - 2.0 * p_x + q_x)))
            / r2,
        dy * (-4.0 * dx * dot + 2.0 * r * (a_x - 2.0 * p_x + q_x)) / r2,
    ];
    let dpy = [
        dx * (-4.0 * dy * dot + 2.0 * r * (a_y - 2.0 * p_y + q_y)) / r2,
        (-4.0 * dy2 * dot
            + 2.0 * r2
            + 2.0 * r * (sx * dx + sy * dy + dy * (a_y - 2.0 * p_y + q_y)))
            / r2,
    ];
    let dqx = [
        (4.0 * dx2 * dot - (4.0 * sx * dx + 2.0 * sy * dy) * r) / r2,
        dy * (-2.0 * sx * r + 4.0 * dx * dot) / r2,
    ];
    let dqy = [
        dx * (-2.0 * sy * r + 4.0 * dy * dot) / r2,
        (4.0 * dy2 * dot - (2.0 * sx * dx + 4.0 * sy * dy) * r) / r2,
    ];
    let dax = [1.0 * (dx2 - dy2) / r, 2.0 * dx * dy / r];
    let day = [2.0 * dx * dy / r, 1.0 * (-dx2 + dy2) / r];
    let dbx = [-1.0, 0.0];
    let dby = [0.0, -1.0];

    Some(SymmetricPds {
        dpx,
        dpy,
        dqx,
        dqy,
        dax,
        day,
        dbx,
        dby,
    })
}

fn pds_for_point_line(
    point: DatumPoint,
    line: &DatumLineSegment,
    point_line_vars: PointLineVars,
) -> [JacobianVar; 6] {
    let PointLineVars {
        px,
        py,
        p0x,
        p0y,
        p1x,
        p1y,
    } = point_line_vars;

    // I used SymPy to get the derivatives. See this playground:
    // https://colab.research.google.com/drive/1zYHmggw6Juj8UFnxh-VKd8U9BG2Ul1gx?usp=sharing
    // This gets pretty hairy, I've tried to translate the math accurately. Please view the
    // playground above to get an intuition for what I'm doing.
    // The first two, d_px and d_py are relatively simple. They use the same denominator,
    // which represents the Euclidean distance between p0 and p1.
    let euclid_dist = libm::hypot(-p0x + p1x, p0y - p1y);
    let d_px = (p0y - p1y) / euclid_dist;
    let d_py = (-p0x + p1x) / euclid_dist;

    // The partial derivatives of the line's components (p0 and p1)
    // are trickier. There are some shared terms, e.g. the denominator of the LHS
    // fraction.
    let denom = ((-p0x + p1x).powi(2) + (p0y - p1y).powi(2)).powf(1.5);
    let d_p0x = {
        let lhs =
            ((-p0x + p1x) * (p0x * p1y - p0y * p1x + px * (p0y - p1y) + py * (-p0x + p1x))) / denom;
        let rhs = (p1y - py) / euclid_dist;
        lhs + rhs
    };

    let d_p0y = {
        let lhs =
            ((-p0y + p1y) * (p0x * p1y - p0y * p1x + px * (p0y - p1y) + py * (-p0x + p1x))) / denom;
        let rhs = (-p1x + px) / euclid_dist;
        lhs + rhs
    };

    let d_p1x = {
        let lhs =
            ((p0x - p1x) * (p0x * p1y - p0y * p1x + px * (p0y - p1y) + py * (-p0x + p1x))) / denom;
        let rhs = (-p0y + py) / euclid_dist;
        lhs + rhs
    };

    let d_p1y = {
        let lhs =
            ((p0y - p1y) * (p0x * p1y - p0y * p1x + px * (p0y - p1y) + py * (-p0x + p1x))) / denom;
        let rhs = (p0x - px) / euclid_dist;
        lhs + rhs
    };
    [
        JacobianVar {
            id: point.id_x(),
            partial_derivative: d_px,
        },
        JacobianVar {
            id: point.id_y(),
            partial_derivative: d_py,
        },
        JacobianVar {
            id: line.p0.id_x(),
            partial_derivative: d_p0x,
        },
        JacobianVar {
            id: line.p0.id_y(),
            partial_derivative: d_p0y,
        },
        JacobianVar {
            id: line.p1.id_x(),
            partial_derivative: d_p1x,
        },
        JacobianVar {
            id: line.p1.id_y(),
            partial_derivative: d_p1y,
        },
    ]
}

/// Partial derivatives for all 4 points that exist
/// in a line segment.
#[derive(Debug)]
struct PartialDerivatives4Points {
    x0: f64,
    y0: f64,
    x1: f64,
    y1: f64,
    x2: f64,
    y2: f64,
    x3: f64,
    y3: f64,
}

impl PartialDerivatives4Points {
    fn jvars(&self, line0: &DatumLineSegment, line1: &DatumLineSegment) -> [JacobianVar; 8] {
        [
            JacobianVar {
                id: line0.p0.id_x(),
                partial_derivative: self.x0,
            },
            JacobianVar {
                id: line0.p0.id_y(),
                partial_derivative: self.y0,
            },
            JacobianVar {
                id: line0.p1.id_x(),
                partial_derivative: self.x1,
            },
            JacobianVar {
                id: line0.p1.id_y(),
                partial_derivative: self.y1,
            },
            JacobianVar {
                id: line1.p0.id_x(),
                partial_derivative: self.x2,
            },
            JacobianVar {
                id: line1.p0.id_y(),
                partial_derivative: self.y2,
            },
            JacobianVar {
                id: line1.p1.id_x(),
                partial_derivative: self.x3,
            },
            JacobianVar {
                id: line1.p1.id_y(),
                partial_derivative: self.y3,
            },
        ]
    }
}

fn get_line_ends(
    current_assignments: &[f64],
    line0: &DatumLineSegment,
    line1: &DatumLineSegment,
    layout: &Layout,
) -> ((V, V), (V, V)) {
    let p0_x_l0 = current_assignments[layout.index_of(line0.p0.id_x())];
    let p0_y_l0 = current_assignments[layout.index_of(line0.p0.id_y())];
    let p1_x_l0 = current_assignments[layout.index_of(line0.p1.id_x())];
    let p1_y_l0 = current_assignments[layout.index_of(line0.p1.id_y())];
    let l0 = (V::new(p0_x_l0, p0_y_l0), V::new(p1_x_l0, p1_y_l0));
    let p0_x_l1 = current_assignments[layout.index_of(line1.p0.id_x())];
    let p0_y_l1 = current_assignments[layout.index_of(line1.p0.id_y())];
    let p1_x_l1 = current_assignments[layout.index_of(line1.p1.id_x())];
    let p1_y_l1 = current_assignments[layout.index_of(line1.p1.id_y())];
    let l1 = (V::new(p0_x_l1, p0_y_l1), V::new(p1_x_l1, p1_y_l1));
    (l0, l1)
}

/// Returns the active part of the residual for an arc centered on the origin given its start point,
/// its end point, and the point to constrain. Both start and end points are assumed to sit on the
/// arc.
fn classify_point_arc_coincident(s: V, e: V, p: V) -> PointArcCoincidentPart {
    // NOTE: This assumes the arc has CCW orientation from start to end
    let two_pi = 2.0 * PI;
    let a_sp = s.signed_angle(p).rem_euclid(two_pi);
    let a_se = s.signed_angle(e).rem_euclid(two_pi);

    if a_sp < a_se {
        PointArcCoincidentPart::Interior
    } else if (e - p).magnitude_squared() < (s - p).magnitude_squared() {
        PointArcCoincidentPart::End
    } else {
        PointArcCoincidentPart::Start
    }
}

/// If we represent the line in the form (Ax + By + C),
/// this returns (A, B, C).
fn equation_of_line(
    current_assignments: &[f64],
    line: &DatumLineSegment,
    layout: &Layout,
) -> (f64, f64, f64) {
    let px = current_assignments[layout.index_of(line.p0.id_x())];
    let py = current_assignments[layout.index_of(line.p0.id_y())];
    let qx = current_assignments[layout.index_of(line.p1.id_x())];
    let qy = current_assignments[layout.index_of(line.p1.id_y())];
    inner_equation_of_line(px, py, qx, qy)
}

/// Given two points on the line P and Q,
/// if we represent the line in the form (Ax + By + C),
/// this returns (A, B, C).
fn inner_equation_of_line(px: f64, py: f64, qx: f64, qy: f64) -> (f64, f64, f64) {
    // A = y1 - y2
    // B = x2 - x1
    // C = x1y2 - x2y1
    //
    // i.e.
    //
    // A = py - qy
    // B = qx - px
    // C = pxqy - qxpy
    let a = py - qy;
    let b = qx - px;
    let c = (px * qy) - (qx * py);
    (a, b, c)
}

fn rotation_for_angle_kind(angle_kind: AngleKind) -> Rotation2 {
    match angle_kind {
        AngleKind::Parallel => Rotation2::from_sincos(0.0, 1.0),
        AngleKind::Perpendicular => Rotation2::from_sincos(1.0, 0.0),
        AngleKind::Other(angle) => Rotation2::from_angle_radians(angle.to_radians()),
    }
}

#[cfg(test)]
fn wrap_angle_delta(delta: f64) -> f64 {
    if delta > -PI && delta <= PI {
        // If inside our interval, return unchanged.
        delta
    } else {
        // Wrap; see: https://stackoverflow.com/a/11181951
        let (sin, cos) = libm::sincos(delta);
        libm::atan2(sin, cos)
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::SQRT_2;

    use crate::{
        IdGenerator,
        datatypes::inputs::{
            DatumCircle, DatumCircularArc, DatumDistance, DatumLineSegment, DatumPoint,
        },
        tests::assert_nearly_eq,
    };

    use super::*;

    #[test]
    fn extend_dependent_variable_ids_reports_only_referenced_components() {
        let mut ids = IdGenerator::default();
        let p0 = DatumPoint::new(&mut ids);
        let p1 = DatumPoint::new(&mut ids);

        let horizontal = Constraint::HorizontalDistance(p0, p1, 10.0);
        let mut horizontal_ids = Vec::with_capacity(2);
        horizontal.extend_dependent_variable_ids(&mut horizontal_ids);
        assert_eq!(horizontal_ids, vec![p0.id_x(), p1.id_x()]);

        let vertical = Constraint::Vertical(DatumLineSegment::new(p0, p1));
        let mut vertical_ids = Vec::with_capacity(2);
        vertical.extend_dependent_variable_ids(&mut vertical_ids);
        assert_eq!(vertical_ids, vec![p0.id_x(), p1.id_x()]);
    }

    #[test]
    fn extend_associated_variable_ids_reports_all_datum_components() {
        let mut ids = IdGenerator::default();
        let p0 = DatumPoint::new(&mut ids);
        let p1 = DatumPoint::new(&mut ids);

        let horizontal = Constraint::HorizontalDistance(p0, p1, 10.0);
        let mut horizontal_ids = Vec::with_capacity(4);
        horizontal.extend_associated_variable_ids(&mut horizontal_ids);
        assert_eq!(
            horizontal_ids,
            vec![p0.id_x(), p0.id_y(), p1.id_x(), p1.id_y()]
        );

        let circle = DatumCircle {
            center: p0,
            radius: DatumDistance::new(ids.next_id()),
        };
        let mut circle_ids = Vec::with_capacity(3);
        Constraint::CircleRadius(circle, 5.0).extend_associated_variable_ids(&mut circle_ids);
        assert_eq!(
            circle_ids,
            vec![circle.center.id_x(), circle.center.id_y(), circle.radius.id]
        );
    }

    #[test]
    fn associated_and_dependent_variable_id_methods_accept_set_outputs() {
        let mut ids = IdGenerator::default();
        let arc = DatumCircularArc {
            center: DatumPoint::new(&mut ids),
            start: DatumPoint::new(&mut ids),
            end: DatumPoint::new(&mut ids),
        };
        let constraint = Constraint::ArcRadius(arc, 5.0);

        let mut out = std::collections::HashSet::new();
        constraint.extend_dependent_variable_ids(&mut out);
        constraint.extend_associated_variable_ids(&mut out);

        assert_eq!(out.len(), 6);
        assert!(out.contains(&arc.center.id_x()));
        assert!(out.contains(&arc.center.id_y()));
        assert!(out.contains(&arc.start.id_x()));
        assert!(out.contains(&arc.start.id_y()));
        assert!(out.contains(&arc.end.id_x()));
        assert!(out.contains(&arc.end.id_y()));
    }

    #[test]
    fn test_pds_of_symmetric() {
        // Arbitrarily chosen values.
        let input = SymmetricVars {
            px: 1.0,
            py: 2.0,
            qx: 0.5,
            qy: -1.0,
            ax: 3.0,
            ay: 4.0,
        };

        // I put these into the Python notebook where I defined the math, and got these answers.
        // https://colab.research.google.com/drive/17L_Lq-yTJOaLhDd2R0OtEe4Rwkr5RHsj#scrollTo=HpAraZ0OhKBW
        let expected = SymmetricPds {
            dpx: [3.59386413440468, 0.482103725346969],
            dpy: [-0.598977355734112, -0.0803506208911613],
            dqx: [-1.64791818845873, -0.806428049671293],
            dqy: [0.274653031409788, 0.134404674945215],
            dax: [-0.945945945945946, 0.324324324324324],
            day: [0.324324324324324, 0.945945945945946],
            dbx: [-1.0, 0.0],
            dby: [0.0, -1.0],
        };
        let actual = pds_from_symmetric(input).unwrap();

        assert_close(actual.dpx[0], expected.dpx[0]);
        assert_close(actual.dpx[1], expected.dpx[1]);
        assert_close(actual.dpy[0], expected.dpy[0]);
        assert_close(actual.dpy[1], expected.dpy[1]);
        assert_close(actual.dqx[0], expected.dqx[0]);
        assert_close(actual.dqx[1], expected.dqx[1]);
        assert_close(actual.dqy[0], expected.dqy[0]);
        assert_close(actual.dqy[1], expected.dqy[1]);
        assert_close(actual.dax[0], expected.dax[0]);
        assert_close(actual.dax[1], expected.dax[1]);
        assert_close(actual.day[0], expected.day[0]);
        assert_close(actual.day[1], expected.day[1]);
        assert_close(actual.dbx[0], expected.dbx[0]);
        assert_close(actual.dbx[1], expected.dbx[1]);
        assert_close(actual.dby[0], expected.dby[0]);
        assert_close(actual.dby[1], expected.dby[1]);
    }

    #[test]
    fn test_equation_of_line() {
        struct Test {
            name: &'static str,
            input: (f64, f64, f64, f64),
            expected: (f64, f64, f64),
        }

        let cases = [
            Test {
                name: "general",
                input: (1.0, 2.0, 3.0, 3.0),
                expected: (-1.0, 2.0, -3.0),
            },
            Test {
                name: "horizontal",
                input: (0.0, 0.0, 5.0, 0.0),
                expected: (0.0, 5.0, 0.0),
            },
            Test {
                name: "vertical",
                input: (2.0, 1.0, 2.0, 4.0),
                expected: (-3.0, 0.0, 6.0),
            },
            Test {
                name: "negative_slope",
                input: (-2.0, 3.0, 1.0, -1.0),
                expected: (4.0, 3.0, -1.0),
            },
        ];

        for case in cases {
            let (px, py, qx, qy) = case.input;
            let actual = inner_equation_of_line(px, py, qx, qy);
            let expected = case.expected;
            assert_eq!(
                actual, expected,
                "{}: got {actual:?} but wanted {expected:?}",
                case.name
            );
        }
    }

    #[test]
    fn test_geometry() {
        assert_nearly_eq(V::new(-1.0, 0.0).euclidean_distance(V::new(2.0, 4.0)), 5.0);
        assert_nearly_eq(V::new(1.0, 2.0).dot(V::new(4.0, -5.0)), 4.0 - 10.0);
        assert_nearly_eq(V::new(1.0, 0.0).cross_2d(V::new(0.0, 1.0)), 1.0);
        assert_nearly_eq(V::new(0.0, 1.0).cross_2d(V::new(1.0, 0.0)), -1.0);
        assert_nearly_eq(V::new(2.0, 2.0).cross_2d(V::new(4.0, 4.0)), 0.0);
        assert_nearly_eq(V::new(3.0, 4.0).cross_2d(V::new(5.0, 6.0)), -2.0);
    }

    #[test]
    fn test_wrap_angle_delta() {
        const EPS_WRAP: f64 = 1e-10;

        // Test angles already in range; should return unchanged.
        assert!(wrap_angle_delta(0.0).abs() < EPS_WRAP);
        assert!((wrap_angle_delta(PI / 2.0) - PI / 2.0).abs() < EPS_WRAP);
        assert!((wrap_angle_delta(-PI / 2.0) - (-PI / 2.0)).abs() < EPS_WRAP);
        assert!((wrap_angle_delta(PI) - PI).abs() < EPS_WRAP);
        assert!((wrap_angle_delta(-PI) - (-PI)).abs() < EPS_WRAP);

        // Test angles that need to be wrapped.
        assert!((wrap_angle_delta(3.0 * PI) - PI).abs() < EPS_WRAP); // 3pi wraps to pi.
        assert!((wrap_angle_delta(-3.0 * PI) - (-PI)).abs() < EPS_WRAP); // -3pi wraps to -pi.
        assert!((wrap_angle_delta(2.0 * PI) - 0.0).abs() < EPS_WRAP); // 2pi wraps to 0.
        assert!((wrap_angle_delta(-2.0 * PI) - 0.0).abs() < EPS_WRAP); // -2pi wraps to 0.

        // Test a value just across the -pi boundary.
        assert!((wrap_angle_delta(-PI - 1e-15) - PI).abs() < EPS_WRAP);
    }

    #[test]
    fn test_pds_for_point_line() {
        const EPS: f64 = 1e-9;

        struct Test {
            name: &'static str,
            point: DatumPoint,
            line: DatumLineSegment,
            vars: PointLineVars,
            expected: [(Id, f64); 6],
        }

        let tests = vec![
            Test {
                name: "horizontal_line",
                point: DatumPoint::new_xy(0, 1),
                line: DatumLineSegment::new(DatumPoint::new_xy(2, 3), DatumPoint::new_xy(4, 5)),
                vars: PointLineVars {
                    px: 0.0,
                    py: 1.0,
                    p0x: 0.0,
                    p0y: 0.0,
                    p1x: 1.0,
                    p1y: 0.0,
                },
                expected: [(0, 0.0), (1, 1.0), (2, 0.0), (3, -1.0), (4, 0.0), (5, 0.0)],
            },
            Test {
                name: "diagonal_line",
                point: DatumPoint::new_xy(100, 101),
                line: DatumLineSegment::new(
                    DatumPoint::new_xy(102, 103),
                    DatumPoint::new_xy(104, 105),
                ),
                vars: PointLineVars {
                    px: 2.0,
                    py: 0.0,
                    p0x: 0.0,
                    p0y: 0.0,
                    p1x: 2.0,
                    p1y: 2.0,
                },
                expected: [
                    (100, -SQRT_2 / 2.0),
                    (101, SQRT_2 / 2.0),
                    (102, SQRT_2 / 4.0),
                    (103, -SQRT_2 / 4.0),
                    (104, SQRT_2 / 4.0),
                    (105, -SQRT_2 / 4.0),
                ],
            },
            Test {
                name: "vertical_line",
                point: DatumPoint::new_xy(200, 201),
                line: DatumLineSegment::new(
                    DatumPoint::new_xy(202, 203),
                    DatumPoint::new_xy(204, 205),
                ),
                vars: PointLineVars {
                    px: 5.0,
                    py: 1.0,
                    p0x: 2.0,
                    p0y: -1.0,
                    p1x: 2.0,
                    p1y: 3.0,
                },
                expected: [
                    (200, -1.0),
                    (201, 0.0),
                    (202, 0.5),
                    (203, 0.0),
                    (204, 0.5),
                    (205, 0.0),
                ],
            },
        ];

        for test in tests {
            let actual = pds_for_point_line(test.point, &test.line, test.vars);

            for (idx, (expected_id, expected_pd)) in test.expected.iter().enumerate() {
                let jacobian_var = &actual[idx];
                assert_eq!(
                    jacobian_var.id, *expected_id,
                    "failed test {}: wrong ID in index {}",
                    test.name, idx
                );
                assert!(
                    (jacobian_var.partial_derivative - expected_pd).abs() < EPS,
                    "failed test {}: wrong derivative in index {} (expected {:.4}, got {:.4})",
                    test.name,
                    idx,
                    expected_pd,
                    jacobian_var.partial_derivative
                );
            }
        }
    }

    #[track_caller]
    fn assert_close(actual: f64, expected: f64) {
        let delta = actual - expected;
        assert!((delta).abs() <= 0.00001, "Delta is {}", delta);
    }
}
