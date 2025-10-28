use crate::{EPSILON, datatypes::*, id::Id, solver::Layout, vector::V};
use std::f64::consts::PI;

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

/// Each geometric constraint we support.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
#[non_exhaustive]
pub enum Constraint {
    /// This line must be tangent to the circle
    /// (i.e. touches its perimeter in exactly one place)
    /// Note this constraint is directional: making circle C
    /// tangent to PQ will produce a different solution to QP.
    LineTangentToCircle(LineSegment, Circle),
    /// These two points should be a given distance apart.
    Distance(DatumPoint, DatumPoint, f64),
    /// These two points have the same Y value.
    Vertical(LineSegment),
    /// These two points have the same X value.
    Horizontal(LineSegment),
    /// These lines meet at this angle.
    LinesAtAngle(LineSegment, LineSegment, AngleKind),
    /// Some scalar value is fixed.
    Fixed(Id, f64),
    /// These two points must coincide.
    PointsCoincident(DatumPoint, DatumPoint),
    /// Constraint radius of a circle
    CircleRadius(Circle, f64),
    /// These lines should be the same distance.
    LinesEqualLength(LineSegment, LineSegment),
    /// The arc should have the given radius.
    ArcRadius(CircularArc, f64),
    /// These 3 points should form an arc,
    /// i.e. `a` and `b` should be equidistant from `center`.
    Arc(CircularArc),
    /// The given point should be the midpoint along the given line.
    Midpoint(LineSegment, DatumPoint),
    /// The given point should be the given (perpendicular) distance away from the line.
    PointLineDistance(DatumPoint, LineSegment, f64),
    /// These two points should be symmetric across the given line.
    Symmetric(LineSegment, DatumPoint, DatumPoint),
}

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
pub enum AngleKind {
    Parallel,
    Perpendicular,
    Other(Angle),
}

/// Describes one value in one row of the Jacobian matrix.
#[derive(Clone, Copy)]
pub struct JacobianVar {
    /// Which variable are we talking about?
    /// Corresponds to one column in the row.
    pub id: Id,
    /// What value is its partial derivative?
    pub partial_derivative: f64,
}

impl std::fmt::Debug for JacobianVar {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "∂ col={} pd={:.3}", self.id, self.partial_derivative)
    }
}

impl Constraint {
    /// For each row of the Jacobian matrix, which variables are involved in them?
    pub fn nonzeroes(&self, row0: &mut Vec<Id>, row1: &mut Vec<Id>) {
        match self {
            Constraint::LineTangentToCircle(line, circle) => {
                row0.extend(line.all_variables());
                row0.extend(circle.all_variables());
            }
            Constraint::Distance(p0, p1, _dist) => {
                row0.extend(p0.all_variables());
                row0.extend(p1.all_variables());
            }
            Constraint::Vertical(line) => row0.extend([line.p0.id_x(), line.p1.id_x()]),
            Constraint::Horizontal(line) => row0.extend([line.p0.id_y(), line.p1.id_y()]),
            Constraint::LinesAtAngle(line0, line1, _angle) => {
                row0.extend(line0.all_variables());
                row0.extend(line1.all_variables());
            }
            Constraint::Fixed(id, _scalar) => row0.push(*id),
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
                // distance(center, a) and distance(center, b).
                let constraints = (
                    Constraint::Distance(arc.center, arc.a, *radius),
                    Constraint::Distance(arc.center, arc.b, *radius),
                );
                constraints.0.nonzeroes(row0, row1);
                constraints.1.nonzeroes(row1, row0);
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
            Constraint::Symmetric(line, a, b) => {
                // Equation: rej(A - P, Q - P) + rej(B - P, Q - P) = 0
                row0.extend(line.all_variables());
                row0.extend(a.all_variables());
                row0.extend(b.all_variables());
                row1.extend(line.all_variables());
                row1.extend(a.all_variables());
                row1.extend(b.all_variables());
            }
        }
    }

    /// Constrain these lines to be parallel.
    pub fn lines_parallel([l0, l1]: [LineSegment; 2]) -> Self {
        // TODO: Check if all points are unique.
        // Our math can't handle a common point just yet.
        Self::LinesAtAngle(l0, l1, AngleKind::Parallel)
    }

    /// Constrain these lines to be perpendicular.
    pub fn lines_perpendicular([l0, l1]: [LineSegment; 2]) -> Self {
        Self::LinesAtAngle(l0, l1, AngleKind::Perpendicular)
    }

    /// How close is this constraint to being satisfied?
    /// For performance reasons (avoiding allocations), this doesn't return a `Vec<f64>`,
    /// instead it takes one as a mutable argument and writes out all residuals to that.
    /// Most constraints have a residual measured as a single number (scalar),
    /// but some constraints have two residuals (e.g. one for the X axis and one for the Y axis).
    /// That's why there's two possible residuals to calculate (and therefore, two &mut residual to write into).
    pub fn residual(
        &self,
        layout: &Layout,
        current_assignments: &[f64],
        residual0: &mut f64,
        residual1: &mut f64,
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

                // Calculate the signed distance from the circle's center to the line
                // Formula: distance = (v × w) / |v|
                // where v is the line vector and w is the vector from p1 to the center.
                let v = p1 - p0;
                // let v = p0 - p1;
                let mag_v = v.magnitude();
                if mag_v < EPSILON {
                    // If line has no length, then the residual is 0, regardless of anything else.
                    *residual0 = 0.0;
                    *degenerate = true;
                    return;
                }
                let w = circle_center - p1;

                // Signed cross product (no absolute value).
                let cross_2d = v.cross_2d(&w);
                // Div-by-zero check:
                // already handled case where mag_v < EPSILON above and early-returned.
                let signed_distance_to_line = cross_2d / mag_v;
                let residual = signed_distance_to_line - radius;
                *residual0 = residual;
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
            Constraint::LinesAtAngle(line0, line1, expected_angle) => {
                // Get direction vectors for both lines.
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

                let v0 = l0.1 - l0.0;
                let v1 = l1.1 - l1.0;

                match expected_angle {
                    AngleKind::Parallel => {
                        *residual0 = v0.x * v1.y - v0.y * v1.x;
                    }
                    AngleKind::Perpendicular => {
                        *residual0 = v0.dot(&v1);
                    }
                    AngleKind::Other(expected_angle) => {
                        // Calculate magnitudes.
                        let mag0 = l0.0.euclidean_distance(l0.1);
                        let mag1 = l1.0.euclidean_distance(l1.1);

                        // Check for zero-length lines.
                        let is_invalid = (mag0 < EPSILON) || (mag1 < EPSILON);
                        if is_invalid {
                            *residual0 = 0.0;
                            *degenerate = true;
                            return;
                        }

                        // 2D cross product and dot product.
                        let cross_2d = v0.cross_2d(&v1);
                        let dot_product = v0.dot(&v1);

                        // Current angle using atan2.
                        let current_angle_radians = libm::atan2(cross_2d, dot_product);

                        // Compute angle difference and wrap to (-pi, pi].
                        let angle_residual = current_angle_radians - expected_angle.to_radians();
                        let wrapped_residual = wrap_angle_delta(angle_residual);
                        *residual0 = wrapped_residual;
                    }
                }
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
                // distance(center, a) and distance(center, b).
                let constraints = (
                    Constraint::Distance(arc.center, arc.a, *radius),
                    Constraint::Distance(arc.center, arc.b, *radius),
                );
                constraints.0.residual(
                    layout,
                    current_assignments,
                    residual0,
                    residual1,
                    degenerate,
                );
                constraints.1.residual(
                    layout,
                    current_assignments,
                    residual1,
                    residual0,
                    degenerate,
                );
            }
            Constraint::Arc(arc) => {
                let ax = current_assignments[layout.index_of(arc.a.id_x())];
                let ay = current_assignments[layout.index_of(arc.a.id_y())];
                let bx = current_assignments[layout.index_of(arc.b.id_x())];
                let by = current_assignments[layout.index_of(arc.b.id_y())];
                let cx = current_assignments[layout.index_of(arc.center.id_x())];
                let cy = current_assignments[layout.index_of(arc.center.id_y())];
                // For numerical stability and simpler derivatives, we compare the squared
                // distances. The residual is zero if the distances are equal.
                // R = distance(center, a)² - distance(center, b)²
                let dist0_sq = (ax - cx).powi(2) + (ay - cy).powi(2);
                let dist1_sq = (bx - cx).powi(2) + (by - cy).powi(2);

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
                let denominator = f64::hypot(a, b);
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
            Constraint::Symmetric(line, a, b) => {
                // Equation: rej(A - P, Q - P) = -rej(B - P, Q - P)
                //      i.e. rej(A - P, Q - P) + rej(B - P, Q - P) = 0

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

                let residual = (a - p).reject(q - p) + (b - p).reject(q - p);
                *residual0 = residual.x;
                *residual1 = residual.y;
            }
        }
    }

    /// How many equations does this constraint correspond to?
    /// Each equation is a residual function (a measure of error)
    pub fn residual_dim(&self) -> usize {
        match self {
            Constraint::LineTangentToCircle(..) => 1,
            Constraint::Distance(..) => 1,
            Constraint::Vertical(..) => 1,
            Constraint::Horizontal(..) => 1,
            Constraint::Fixed(..) => 1,
            Constraint::LinesAtAngle(..) => 1,
            Constraint::PointsCoincident(..) => 2,
            Constraint::CircleRadius(..) => 1,
            Constraint::LinesEqualLength(..) => 1,
            Constraint::ArcRadius(..) => 2,
            Constraint::Arc(..) => 1,
            Constraint::Midpoint(..) => 2,
            Constraint::PointLineDistance(..) => 1,
            Constraint::Symmetric(..) => 2,
        }
    }

    /// Used to construct part of a Jacobian matrix.
    /// For performance reasons (avoiding allocations), this doesn't return a
    /// `Vec<JacobianVar>` for each Jacobian row, instead takes the output rows as
    /// mutable arguments and writes out all nonzero variables for each row to
    /// one of them.
    pub fn jacobian_rows(
        &self,
        layout: &Layout,
        current_assignments: &[f64],
        row0: &mut Vec<JacobianVar>,
        row1: &mut Vec<JacobianVar>,
        degenerate: &mut bool,
    ) {
        match self {
            Constraint::LineTangentToCircle(line, circle) => {
                // Residual: R = ((x1-x0)*(yc-y0) - (y1-y0)*(xc-x0)) / sqrt((x1-x0)**2 + (y1-y0)**2) - r
                // ∂R/∂x0 = (-(x0 - x1)*((x0 - x1)*(y0 - yc) - (x0 - xc)*(y0 - y1)) + (y1 - yc)*((x0 - x1)**2 + (y0 - y1)**2))/((x0 - x1)**2 + (y0 - y1)**2)**(3/2)
                // ∂R/∂y0 = ((-x1 + xc)*((x0 - x1)**2 + (y0 - y1)**2) - (y0 - y1)*((x0 - x1)*(y0 - yc) - (x0 - xc)*(y0 - y1)))/((x0 - x1)**2 + (y0 - y1)**2)**(3/2)
                // ∂R/∂x1 = ((x0 - x1)*((x0 - x1)*(y0 - yc) - (x0 - xc)*(y0 - y1)) + (-y0 + yc)*((x0 - x1)**2 + (y0 - y1)**2))/((x0 - x1)**2 + (y0 - y1)**2)**(3/2)
                // ∂R/∂y1 = ((x0 - xc)*((x0 - x1)**2 + (y0 - y1)**2) + (y0 - y1)*((x0 - x1)*(y0 - yc) - (x0 - xc)*(y0 - y1)))/((x0 - x1)**2 + (y0 - y1)**2)**(3/2)
                // ∂R/∂xc = (y0 - y1)/sqrt((x0 - x1)**2 + (y0 - y1)**2)
                // ∂R/∂yc = (-x0 + x1)/sqrt((x0 - x1)**2 + (y0 - y1)**2)
                // ∂R/∂r = -1
                let x0 = current_assignments[layout.index_of(line.p0.id_x())];
                let y0 = current_assignments[layout.index_of(line.p0.id_y())];
                let p0 = V::new(x0, y0);
                let x1 = current_assignments[layout.index_of(line.p1.id_x())];
                let y1 = current_assignments[layout.index_of(line.p1.id_y())];
                let p1 = V::new(x1, y1);
                let xc = current_assignments[layout.index_of(circle.center.id_x())];
                let yc = current_assignments[layout.index_of(circle.center.id_y())];

                // Calculate common terms.
                let d = p0 - p1;
                let mag_v = d.magnitude();
                let mag_v_sq = d.magnitude_squared();
                let mag_v_cubed = mag_v.powi(3);

                if mag_v_sq < EPSILON {
                    *degenerate = true;
                    return;
                }

                // Cross product term that appears in the derivatives.
                let cross_term = d.x * (p0.y - yc) - (p0.x - xc) * d.y;

                let dr_dx0 = (-d.x * cross_term + (y1 - yc) * mag_v_sq) / mag_v_cubed;
                let dr_dy0 = ((-x1 + xc) * mag_v_sq - d.y * cross_term) / mag_v_cubed;
                let dr_dx1 = (d.x * cross_term + (-y0 + yc) * mag_v_sq) / mag_v_cubed;
                let dr_dy1 = ((x0 - xc) * mag_v_sq + d.y * cross_term) / mag_v_cubed;

                let dr_dxc = (y0 - y1) / mag_v;
                let dr_dyc = (-x0 + x1) / mag_v;

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
            Constraint::LinesAtAngle(line0, line1, expected_angle) => {
                // Residual: R = atan2(v1×v2, v1·v2) - α
                // ∂R/∂x1 = (y1 - y2)/(x1**2 - 2*x1*x2 + x2**2 + y1**2 - 2*y1*y2 + y2**2)
                // ∂R/∂y1 = (-x1 + x2)/(x1**2 - 2*x1*x2 + x2**2 + y1**2 - 2*y1*y2 + y2**2)
                // ∂R/∂x2 = (-y1 + y2)/(x1**2 - 2*x1*x2 + x2**2 + y1**2 - 2*y1*y2 + y2**2)
                // ∂R/∂y2 = (x1 - x2)/(x1**2 - 2*x1*x2 + x2**2 + y1**2 - 2*y1*y2 + y2**2)
                // ∂R/∂x3 = (-y3 + y4)/(x3**2 - 2*x3*x4 + x4**2 + y3**2 - 2*y3*y4 + y4**2)
                // ∂R/∂y3 = (x3 - x4)/(x3**2 - 2*x3*x4 + x4**2 + y3**2 - 2*y3*y4 + y4**2)
                // ∂R/∂x4 = (y3 - y4)/(x3**2 - 2*x3*x4 + x4**2 + y3**2 - 2*y3*y4 + y4**2)
                // ∂R/∂y4 = (-x3 + x4)/(x3**2 - 2*x3*x4 + x4**2 + y3**2 - 2*y3*y4 + y4**2)

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

                // Calculate partial derivatives
                let pds = match expected_angle {
                    AngleKind::Parallel => PartialDerivatives4Points {
                        // Residual: R = (x1-x0)*(y3-y2) - (y1-y0)*(x3-x2)
                        dr_dx0: y2 - y3,
                        dr_dy0: -x2 + x3,
                        dr_dx1: -y2 + y3,
                        dr_dy1: x2 - x3,
                        dr_dx2: -y0 + y1,
                        dr_dy2: x0 - x1,
                        dr_dx3: y0 - y1,
                        dr_dy3: -x0 + x1,
                    },
                    AngleKind::Perpendicular => PartialDerivatives4Points {
                        // Residual: R = (x1-x0)*(x3-x2) + (y1-y0)*(y3-y2)
                        dr_dx0: x2 - x3,
                        dr_dy0: y2 - y3,
                        dr_dx1: -x2 + x3,
                        dr_dy1: -y2 + y3,
                        dr_dx2: x0 - x1,
                        dr_dy2: y0 - y1,
                        dr_dx3: -x0 + x1,
                        dr_dy3: -y0 + y1,
                    },
                    AngleKind::Other(_expected_angle) => {
                        // Expected angle isn't used because its derivative is zero.
                        // Calculate magnitudes.
                        let mag0 = l0.0.euclidean_distance(l0.1);
                        let mag1 = l1.0.euclidean_distance(l1.1);

                        // Check for zero-length lines.
                        let is_invalid = (mag0 < EPSILON) || (mag1 < EPSILON);
                        if is_invalid {
                            // All zeroes
                            *degenerate = true;
                            return;
                        }

                        // Calculate derivatives.

                        // Note that our denominator terms for the partial derivatives above are
                        // the squared magnitudes of the vectors, i.e.:
                        // x1**2 - 2*x1*x2 + x2**2 + y1**2 - 2*y1*y2 + y2**2 == (x1 - x2)²  + (y1 - y2)²
                        // x3**2 - 2*x3*x4 + x4**2 + y3**2 - 2*y3*y4 + y4**2 == (x3 - x4)²  + (y3 - y4)²
                        let mag0_squared = mag0.powi(2);
                        let mag1_squared = mag1.powi(2);

                        PartialDerivatives4Points {
                            dr_dx0: (y0 - y1) / mag0_squared,
                            dr_dy0: (-x0 + x1) / mag0_squared,
                            dr_dx1: (-y0 + y1) / mag0_squared,
                            dr_dy1: (x0 - x1) / mag0_squared,
                            dr_dx2: (-y2 + y3) / mag1_squared,
                            dr_dy2: (x2 - x3) / mag1_squared,
                            dr_dx3: (y2 - y3) / mag1_squared,
                            dr_dy3: (-x2 + x3) / mag1_squared,
                        }
                    }
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
                    dr_dx0: (x0 - x1) / len0,
                    dr_dy0: (y0 - y1) / len0,
                    dr_dx1: (-x0 + x1) / len0,
                    dr_dy1: (-y0 + y1) / len0,
                    dr_dx2: (-x2 + x3) / len1,
                    dr_dy2: (-y2 + y3) / len1,
                    dr_dx3: (x2 - x3) / len1,
                    dr_dy3: (y2 - y3) / len1,
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
                })
            }
            Constraint::ArcRadius(arc, radius) => {
                // This is really just equivalent to 2 constraints,
                // distance(center, a) and distance(center, b).
                let constraints = (
                    Constraint::Distance(arc.center, arc.a, *radius),
                    Constraint::Distance(arc.center, arc.b, *radius),
                );
                constraints
                    .0
                    .jacobian_rows(layout, current_assignments, row0, row1, degenerate);
                constraints
                    .1
                    .jacobian_rows(layout, current_assignments, row1, row0, degenerate);
            }
            Constraint::Arc(arc) => {
                // Residual: R = (x1-xc)²+(y1-yc)² - (x2-xc)²-(y2-yc)²
                // The partial derivatives are:
                // ∂R/∂x1 = 2*(x1-xc)
                // ∂R/∂y1 = 2*(y1-yc)
                // ∂R/∂x2 = -2*(x2-xc)
                // ∂R/∂y2 = -2*(y2-yc)
                // ∂R/∂xc = 2*(x2-x1)
                // ∂R/∂yc = 2*(y2-y1)

                let ax = current_assignments[layout.index_of(arc.a.id_x())];
                let ay = current_assignments[layout.index_of(arc.a.id_y())];
                let bx = current_assignments[layout.index_of(arc.b.id_x())];
                let by = current_assignments[layout.index_of(arc.b.id_y())];
                let cx = current_assignments[layout.index_of(arc.center.id_x())];
                let cy = current_assignments[layout.index_of(arc.center.id_y())];

                // TODO: Handle degenerate case here

                // Calculate derivative values.
                let dx_a = (ax - cx) * 2.0;
                let dy_a = (ay - cy) * 2.0;
                let dx_b = (bx - cx) * -2.0;
                let dy_b = (by - cy) * -2.0;
                let dx_c = (bx - ax) * 2.0;
                let dy_c = (by - ay) * 2.0;
                row0.extend([
                    JacobianVar {
                        id: arc.a.id_x(),
                        partial_derivative: dx_a,
                    },
                    JacobianVar {
                        id: arc.a.id_y(),
                        partial_derivative: dy_a,
                    },
                    JacobianVar {
                        id: arc.b.id_x(),
                        partial_derivative: dx_b,
                    },
                    JacobianVar {
                        id: arc.b.id_y(),
                        partial_derivative: dy_b,
                    },
                    JacobianVar {
                        id: arc.center.id_x(),
                        partial_derivative: dx_c,
                    },
                    JacobianVar {
                        id: arc.center.id_y(),
                        partial_derivative: dy_c,
                    },
                ])
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
                    point,
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
                    bx: current_assignments[layout.index_of(b.id_x())],
                    by: current_assignments[layout.index_of(b.id_y())],
                };
                let Some(pds) = pds_from_symmetric(values) else {
                    *degenerate = true;
                    return;
                };

                row0.extend([
                    JacobianVar {
                        id: id_px,
                        partial_derivative: pds.dpx.0,
                    },
                    JacobianVar {
                        id: id_py,
                        partial_derivative: pds.dpy.0,
                    },
                    JacobianVar {
                        id: id_qx,
                        partial_derivative: pds.dqx.0,
                    },
                    JacobianVar {
                        id: id_qy,
                        partial_derivative: pds.dqy.0,
                    },
                    JacobianVar {
                        id: id_ax,
                        partial_derivative: pds.dax.0,
                    },
                    JacobianVar {
                        id: id_ay,
                        partial_derivative: pds.day.0,
                    },
                    JacobianVar {
                        id: id_bx,
                        partial_derivative: pds.dbx.0,
                    },
                    JacobianVar {
                        id: id_by,
                        partial_derivative: pds.dby.0,
                    },
                ]);

                row1.extend([
                    JacobianVar {
                        id: id_px,
                        partial_derivative: pds.dpx.1,
                    },
                    JacobianVar {
                        id: id_py,
                        partial_derivative: pds.dpy.1,
                    },
                    JacobianVar {
                        id: id_qx,
                        partial_derivative: pds.dqx.1,
                    },
                    JacobianVar {
                        id: id_qy,
                        partial_derivative: pds.dqy.1,
                    },
                    JacobianVar {
                        id: id_ax,
                        partial_derivative: pds.dax.1,
                    },
                    JacobianVar {
                        id: id_ay,
                        partial_derivative: pds.day.1,
                    },
                    JacobianVar {
                        id: id_bx,
                        partial_derivative: pds.dbx.1,
                    },
                    JacobianVar {
                        id: id_by,
                        partial_derivative: pds.dby.1,
                    },
                ]);
            }
        }
    }

    /// Human-readable constraint name, useful for debugging.
    pub fn constraint_kind(&self) -> &'static str {
        match self {
            Constraint::LineTangentToCircle(..) => "LineTangentToCircle",
            Constraint::Distance(..) => "Distance",
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
            Constraint::Symmetric(..) => "Symmetric",
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
    dpx: (f64, f64),
    dpy: (f64, f64),
    dqx: (f64, f64),
    dqy: (f64, f64),
    dax: (f64, f64),
    day: (f64, f64),
    dbx: (f64, f64),
    dby: (f64, f64),
}

struct SymmetricVars {
    px: f64,
    py: f64,
    qx: f64,
    qy: f64,
    ax: f64,
    ay: f64,
    bx: f64,
    by: f64,
}

fn pds_from_symmetric(
    SymmetricVars {
        px,
        py,
        qx,
        qy,
        ax,
        ay,
        bx,
        by,
    }: SymmetricVars,
) -> Option<SymmetricPds> {
    // See sympy notebook:
    // <https://colab.research.google.com/drive/12FUwqfpKzmWU2ZzNpdcT-0E1W3surJSt?usp=sharing#scrollTo=qCp9q2SznBJQ>
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
    let b_x = bx;
    let b_y = by;
    let s = (a_x - p_x) * dx + (a_y - p_y) * dy + (b_x - p_x) * dx + (b_y - p_y) * dy;
    let dpx = [
        (2.0 * dx2 * s
            - 2.0 * r2
            - r * ((a_x - p_x) * dx
                + (a_y - p_y) * dy
                + (b_x - p_x) * dx
                + (b_y - p_y) * dy
                + dx * (a_x - 2.0 * p_x + q_x)
                + dx * (b_x - 2.0 * p_x + q_x)))
            / r2,
        dy * (2.0 * dx * s + r * (-a_x - b_x + 4.0 * p_x - 2.0 * q_x)) / r2,
    ];
    let dpy = [
        dx * (2.0 * dy * s + r * (-a_y - b_y + 4.0 * p_y - 2.0 * q_y)) / r2,
        (2.0 * dy2 * s
            - 2.0 * r2
            - r * ((a_x - p_x) * dx
                + (a_y - p_y) * dy
                + (b_x - p_x) * dx
                + (b_y - p_y) * dy
                + dy * (a_y - 2.0 * p_y + q_y)
                + dy * (b_y - 2.0 * p_y + q_y)))
            / r2,
    ];
    let dqx = [
        (-2.0 * dx2 * s
            + r * (2.0 * (a_x - p_x) * dx
                + (a_y - p_y) * dy
                + 2.0 * (b_x - p_x) * dx
                + (b_y - p_y) * dy))
            / r2,
        dy * (-2.0 * dx * s + r * (a_x + b_x - 2.0 * p_x)) / r2,
    ];
    let dqy = [
        dx * (-2.0 * dy * s + r * (a_y + b_y - 2.0 * p_y)) / r2,
        (-2.0 * dy2 * s
            + r * ((a_x - p_x) * dx
                + 2.0 * (a_y - p_y) * dy
                + (b_x - p_x) * dx
                + 2.0 * (b_y - p_y) * dy))
            / r2,
    ];
    let dax = [dy2 / r, -dx * dy / r];
    let day = [-dx * dy / r, dx2 / r];
    let dbx = [dy2 / r, -dx * dy / r];
    let dby = [-dx * dy / r, dx2 / r];

    Some(SymmetricPds {
        dpx: (dpx[0], dpx[1]),
        dpy: (dpy[0], dpy[1]),
        dqx: (dqx[0], dqx[1]),
        dqy: (dqy[0], dqy[1]),
        dax: (dax[0], dax[1]),
        day: (day[0], day[1]),
        dbx: (dbx[0], dbx[1]),
        dby: (dby[0], dby[1]),
    })
}

fn pds_for_point_line(
    point: &DatumPoint,
    line: &LineSegment,
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
    let euclid_dist = f64::hypot(-p0x + p1x, p0y - p1y);
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

#[derive(Debug)]
struct PartialDerivatives4Points {
    dr_dx0: f64,
    dr_dy0: f64,
    dr_dx1: f64,
    dr_dy1: f64,
    dr_dx2: f64,
    dr_dy2: f64,
    dr_dx3: f64,
    dr_dy3: f64,
}

impl PartialDerivatives4Points {
    fn jvars(&self, line0: &LineSegment, line1: &LineSegment) -> [JacobianVar; 8] {
        [
            JacobianVar {
                id: line0.p0.id_x(),
                partial_derivative: self.dr_dx0,
            },
            JacobianVar {
                id: line0.p0.id_y(),
                partial_derivative: self.dr_dy0,
            },
            JacobianVar {
                id: line0.p1.id_x(),
                partial_derivative: self.dr_dx1,
            },
            JacobianVar {
                id: line0.p1.id_y(),
                partial_derivative: self.dr_dy1,
            },
            JacobianVar {
                id: line1.p0.id_x(),
                partial_derivative: self.dr_dx2,
            },
            JacobianVar {
                id: line1.p0.id_y(),
                partial_derivative: self.dr_dy2,
            },
            JacobianVar {
                id: line1.p1.id_x(),
                partial_derivative: self.dr_dx3,
            },
            JacobianVar {
                id: line1.p1.id_y(),
                partial_derivative: self.dr_dy3,
            },
        ]
    }
}

fn get_line_ends(
    current_assignments: &[f64],
    line0: &LineSegment,
    line1: &LineSegment,
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

/// If we represent the line in the form (Ax + By + C),
/// this returns (A, B, C).
fn equation_of_line(
    current_assignments: &[f64],
    line: &LineSegment,
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

#[cfg(test)]
mod tests {
    use std::f64::consts::SQRT_2;

    use super::*;

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
            bx: 5.0,
            by: 6.0,
        };

        // I put these into the Python notebook where I defined the math, and got these answers.
        let expected = SymmetricPds {
            dpx: (-4.41782322863404, -0.885317750182615),
            dpy: (0.736303871439007, 0.147552958363769),
            dqx: (2.47187728268809, 1.20964207450694),
            dqy: (-0.411979547114682, -0.201607012417824),
            dax: (0.972972972972973, -0.162162162162162),
            day: (-0.162162162162162, 0.0270270270270270),
            dbx: (0.972972972972973, -0.162162162162162),
            dby: (-0.162162162162162, 0.0270270270270270),
        };
        let actual = pds_from_symmetric(input).unwrap();

        assert_close(actual.dpx.0, expected.dpx.0);
        assert_close(actual.dpx.1, expected.dpx.1);
        assert_close(actual.dpy.0, expected.dpy.0);
        assert_close(actual.dpy.1, expected.dpy.1);
        assert_close(actual.dqx.0, expected.dqx.0);
        assert_close(actual.dqx.1, expected.dqx.1);
        assert_close(actual.dqy.0, expected.dqy.0);
        assert_close(actual.dqy.1, expected.dqy.1);
        assert_close(actual.dax.0, expected.dax.0);
        assert_close(actual.dax.1, expected.dax.1);
        assert_close(actual.day.0, expected.day.0);
        assert_close(actual.day.1, expected.day.1);
        assert_close(actual.dbx.0, expected.dbx.0);
        assert_close(actual.dbx.1, expected.dbx.1);
        assert_close(actual.dby.0, expected.dby.0);
        assert_close(actual.dby.1, expected.dby.1);
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
        assert_eq!(V::new(-1.0, 0.0).euclidean_distance(V::new(2.0, 4.0)), 5.0);
        assert_eq!(V::new(1.0, 2.0).dot(&V::new(4.0, -5.0)), 4.0 - 10.0);
        assert_eq!(V::new(1.0, 0.0).cross_2d(&V::new(0.0, 1.0)), 1.0);
        assert_eq!(V::new(0.0, 1.0).cross_2d(&V::new(1.0, 0.0)), -1.0);
        assert_eq!(V::new(2.0, 2.0).cross_2d(&V::new(4.0, 4.0)), 0.0);
        assert_eq!(V::new(3.0, 4.0).cross_2d(&V::new(5.0, 6.0)), -2.0);
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
            line: LineSegment,
            vars: PointLineVars,
            expected: [(Id, f64); 6],
        }

        let tests = vec![
            Test {
                name: "horizontal_line",
                point: DatumPoint::new_xy(0, 1),
                line: LineSegment::new(DatumPoint::new_xy(2, 3), DatumPoint::new_xy(4, 5)),
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
                line: LineSegment::new(DatumPoint::new_xy(102, 103), DatumPoint::new_xy(104, 105)),
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
                line: LineSegment::new(DatumPoint::new_xy(202, 203), DatumPoint::new_xy(204, 205)),
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
            let actual = pds_for_point_line(&test.point, &test.line, test.vars);

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
        if (delta).abs() > 0.00001 {
            panic!("Delta is {}", delta);
        }
    }
}
