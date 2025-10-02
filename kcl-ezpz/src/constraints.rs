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
    pub fn residual(
        &self,
        layout: &Layout,
        current_assignments: &[f64],
        // Implicitly output0, i.e. residual for first row.
        output: &mut Vec<f64>,
        output1: &mut Vec<f64>,
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
                    output.push(0.0);
                    return;
                }
                let w = circle_center - p1;

                // Signed cross product (no absolute value).
                let cross_2d = v.cross_2d(&w);
                // Div-by-zero check:
                // already handled case where mag_v < EPSILON above and early-returned.
                let signed_distance_to_line = cross_2d / mag_v;
                let residual = signed_distance_to_line - radius;
                output.push(residual);
            }
            Constraint::Distance(p0, p1, expected_distance) => {
                let p0_x = current_assignments[layout.index_of(p0.id_x())];
                let p0_y = current_assignments[layout.index_of(p0.id_y())];
                let p0 = V::new(p0_x, p0_y);
                let p1_x = current_assignments[layout.index_of(p1.id_x())];
                let p1_y = current_assignments[layout.index_of(p1.id_y())];
                let p1 = V::new(p1_x, p1_y);
                let actual_distance = p0.euclidean_distance(p1);
                output.push(actual_distance - expected_distance);
            }
            Constraint::Vertical(line) => {
                let p0_x = current_assignments[layout.index_of(line.p0.id_x())];
                let p1_x = current_assignments[layout.index_of(line.p1.id_x())];
                output.push(p0_x - p1_x);
            }
            Constraint::Horizontal(line) => {
                let p0_y = current_assignments[layout.index_of(line.p0.id_y())];
                let p1_y = current_assignments[layout.index_of(line.p1.id_y())];
                output.push(p0_y - p1_y);
            }
            Constraint::Fixed(id, expected) => {
                let actual = current_assignments[layout.index_of(*id)];
                output.push(actual - expected);
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
                        output.push(v0.x * v1.y - v0.y * v1.x);
                    }
                    AngleKind::Perpendicular => {
                        output.push(v0.dot(&v1));
                    }
                    AngleKind::Other(expected_angle) => {
                        // Calculate magnitudes.
                        let mag0 = l0.0.euclidean_distance(l0.1);
                        let mag1 = l1.0.euclidean_distance(l1.1);

                        // Check for zero-length lines.
                        let is_invalid = (mag0 < EPSILON) || (mag1 < EPSILON);
                        if is_invalid {
                            output.push(0.0);
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
                        output.push(wrapped_residual);
                    }
                }
            }
            Constraint::PointsCoincident(p0, p1) => {
                let p0_x = current_assignments[layout.index_of(p0.id_x())];
                let p0_y = current_assignments[layout.index_of(p0.id_y())];
                let p1_x = current_assignments[layout.index_of(p1.id_x())];
                let p1_y = current_assignments[layout.index_of(p1.id_y())];
                output.push(p0_x - p1_x);
                output1.push(p0_y - p1_y);
            }
            Constraint::CircleRadius(circle, expected_radius) => {
                let actual_radius = current_assignments[layout.index_of(circle.radius.id)];
                output.push(actual_radius - *expected_radius);
            }
            Constraint::LinesEqualLength(line0, line1) => {
                let (l0, l1) = get_line_ends(current_assignments, line0, line1, layout);
                let len0 = l0.0.euclidean_distance(l0.1);
                let len1 = l1.0.euclidean_distance(l1.1);
                output.push(len0 - len1);
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
                    .residual(layout, current_assignments, output, output1);
                constraints
                    .1
                    .residual(layout, current_assignments, output1, output);
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

                output.push(dist0_sq - dist1_sq);
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
                    .jacobian_rows(layout, current_assignments, row0, row1);
                constraints
                    .1
                    .jacobian_rows(layout, current_assignments, row1, row0);
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

                // a = 1, b = 2

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
        }
    }
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

#[cfg(test)]
mod tests {
    use super::*;

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
}
