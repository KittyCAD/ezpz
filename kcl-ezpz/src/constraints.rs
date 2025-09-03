use kittycad_modeling_cmds::shared::Angle;

use crate::{EPSILON, datatypes::*, id::Id, solver::Layout};

/// Each geometric constraint we support.
#[derive(Clone, Copy, Debug)]
pub enum Constraint {
    /// These two points should be a given distance apart.
    Distance(DatumPoint, DatumPoint, Distance),
    /// These two points have the same Y value.
    Vertical(LineSegment),
    /// These two points have the same X value.
    Horizontal(LineSegment),
    /// These lines meet at this angle.
    LinesAtAngle(LineSegment, LineSegment, AngleKind),
    /// Some scalar value is fixed.
    Fixed(Id, f64),
}

#[derive(Clone, Copy, Debug)]
pub enum AngleKind {
    Parallel,
    Perpendicular,
    Other(Angle),
}

/// Describes one value in one row of the Jacobian matrix.
#[derive(Clone, Copy, Debug)]
pub struct JacobianVar {
    /// Which variable are we talking about?
    /// Corresponds to one column in the row.
    pub id: Id,
    /// What value is its partial derivative?
    pub partial_derivative: f64,
}

/// One row of the Jacobian matrix.
/// I.e. describes a single equation in the system of equations being solved.
/// Specifically, it gives the partial derivatives of every variable in the equation.
/// If a variable isn't given, assume its partial derivative is 0.
#[derive(Default, Debug, Clone)]
pub struct JacobianRow {
    nonzero_columns: Vec<JacobianVar>,
}

/// Iterate over columns in the row.
impl IntoIterator for JacobianRow {
    type Item = JacobianVar;
    type IntoIter = std::vec::IntoIter<Self::Item>;

    /// Iterate over columns in the row.
    fn into_iter(self) -> Self::IntoIter {
        self.nonzero_columns.into_iter()
    }
}

impl Constraint {
    /// For each row of the Jacobian matrix, which variables are involved in them?
    pub fn nonzeroes(&self, row0: &mut Vec<Id>) {
        match self {
            Constraint::Distance(p0, p1, _dist) => {
                row0.extend([p0.id_x(), p0.id_y(), p1.id_x(), p1.id_y()])
            }
            Constraint::Vertical(line) => row0.extend([line.p0.id_x(), line.p1.id_x()]),
            Constraint::Horizontal(line) => row0.extend([line.p0.id_y(), line.p1.id_y()]),
            Constraint::LinesAtAngle(line0, line1, _angle) => row0.extend([
                line0.p0.id_x(),
                line0.p0.id_y(),
                line0.p1.id_x(),
                line0.p1.id_y(),
                line1.p0.id_x(),
                line1.p0.id_y(),
                line1.p1.id_x(),
                line1.p1.id_y(),
            ]),
            Constraint::Fixed(id, _scalar) => row0.push(*id),
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
    pub fn residual(&self, layout: &Layout, current_assignments: &[f64], output: &mut Vec<f64>) {
        match self {
            Constraint::Distance(p0, p1, expected_distance) => {
                let p0_x = current_assignments[layout.index_of(p0.id_x())];
                let p0_y = current_assignments[layout.index_of(p0.id_y())];
                let p1_x = current_assignments[layout.index_of(p1.id_x())];
                let p1_y = current_assignments[layout.index_of(p1.id_y())];
                let actual_distance = euclidean_distance((p0_x, p0_y), (p1_x, p1_y));
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
                let l0 = ((p0_x_l0, p0_y_l0), (p1_x_l0, p1_y_l0));
                let p0_x_l1 = current_assignments[layout.index_of(line1.p0.id_x())];
                let p0_y_l1 = current_assignments[layout.index_of(line1.p0.id_y())];
                let p1_x_l1 = current_assignments[layout.index_of(line1.p1.id_x())];
                let p1_y_l1 = current_assignments[layout.index_of(line1.p1.id_y())];
                let l1 = ((p0_x_l1, p0_y_l1), (p1_x_l1, p1_y_l1));

                let v0 = (p1_x_l0 - p0_x_l0, p1_y_l0 - p0_y_l0);
                let v1 = (p1_x_l1 - p0_x_l1, p1_y_l1 - p0_y_l1);

                match expected_angle {
                    AngleKind::Parallel => {
                        output.push(v0.0 * v1.1 - v0.1 * v1.0);
                    }
                    AngleKind::Perpendicular => {
                        let dot = v0.0 * v1.0 + v0.1 * v1.1;
                        output.push(dot);
                    }
                    AngleKind::Other(expected_angle) => {
                        // Calculate magnitudes.
                        let mag0 = euclidean_distance_line(l0);
                        let mag1 = euclidean_distance_line(l1);

                        // Check for zero-length lines.
                        let is_invalid = (mag0 < EPSILON) || (mag1 < EPSILON);
                        if is_invalid {
                            output.push(0.0);
                            return;
                        }

                        // 2D cross product and dot product.
                        let cross_2d = cross(v0, v1);
                        let dot_product = dot(v0, v1);

                        // Current angle using atan2.
                        let current_angle_radians = libm::atan2(cross_2d, dot_product);

                        // Compute angle difference.
                        let angle_residual = current_angle_radians - expected_angle.to_radians();
                        output.push(angle_residual);
                    }
                }
            }
        }
    }

    /// How many equations does this constraint correspond to?
    /// Each equation is a residual function (a measure of error)
    pub fn residual_dim(&self) -> usize {
        match self {
            Constraint::Distance(..) => 1,
            Constraint::Vertical(..) => 1,
            Constraint::Horizontal(..) => 1,
            Constraint::Fixed(..) => 1,
            Constraint::LinesAtAngle(..) => 1,
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
    ) {
        match self {
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

                let dist = euclidean_distance((x0, y0), (x1, y1));
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
                let l0 = ((x0, y0), (x1, y1));
                let x2 = current_assignments[layout.index_of(line1.p0.id_x())];
                let y2 = current_assignments[layout.index_of(line1.p0.id_y())];
                let x3 = current_assignments[layout.index_of(line1.p1.id_x())];
                let y3 = current_assignments[layout.index_of(line1.p1.id_y())];
                let l1 = ((x2, y2), (x3, y3));

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
                        let mag0 = euclidean_distance_line(l0);
                        let mag1 = euclidean_distance_line(l1);

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

                let jvars = [
                    JacobianVar {
                        id: line0.p0.id_x(),
                        partial_derivative: pds.dr_dx0,
                    },
                    JacobianVar {
                        id: line0.p0.id_y(),
                        partial_derivative: pds.dr_dy0,
                    },
                    JacobianVar {
                        id: line0.p1.id_x(),
                        partial_derivative: pds.dr_dx1,
                    },
                    JacobianVar {
                        id: line0.p1.id_y(),
                        partial_derivative: pds.dr_dy1,
                    },
                    JacobianVar {
                        id: line1.p0.id_x(),
                        partial_derivative: pds.dr_dx2,
                    },
                    JacobianVar {
                        id: line1.p0.id_y(),
                        partial_derivative: pds.dr_dy2,
                    },
                    JacobianVar {
                        id: line1.p1.id_x(),
                        partial_derivative: pds.dr_dx3,
                    },
                    JacobianVar {
                        id: line1.p1.id_y(),
                        partial_derivative: pds.dr_dy3,
                    },
                ];
                row0.extend(jvars.as_slice());
            }
        }
    }

    /// Human-readable constraint name, useful for debugging.
    pub fn constraint_kind(&self) -> &'static str {
        match self {
            Constraint::Distance(..) => "Distance",
            Constraint::Vertical(..) => "Vertical",
            Constraint::Horizontal(..) => "Horizontal",
            Constraint::Fixed(..) => "Fixed",
            Constraint::LinesAtAngle(..) => "LinesAtAngle",
        }
    }
}

/// Euclidean distance between two points.
pub(crate) fn euclidean_distance(p0: (f64, f64), p1: (f64, f64)) -> f64 {
    let dx = p0.0 - p1.0;
    let dy = p0.1 - p1.1;
    (dx.powf(2.0) + dy.powf(2.0)).sqrt()
}

/// Euclidean distance of a line.
fn euclidean_distance_line(line: ((f64, f64), (f64, f64))) -> f64 {
    euclidean_distance(line.0, line.1)
}

fn cross(p0: (f64, f64), p1: (f64, f64)) -> f64 {
    p0.0 * p1.1 - p0.1 * p1.0
}

fn dot(p0: (f64, f64), p1: (f64, f64)) -> f64 {
    p0.0 * p1.0 + p0.1 * p1.1
}

#[allow(dead_code)]
fn dot_line((p0, p1): ((f64, f64), (f64, f64))) -> f64 {
    dot(p0, p1)
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_geometry() {
        assert_eq!(euclidean_distance((-1.0, 0.0), (2.0, 4.0)), 5.0);
        assert_eq!(dot_line(((1.0, 2.0), (4.0, -5.0))), 4.0 - 10.0);
        assert_eq!(cross((1.0, 0.0), (0.0, 1.0)), 1.0);
        assert_eq!(cross((0.0, 1.0), (1.0, 0.0)), -1.0);
        assert_eq!(cross((2.0, 2.0), (4.0, 4.0)), 0.0);
        assert_eq!(cross((3.0, 4.0), (5.0, 6.0)), -2.0);
    }
}
