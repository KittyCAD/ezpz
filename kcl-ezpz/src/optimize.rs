//! Optimize an external, public-facing problem specified by initial guesses and
//! constraints to an equivalent internal problem.

use std::collections::HashMap;

use ena::unify::{InPlaceUnificationTable, UnifyKey};

use crate::{
    Constraint, Id, NonLinearSystemError,
    constraints::ConstraintEntry,
    datatypes::{Circle, CircularArc, DatumDistance, DatumPoint, LineSegment},
};

/// A variable ID in the internal problem.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct InternalId(Id);

impl UnifyKey for InternalId {
    type Value = ();

    fn index(&self) -> u32 {
        self.0
    }

    fn from_index(index: u32) -> Self {
        InternalId(index)
    }

    fn tag() -> &'static str {
        "InternalId"
    }
}

/// A mapping from external problem variable IDs to internal problem variable
/// IDs.
#[derive(Debug)]
pub(super) struct ProblemMapping {
    /// Map from external variable ID to internal variable ID. The index in
    /// the vector is the external variable ID.
    map: Vec<InternalId>,
}

impl ProblemMapping {
    fn new(map: Vec<InternalId>) -> Self {
        Self { map }
    }

    /// Create a problem mapping from a set of constraints and all variable IDs.
    pub fn from_constraints(constraints: &[ConstraintEntry], num_external_variables: u32) -> Self {
        // Build the unification table where every key starts out separate.
        let mut vars_table = InPlaceUnificationTable::new();
        vars_table.reserve(num_external_variables as usize);
        for _ in all_external_variables(num_external_variables) {
            vars_table.new_key(());
        }

        // Unify variables according to equality constraints.
        for constraint in constraints.iter() {
            match &constraint.constraint {
                Constraint::PointsCoincident(p0, p1) => {
                    let (x0, x1) = (p0.id_x(), p1.id_x());
                    if x0 != x1 {
                        let a_id = InternalId(x0);
                        let b_id = InternalId(x1);
                        vars_table.union(a_id, b_id);
                    }
                    let (y0, y1) = (p0.id_y(), p1.id_y());
                    if y0 != y1 {
                        let a_id = InternalId(y0);
                        let b_id = InternalId(y1);
                        vars_table.union(a_id, b_id);
                    }
                }
                Constraint::LineTangentToCircle(_, _)
                | Constraint::Distance(_, _, _)
                | Constraint::Vertical(_)
                | Constraint::Horizontal(_)
                | Constraint::LinesAtAngle(_, _, _)
                | Constraint::Fixed(_, _)
                | Constraint::CircleRadius(_, _)
                | Constraint::LinesEqualLength(_, _)
                | Constraint::ArcRadius(_, _)
                | Constraint::Arc(_)
                | Constraint::Midpoint(_, _)
                | Constraint::PointLineDistance(_, _, _)
                | Constraint::Symmetric(_, _, _) => {}
            }
        }
        let external_to_internal = map_vars(&mut vars_table, num_external_variables);
        debug_assert_eq!(external_to_internal.len(), num_external_variables as usize);
        ProblemMapping::new(external_to_internal)
    }

    fn find_by_external(
        &self,
        external: Id,
        constraint_id: usize,
    ) -> Result<InternalId, NonLinearSystemError> {
        if let Some(internal) = self.map.get(external as usize) {
            Ok(*internal)
        } else {
            // A constraint references a variable ID that was never given an
            // initial guess.
            Err(NonLinearSystemError::MissingGuess {
                constraint_id,
                variable: external,
            })
        }
    }

    /// Convert an external constraint to an internal constraint.
    pub fn internal_constraint(
        &self,
        constraint: Constraint,
        constraint_id: usize,
    ) -> Result<Constraint, NonLinearSystemError> {
        match constraint {
            Constraint::LineTangentToCircle(line, circle) => Ok(Constraint::LineTangentToCircle(
                self.map_line_segment(line, constraint_id)?,
                self.map_circle(circle, constraint_id)?,
            )),
            Constraint::Distance(p0, p1, distance) => Ok(Constraint::Distance(
                self.map_datum_point(p0, constraint_id)?,
                self.map_datum_point(p1, constraint_id)?,
                distance,
            )),
            Constraint::Vertical(line) => Ok(Constraint::Vertical(
                self.map_line_segment(line, constraint_id)?,
            )),
            Constraint::Horizontal(line) => Ok(Constraint::Horizontal(
                self.map_line_segment(line, constraint_id)?,
            )),
            Constraint::LinesAtAngle(line0, line1, angle) => Ok(Constraint::LinesAtAngle(
                self.map_line_segment(line0, constraint_id)?,
                self.map_line_segment(line1, constraint_id)?,
                angle,
            )),
            Constraint::Fixed(id, scalar) => Ok(Constraint::Fixed(
                self.find_by_external(id, constraint_id)?.0,
                scalar,
            )),
            Constraint::PointsCoincident(datum_point0, datum_point1) => {
                Ok(Constraint::PointsCoincident(
                    self.map_datum_point(datum_point0, constraint_id)?,
                    self.map_datum_point(datum_point1, constraint_id)?,
                ))
            }
            Constraint::CircleRadius(circle, radius) => Ok(Constraint::CircleRadius(
                self.map_circle(circle, constraint_id)?,
                radius,
            )),
            Constraint::LinesEqualLength(line0, line1) => Ok(Constraint::LinesEqualLength(
                self.map_line_segment(line0, constraint_id)?,
                self.map_line_segment(line1, constraint_id)?,
            )),
            Constraint::ArcRadius(circular_arc, radius) => Ok(Constraint::ArcRadius(
                self.map_circular_arc(circular_arc, constraint_id)?,
                radius,
            )),
            Constraint::Arc(circular_arc) => Ok(Constraint::Arc(
                self.map_circular_arc(circular_arc, constraint_id)?,
            )),
            Constraint::Midpoint(line, point) => Ok(Constraint::Midpoint(
                self.map_line_segment(line, constraint_id)?,
                self.map_datum_point(point, constraint_id)?,
            )),
            Constraint::PointLineDistance(pt, line, distance) => Ok(Constraint::PointLineDistance(
                self.map_datum_point(pt, constraint_id)?,
                self.map_line_segment(line, constraint_id)?,
                distance,
            )),
            Constraint::Symmetric(line, p0, p1) => Ok(Constraint::Symmetric(
                self.map_line_segment(line, constraint_id)?,
                self.map_datum_point(p0, constraint_id)?,
                self.map_datum_point(p1, constraint_id)?,
            )),
        }
    }

    fn map_datum_point(
        &self,
        datum_point: DatumPoint,
        constraint_id: usize,
    ) -> Result<DatumPoint, NonLinearSystemError> {
        Ok(DatumPoint::new_xy(
            self.find_by_external(datum_point.id_x(), constraint_id)?.0,
            self.find_by_external(datum_point.id_y(), constraint_id)?.0,
        ))
    }

    fn map_line_segment(
        &self,
        line: LineSegment,
        constraint_id: usize,
    ) -> Result<LineSegment, NonLinearSystemError> {
        Ok(LineSegment::new(
            self.map_datum_point(line.p0, constraint_id)?,
            self.map_datum_point(line.p1, constraint_id)?,
        ))
    }

    fn map_datum_distance(
        &self,
        datum_distance: DatumDistance,
        constraint_id: usize,
    ) -> Result<DatumDistance, NonLinearSystemError> {
        Ok(DatumDistance::new(
            self.find_by_external(datum_distance.id, constraint_id)?.0,
        ))
    }

    fn map_circle(
        &self,
        circle: Circle,
        constraint_id: usize,
    ) -> Result<Circle, NonLinearSystemError> {
        Ok(Circle {
            center: self.map_datum_point(circle.center, constraint_id)?,
            radius: self.map_datum_distance(circle.radius, constraint_id)?,
        })
    }

    fn map_circular_arc(
        &self,
        circular_arc: CircularArc,
        constraint_id: usize,
    ) -> Result<CircularArc, NonLinearSystemError> {
        Ok(CircularArc {
            center: self.map_datum_point(circular_arc.center, constraint_id)?,
            a: self.map_datum_point(circular_arc.a, constraint_id)?,
            b: self.map_datum_point(circular_arc.b, constraint_id)?,
        })
    }

    /// Convert an internal solution to an external solution.
    pub fn external_solution(&self, internal_solution: &[f64]) -> Vec<f64> {
        self.map
            .iter()
            .copied()
            .map(|internal| *internal_solution.get(internal.0 as usize).unwrap())
            .collect()
    }
}

fn all_external_variables(num_external_variables: u32) -> impl Iterator<Item = Id> {
    0..num_external_variables
}

fn map_vars(
    table: &mut InPlaceUnificationTable<InternalId>,
    num_external_variables: u32,
) -> Vec<InternalId> {
    let mut next_internal_id: Id = 0;
    let mut external_to_internal = Vec::with_capacity(num_external_variables as usize);
    let mut root_to_internal = HashMap::new();
    for external_id in all_external_variables(num_external_variables) {
        // SAFETY: find() will panic if the key is not present.
        let root = table.find(InternalId(external_id));
        let internal_id = root_to_internal.entry(root).or_insert_with(|| {
            let id = next_internal_id;
            next_internal_id += 1;
            id
        });
        external_to_internal.push(InternalId(*internal_id));
    }
    external_to_internal
}
