//! Optimize an external, public-facing problem specified by initial guesses and
//! constraints to an equivalent internal problem.

use std::collections::HashMap;

use ena::unify::{InPlaceUnificationTable, NoError, UnifyKey, UnifyValue};

use crate::{
    Constraint, ConstraintRequest, Error, FailureOutcome, Id, NonLinearSystemError, Warning,
    constraints::ConstraintEntry,
    datatypes::{Circle, CircularArc, DatumDistance, DatumPoint, LineSegment},
};

/// A variable ID in the internal problem.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct InternalId(Id);

/// A variable ID in the external problem.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct ExternalId(Id);

impl UnifyKey for ExternalId {
    type Value = InitialValue;

    fn index(&self) -> u32 {
        self.0
    }

    fn from_index(index: u32) -> Self {
        ExternalId(index)
    }

    fn tag() -> &'static str {
        "ExternalId"
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct InitialValue(f64);

impl UnifyValue for InitialValue {
    type Error = NoError;

    fn unify_values(value1: &Self, _value2: &Self) -> Result<Self, Self::Error> {
        // For initial values, we can pick one of the values. We arbitrarily
        // choose to keep the first one.
        Ok(*value1)
    }
}

/// A mapping from external problem to optimized internal problem.
#[derive(Debug)]
pub(super) struct ProblemMapping {
    /// Map from external variable ID to internal variable ID. The index in
    /// the vector is the external variable ID.
    map: Vec<InternalId>,
    /// Initial values for the internal variables.
    internal_initial_values: Vec<f64>,
    /// Since `ConstraintEntry`s borrow their `Constraint`s, we need to
    /// materialize the internal constraints and store them somewhere. The usize
    /// is the constraint ID.
    internal_constraints: Vec<(usize, ConstraintRequest)>,
}

impl ProblemMapping {
    fn new(
        map: Vec<InternalId>,
        internal_initial_values: Vec<f64>,
        internal_constraints: Vec<(usize, ConstraintRequest)>,
    ) -> Self {
        Self {
            map,
            internal_initial_values,
            internal_constraints,
        }
    }

    /// Create a problem mapping from a set of constraints and all variable IDs.
    pub fn from_constraints(
        initial_values: &[f64],
        constraints: &[ConstraintEntry],
        warnings: &[Warning],
    ) -> Result<Self, FailureOutcome> {
        // Build the unification table where every key starts out separate.
        let num_external_variables: u32 = initial_values.len() as u32;
        let mut vars_table = InPlaceUnificationTable::new();
        vars_table.reserve(initial_values.len());
        for value in initial_values {
            vars_table.new_key(InitialValue(*value));
        }

        // Unify variables according to equality constraints.
        for constraint in constraints.iter() {
            match &constraint.constraint {
                Constraint::PointsCoincident(p0, p1) => {
                    let (x0, x1) = (p0.id_x(), p1.id_x());
                    if x0 != x1 {
                        let a = ExternalId(x0);
                        let b = ExternalId(x1);
                        vars_table.union(a, b);
                    }
                    let (y0, y1) = (p0.id_y(), p1.id_y());
                    if y0 != y1 {
                        let a = ExternalId(y0);
                        let b = ExternalId(y1);
                        vars_table.union(a, b);
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
        // Build the mapping from external variable IDs to internal variable
        // IDs.
        let (external_to_internal, internal_initial_values) =
            map_vars(&mut vars_table, num_external_variables);
        debug_assert_eq!(external_to_internal.len(), initial_values.len());

        // Use the mapping to convert the constraints to the internal problem.
        let transformer = ConstraintTransformer {
            map: external_to_internal,
        };
        transformer.into_problem_mapping(
            internal_initial_values,
            constraints,
            warnings,
            num_external_variables as usize,
            constraints.len(),
        )
    }

    pub fn constraints(&self) -> &[(usize, ConstraintRequest)] {
        &self.internal_constraints
    }

    pub fn internal_initial_values(&self) -> &[f64] {
        &self.internal_initial_values
    }

    pub fn internal_variables(&self) -> Vec<Id> {
        (0..self.internal_initial_values.len())
            .map(|i| i as u32)
            .collect()
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

/// Struct to convert external constraints to internal constraints.
#[derive(Debug)]
struct ConstraintTransformer {
    /// Map from external variable ID to internal variable ID. The index in
    /// the vector is the external variable ID.
    map: Vec<InternalId>,
}

impl ConstraintTransformer {
    pub fn into_problem_mapping(
        self,
        internal_initial_values: Vec<f64>,
        external_constraints: &[ConstraintEntry],
        warnings: &[Warning],
        num_external_vars: usize,
        num_constraints: usize,
    ) -> Result<ProblemMapping, FailureOutcome> {
        let internal_constraints = external_constraints
            .iter()
            .map(|c| {
                let internal_constraint =
                    self.internal_constraint(*c.constraint, c.id)
                        .map_err(|err| FailureOutcome {
                            error: Error::NonLinearSystemError(err),
                            warnings: warnings.to_vec(),
                            num_vars: num_external_vars,
                            num_eqs: num_constraints,
                        })?;
                Ok(internal_constraint.map(|internal| {
                    (
                        c.id,
                        ConstraintRequest {
                            constraint: internal,
                            priority: c.priority,
                        },
                    )
                }))
            })
            .filter_map(Result::transpose)
            .collect::<Result<Vec<_>, _>>()?;

        Ok(ProblemMapping::new(
            self.map,
            internal_initial_values,
            internal_constraints,
        ))
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
    ) -> Result<Option<Constraint>, NonLinearSystemError> {
        match constraint {
            Constraint::LineTangentToCircle(line, circle) => {
                Ok(Some(Constraint::LineTangentToCircle(
                    self.map_line_segment(line, constraint_id)?,
                    self.map_circle(circle, constraint_id)?,
                )))
            }
            Constraint::Distance(p0, p1, distance) => Ok(Some(Constraint::Distance(
                self.map_datum_point(p0, constraint_id)?,
                self.map_datum_point(p1, constraint_id)?,
                distance,
            ))),
            Constraint::Vertical(line) => Ok(Some(Constraint::Vertical(
                self.map_line_segment(line, constraint_id)?,
            ))),
            Constraint::Horizontal(line) => Ok(Some(Constraint::Horizontal(
                self.map_line_segment(line, constraint_id)?,
            ))),
            Constraint::LinesAtAngle(line0, line1, angle) => Ok(Some(Constraint::LinesAtAngle(
                self.map_line_segment(line0, constraint_id)?,
                self.map_line_segment(line1, constraint_id)?,
                angle,
            ))),
            Constraint::Fixed(id, scalar) => Ok(Some(Constraint::Fixed(
                self.find_by_external(id, constraint_id)?.0,
                scalar,
            ))),
            // Point variables are unified, so the constraint isn't needed.
            Constraint::PointsCoincident(_, _) => Ok(None),
            Constraint::CircleRadius(circle, radius) => Ok(Some(Constraint::CircleRadius(
                self.map_circle(circle, constraint_id)?,
                radius,
            ))),
            Constraint::LinesEqualLength(line0, line1) => Ok(Some(Constraint::LinesEqualLength(
                self.map_line_segment(line0, constraint_id)?,
                self.map_line_segment(line1, constraint_id)?,
            ))),
            Constraint::ArcRadius(circular_arc, radius) => Ok(Some(Constraint::ArcRadius(
                self.map_circular_arc(circular_arc, constraint_id)?,
                radius,
            ))),
            Constraint::Arc(circular_arc) => Ok(Some(Constraint::Arc(
                self.map_circular_arc(circular_arc, constraint_id)?,
            ))),
            Constraint::Midpoint(line, point) => Ok(Some(Constraint::Midpoint(
                self.map_line_segment(line, constraint_id)?,
                self.map_datum_point(point, constraint_id)?,
            ))),
            Constraint::PointLineDistance(pt, line, distance) => {
                Ok(Some(Constraint::PointLineDistance(
                    self.map_datum_point(pt, constraint_id)?,
                    self.map_line_segment(line, constraint_id)?,
                    distance,
                )))
            }
            Constraint::Symmetric(line, p0, p1) => Ok(Some(Constraint::Symmetric(
                self.map_line_segment(line, constraint_id)?,
                self.map_datum_point(p0, constraint_id)?,
                self.map_datum_point(p1, constraint_id)?,
            ))),
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
}

fn all_external_variables(num_external_variables: u32) -> impl Iterator<Item = Id> {
    0..num_external_variables
}

/// Compact only the roots of the external variables into a contiguous range of
/// internal variable IDs that can be used in a solve. Returns a mapping from
/// external variable ID to internal variable ID, and the initial values of the
/// internal variables.
fn map_vars(
    table: &mut InPlaceUnificationTable<ExternalId>,
    num_external_variables: u32,
) -> (Vec<InternalId>, Vec<f64>) {
    let mut next_internal_id: Id = 0;
    let mut external_to_internal = Vec::with_capacity(num_external_variables as usize);
    let mut root_to_internal = HashMap::new();
    let mut internal_initial_values = Vec::new();
    for external_id in all_external_variables(num_external_variables) {
        // SAFETY: find() will panic if the key is not present.
        let root = table.find(ExternalId(external_id));
        let internal_id = root_to_internal.entry(root).or_insert_with(|| {
            internal_initial_values.push(table.probe_value(root).0);

            let id = InternalId(next_internal_id);
            next_internal_id += 1;
            id
        });
        external_to_internal.push(*internal_id);
    }
    debug_assert_eq!(next_internal_id as usize, internal_initial_values.len());

    (external_to_internal, internal_initial_values)
}
