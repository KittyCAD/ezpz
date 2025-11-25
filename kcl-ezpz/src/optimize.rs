//! Optimize an external, public-facing problem specified by initial guesses and
//! constraints to an equivalent internal problem.
use std::collections::HashMap;

use crate::{Constraint, Id, constraints::ConstraintEntry, datatypes::DatumPoint};

/// A variable ID in the internal problem.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct InternalId(Id);

/// A mapping from external problem variable IDs to internal problem variable
/// IDs.
#[derive(Debug)]
pub(super) struct ProblemMapping {
    /// Mapping from public-facing external problem variable ID to internal ID.
    /// It's a sparse mapping. If the ID isn't present, it maps to itself.
    map: HashMap<Id, InternalId>,
    /// The number of variables in the external problem. We assume the IDs are
    /// the range `0..num_external_variables`.
    num_external_variables: u32,
}

impl ProblemMapping {
    fn new(map: HashMap<Id, InternalId>, num_external_variables: u32) -> Self {
        Self {
            map,
            num_external_variables,
        }
    }

    /// Create a problem mapping from a set of constraints and all variable IDs.
    pub fn from_constraints(constraints: &[ConstraintEntry], num_external_variables: u32) -> Self {
        let mut vars_map: HashMap<Id, InternalId> = HashMap::new();
        for constraint in constraints.iter() {
            match &constraint.constraint {
                Constraint::PointsCoincident(p0, p1) => {
                    // Create an alias from the higher ID to the lower ID.
                    let (a, b) = (p0.id_x().min(p1.id_x()), p0.id_x().max(p1.id_x()));
                    if a != b {
                        vars_map.insert(b, InternalId(a));
                    }
                    let (a, b) = (p0.id_y().min(p1.id_y()), p0.id_y().max(p1.id_y()));
                    if a != b {
                        vars_map.insert(b, InternalId(a));
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
        ProblemMapping::new(vars_map, num_external_variables)
    }

    /// Convert an external constraint to an internal constraint.
    pub fn to_internal_constraint(&self, constraint: Constraint) -> Constraint {
        match constraint {
            Constraint::LineTangentToCircle(_, _) => constraint,
            Constraint::Distance(_, _, _) => constraint,
            Constraint::Vertical(_) => constraint,
            Constraint::Horizontal(_) => constraint,
            Constraint::LinesAtAngle(_, _, _) => constraint,
            Constraint::Fixed(_, _) => constraint,
            Constraint::PointsCoincident(datum_point0, datum_point1) => {
                Constraint::PointsCoincident(
                    self.map_datum_point(datum_point0),
                    self.map_datum_point(datum_point1),
                )
            }
            Constraint::CircleRadius(_, _) => constraint,
            Constraint::LinesEqualLength(_, _) => constraint,
            Constraint::ArcRadius(_, _) => constraint,
            Constraint::Arc(_) => constraint,
            Constraint::Midpoint(_, _) => constraint,
            Constraint::PointLineDistance(_, _, _) => constraint,
            Constraint::Symmetric(_, _, _) => constraint,
        }
    }

    fn map_datum_point(&self, datum_point: DatumPoint) -> DatumPoint {
        DatumPoint::new_xy(
            match self.map.get(&datum_point.id_x()) {
                Some(id) => id.0,
                None => datum_point.id_x(),
            },
            match self.map.get(&datum_point.id_y()) {
                Some(id) => id.0,
                None => datum_point.id_y(),
            },
        )
    }

    /// Convert an internal solution to an external solution.
    pub fn to_external_solution(&self, internal_solution: &[f64]) -> Vec<f64> {
        (0..self.num_external_variables)
            .map(|external_idx| {
                let target = self
                    .map
                    .get(&external_idx)
                    .copied()
                    .unwrap_or(InternalId(external_idx));
                internal_solution[target.0 as usize]
            })
            .collect()
    }
}
