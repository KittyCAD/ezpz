use std::collections::HashMap;

use indexmap::IndexMap;

use crate::Config;
use crate::Constraint;
use crate::Error;
use crate::FailureOutcome;
use crate::IdGenerator;
use crate::SolveOutcome;
use crate::Warning;
use crate::constraints::AngleKind;
use crate::datatypes;
use crate::datatypes::CircularArc;
use crate::datatypes::DatumDistance;
use crate::datatypes::DatumPoint;
use crate::datatypes::LineSegment;
use crate::textual::Arc;
use crate::textual::geometry_variables::DoneState;
use crate::textual::geometry_variables::GeometryVariables;
use crate::textual::geometry_variables::PointsState;
use crate::textual::geometry_variables::VARS_PER_ARC;
use crate::textual::instruction::*;
use crate::textual::{Circle, Component, Label, Point};

use super::Instruction;
use super::Problem;

impl Problem {
    pub fn to_constraint_system(&self) -> Result<ConstraintSystem<'_>, Error> {
        let mut id_generator = IdGenerator::default();
        // First, construct the list of initial guesses,
        // and assign them to solver variables.
        let mut initial_guesses: GeometryVariables<PointsState> = Default::default();
        // Maps labels to points
        let mut guessmap_points = HashMap::new();
        guessmap_points.extend(
            self.point_guesses
                .iter()
                .map(|pg| (pg.point.0.clone(), pg.guess)),
        );
        for point in &self.inner_points {
            let Some(guess) = guessmap_points.remove(&point.0) else {
                return Err(Error::MissingGuess {
                    label: point.0.clone(),
                });
            };
            initial_guesses.push_point(&mut id_generator, guess.x, guess.y);
        }
        let mut guessmap_scalars = HashMap::new();
        guessmap_scalars.extend(
            self.scalar_guesses
                .iter()
                .map(|sg| (sg.scalar.0.clone(), sg.guess)),
        );
        let mut initial_guesses = initial_guesses.done();
        for circle in &self.inner_circles {
            // Each circle should have a guess for its center and radius.
            // First, find the guess for its center:
            let center_label = format!("{}.center", circle.0);
            let Some(center_guess) = guessmap_points.remove(&center_label) else {
                return Err(Error::MissingGuess {
                    label: center_label,
                });
            };
            // Now, find the guess for its radius.
            let radius_label = format!("{}.radius", circle.0);
            let Some(radius_guess) = guessmap_scalars.remove(&radius_label) else {
                return Err(Error::MissingGuess {
                    label: radius_label,
                });
            };
            initial_guesses.push_circle(
                &mut id_generator,
                center_guess.x,
                center_guess.y,
                radius_guess,
            );
        }
        let mut initial_guesses = initial_guesses.done();
        for arc in &self.inner_arcs {
            // Each arc should have a guess for its 3 points (p, q, and center).
            let center_label = format!("{}.center", arc.0);
            let Some(center_guess) = guessmap_points.remove(&center_label) else {
                return Err(Error::MissingGuess {
                    label: center_label,
                });
            };
            let a_label = format!("{}.a", arc.0);
            let Some(a_guess) = guessmap_points.remove(&a_label) else {
                return Err(Error::MissingGuess { label: a_label });
            };
            let b_label = format!("{}.b", arc.0);
            let Some(b_guess) = guessmap_points.remove(&b_label) else {
                return Err(Error::MissingGuess { label: b_label });
            };
            initial_guesses.push_arc(&mut id_generator, a_guess, b_guess, center_guess);
        }
        if !guessmap_points.is_empty() {
            let labels: Vec<String> = guessmap_points.keys().cloned().collect();
            return Err(Error::UnusedGuesses { labels });
        }
        if !guessmap_scalars.is_empty() {
            let labels: Vec<String> = guessmap_scalars.keys().cloned().collect();
            return Err(Error::UnusedGuesses { labels });
        }

        // Good. Now we can define all the constraints, referencing the solver variables that
        // were defined in the previous step.
        let mut constraints = Vec::new();
        let datum_point_for_label = |label: &Label| -> Result<DatumPoint, crate::Error> {
            // Is the point a single geometric point?
            if let Some(point_id) = self.inner_points.iter().position(|p| p == &label.0) {
                let ids = initial_guesses.point_ids(point_id);
                return Ok(DatumPoint {
                    x_id: ids.x,
                    y_id: ids.y,
                });
            }
            // Maybe it's a point in a circle?
            if let Some(circle_id) = self
                .inner_circles
                .iter()
                .position(|circ| format!("{}.center", circ.0) == label.0.as_str())
            {
                let center = initial_guesses.circle_ids(circle_id).center;
                return Ok(DatumPoint {
                    x_id: center.x,
                    y_id: center.y,
                });
            }
            // Maybe it's a point in an arc?
            // Is it an arc's center?
            if let Some(arc_id) = self
                .inner_arcs
                .iter()
                .position(|arc| format!("{}.center", arc.0) == label.0.as_str())
            {
                let center = initial_guesses.arc_ids(arc_id).center;
                return Ok(center.into());
            }
            // Is it an arc's `p` point?
            if let Some(arc_id) = self
                .inner_arcs
                .iter()
                .position(|arc| format!("{}.a", arc.0) == label.0.as_str())
            {
                let a = initial_guesses.arc_ids(arc_id).a;
                return Ok(a.into());
            }
            // Is it an arc's `b` point?
            if let Some(arc_id) = self
                .inner_arcs
                .iter()
                .position(|arc| format!("{}.b", arc.0) == label.0.as_str())
            {
                let b = initial_guesses.arc_ids(arc_id).b;
                return Ok(b.into());
            }
            // Well, it wasn't any of the geometries we recognize.
            Err(Error::UndefinedPoint {
                label: label.0.clone(),
            })
        };
        let datum_distance_for_label = |label: &Label| -> Result<DatumDistance, crate::Error> {
            if let Some(circle_id) = self
                .inner_circles
                .iter()
                .position(|circ| format!("{}.radius", circ.0) == label.0.as_str())
            {
                let ids = initial_guesses.circle_ids(circle_id);
                return Ok(DatumDistance { id: ids.radius });
            }
            Err(Error::UndefinedPoint {
                label: label.0.clone(),
            })
        };

        for instr in &self.instructions {
            match instr {
                Instruction::DeclarePoint(_) => {}
                Instruction::DeclareCircle(_) => {}
                Instruction::DeclareArc(_) => {}
                Instruction::Line(_) => {}
                Instruction::CircleRadius(CircleRadius { circle, radius }) => {
                    let circ = &circle.0;
                    let center_id = datum_point_for_label(&Label(format!("{circ}.center")))?;
                    let radius_id = datum_distance_for_label(&Label(format!("{circ}.radius")))?;
                    constraints.push(Constraint::CircleRadius(
                        datatypes::Circle {
                            center: center_id,
                            radius: radius_id,
                        },
                        *radius,
                    ));
                }
                Instruction::ArcRadius(ArcRadius { arc_label, radius }) => {
                    let arc_label = &arc_label.0;
                    let circular_arc = CircularArc {
                        center: datum_point_for_label(&Label(format!("{arc_label}.center")))?,
                        a: datum_point_for_label(&Label(format!("{arc_label}.a")))?,
                        b: datum_point_for_label(&Label(format!("{arc_label}.b")))?,
                    };
                    constraints.push(Constraint::ArcRadius(circular_arc, *radius));
                }
                Instruction::IsArc(IsArc { arc_label }) => {
                    let arc_label = &arc_label.0;
                    let circular_arc = CircularArc {
                        center: datum_point_for_label(&Label(format!("{arc_label}.center")))?,
                        a: datum_point_for_label(&Label(format!("{arc_label}.a")))?,
                        b: datum_point_for_label(&Label(format!("{arc_label}.b")))?,
                    };
                    constraints.push(Constraint::Arc(circular_arc));
                }
                Instruction::PointLineDistance(PointLineDistance {
                    point,
                    line_p0,
                    line_p1,
                    distance,
                }) => {
                    let line = LineSegment {
                        p0: datum_point_for_label(line_p0)?,
                        p1: datum_point_for_label(line_p1)?,
                    };
                    let p = datum_point_for_label(point)?;
                    constraints.push(Constraint::PointLineDistance(p, line, *distance))
                }
                Instruction::Tangent(Tangent {
                    circle,
                    line_p0,
                    line_p1,
                }) => {
                    let circ = &circle.0;
                    let center_id = datum_point_for_label(&Label(format!("{circ}.center")))?;
                    let radius_id = datum_distance_for_label(&Label(format!("{circ}.radius")))?;
                    let line = LineSegment {
                        p0: datum_point_for_label(line_p0)?,
                        p1: datum_point_for_label(line_p1)?,
                    };
                    constraints.push(Constraint::LineTangentToCircle(
                        line,
                        datatypes::Circle {
                            center: center_id,
                            radius: radius_id,
                        },
                    ));
                }
                Instruction::FixPointComponent(FixPointComponent {
                    point,
                    component,
                    value,
                }) => {
                    if let Some(point_id) =
                        self.inner_points.iter().position(|label| label == point)
                    {
                        let ids = initial_guesses.point_ids(point_id);
                        let id = match component {
                            Component::X => ids.x,
                            Component::Y => ids.y,
                        };
                        constraints.push(Constraint::Fixed(id, *value));
                    } else if let Some(circle_label) = point.0.strip_suffix(".center") {
                        if let Some(circle_id) =
                            self.inner_circles.iter().position(|p| p.0 == circle_label)
                        {
                            let center = initial_guesses.circle_ids(circle_id).center;
                            let id = match component {
                                Component::X => center.x,
                                Component::Y => center.y,
                            };
                            constraints.push(Constraint::Fixed(id, *value))
                        }
                    } else {
                        return Err(Error::UndefinedPoint {
                            label: point.0.clone(),
                        });
                    }
                }
                Instruction::FixCenterPointComponent(FixCenterPointComponent {
                    object,
                    center_component,
                    value,
                }) => {
                    // Is this center talking about a circle object?
                    if let Some(circle_id) =
                        self.inner_circles.iter().position(|label| label == object)
                    {
                        let center = initial_guesses.circle_ids(circle_id).center;
                        let id = match center_component {
                            Component::X => center.x,
                            Component::Y => center.y,
                        };
                        constraints.push(Constraint::Fixed(id, *value));
                    // Is this center talking about an arc object?
                    } else if let Some(arc_id) =
                        self.inner_arcs.iter().position(|label| label == object)
                    {
                        let center = initial_guesses.arc_ids(arc_id).center;
                        let id = match center_component {
                            Component::X => center.x,
                            Component::Y => center.y,
                        };
                        constraints.push(Constraint::Fixed(id, *value));
                    } else {
                        return Err(Error::UndefinedPoint {
                            label: object.0.clone(),
                        });
                    }
                }
                Instruction::Vertical(Vertical { label }) => {
                    let p0 = datum_point_for_label(&label.0)?;
                    let p1 = datum_point_for_label(&label.1)?;
                    constraints.push(Constraint::Vertical(LineSegment { p0, p1 }));
                }
                Instruction::PointsCoincident(PointsCoincident { point0, point1 }) => {
                    let p0 = datum_point_for_label(point0)?;
                    let p1 = datum_point_for_label(point1)?;
                    constraints.push(Constraint::PointsCoincident(p0, p1));
                }
                Instruction::Midpoint(Midpoint { point0, point1, mp }) => {
                    let p0 = datum_point_for_label(point0)?;
                    let p1 = datum_point_for_label(point1)?;
                    let mp = datum_point_for_label(mp)?;
                    constraints.push(Constraint::Midpoint(LineSegment { p0, p1 }, mp));
                }
                Instruction::Symmetric(Symmetric { p0, p1, line }) => {
                    let p0 = datum_point_for_label(p0)?;
                    let p1 = datum_point_for_label(p1)?;
                    let line = (
                        datum_point_for_label(&line.0)?,
                        datum_point_for_label(&line.1)?,
                    );
                    let line = LineSegment {
                        p0: line.0,
                        p1: line.1,
                    };
                    constraints.push(Constraint::Symmetric(line, p0, p1));
                }
                Instruction::Horizontal(Horizontal { label }) => {
                    let p0 = datum_point_for_label(&label.0)?;
                    let p1 = datum_point_for_label(&label.1)?;
                    constraints.push(Constraint::Horizontal(LineSegment { p0, p1 }));
                }
                Instruction::Distance(Distance { label, distance }) => {
                    let p0 = datum_point_for_label(&label.0)?;
                    let p1 = datum_point_for_label(&label.1)?;
                    constraints.push(Constraint::Distance(p0, p1, *distance));
                }
                Instruction::Parallel(Parallel { line0, line1 }) => {
                    let p0 = datum_point_for_label(&line0.0)?;
                    let p1 = datum_point_for_label(&line0.1)?;
                    let p2 = datum_point_for_label(&line1.0)?;
                    let p3 = datum_point_for_label(&line1.1)?;
                    constraints.push(Constraint::lines_parallel([
                        LineSegment { p0, p1 },
                        LineSegment { p0: p2, p1: p3 },
                    ]));
                }
                Instruction::LinesEqualLength(LinesEqualLength { line0, line1 }) => {
                    let p0 = datum_point_for_label(&line0.0)?;
                    let p1 = datum_point_for_label(&line0.1)?;
                    let p2 = datum_point_for_label(&line1.0)?;
                    let p3 = datum_point_for_label(&line1.1)?;
                    constraints.push(Constraint::LinesEqualLength(
                        LineSegment { p0, p1 },
                        LineSegment { p0: p2, p1: p3 },
                    ));
                }
                Instruction::Perpendicular(Perpendicular { line0, line1 }) => {
                    let p0 = datum_point_for_label(&line0.0)?;
                    let p1 = datum_point_for_label(&line0.1)?;
                    let p2 = datum_point_for_label(&line1.0)?;
                    let p3 = datum_point_for_label(&line1.1)?;
                    constraints.push(Constraint::lines_perpendicular([
                        LineSegment { p0, p1 },
                        LineSegment { p0: p2, p1: p3 },
                    ]));
                }
                Instruction::AngleLine(AngleLine {
                    line0,
                    line1,
                    angle,
                }) => {
                    let p0 = datum_point_for_label(&line0.0)?;
                    let p1 = datum_point_for_label(&line0.1)?;
                    let p2 = datum_point_for_label(&line1.0)?;
                    let p3 = datum_point_for_label(&line1.1)?;
                    constraints.push(Constraint::LinesAtAngle(
                        LineSegment { p0, p1 },
                        LineSegment { p0: p2, p1: p3 },
                        AngleKind::Other(*angle),
                    ));
                }
            }
        }
        let initial_guesses = initial_guesses.done();

        Ok(ConstraintSystem {
            constraints,
            initial_guesses,
            inner_points: &self.inner_points,
            inner_circles: &self.inner_circles,
            inner_arcs: &self.inner_arcs,
            inner_lines: &self.inner_lines,
        })
    }
}

#[derive(Clone)]
pub struct ConstraintSystem<'a> {
    pub constraints: Vec<Constraint>,
    initial_guesses: GeometryVariables<DoneState>,
    inner_points: &'a [Label],
    inner_circles: &'a [Label],
    inner_arcs: &'a [Label],
    inner_lines: &'a [(Label, Label)],
}

impl ConstraintSystem<'_> {
    pub fn solve_no_metadata(&self, config: Config) -> Result<SolveOutcome, FailureOutcome> {
        crate::solve(&self.constraints, self.initial_guesses.variables(), config)
    }

    pub fn solve(&self) -> Result<Outcome, FailureOutcome> {
        self.solve_with_config(Default::default())
    }

    pub fn solve_with_config(&self, config: Config) -> Result<Outcome, FailureOutcome> {
        let num_vars = self.initial_guesses.len();
        let num_eqs = self.constraints.iter().map(|c| c.residual_dim()).sum();
        // Pass into the solver.
        let SolveOutcome {
            iterations,
            warnings,
            final_values,
            unsatisfied,
        } = self.solve_no_metadata(config)?;
        let num_points = self.inner_points.len();
        let num_circles = self.inner_circles.len();
        let num_arcs = self.inner_arcs.len();

        let mut final_points = IndexMap::with_capacity(num_points);
        for (i, point) in self.inner_points.iter().enumerate() {
            let x_id = 2 * i;
            let y_id = 2 * i + 1;
            let p = Point {
                x: final_values[x_id],
                y: final_values[y_id],
            };
            final_points.insert(point.0.clone(), p);
        }
        let start_of_circles = 2 * self.inner_points.len();
        let mut final_circles = IndexMap::with_capacity(num_circles);
        for (i, circle_label) in self.inner_circles.iter().enumerate() {
            let cx = final_values[start_of_circles + 3 * i]; // center x
            let cy = final_values[start_of_circles + 3 * i + 1]; // center y
            let rd = final_values[start_of_circles + 3 * i + 2]; // radius
            final_circles.insert(
                circle_label.0.clone(),
                Circle {
                    radius: rd,
                    center: Point { x: cx, y: cy },
                },
            );
        }
        let start_of_arcs = start_of_circles + 3 * self.inner_circles.len();
        let mut final_arcs = IndexMap::with_capacity(num_arcs);
        for (i, arc_label) in self.inner_arcs.iter().enumerate() {
            let ax = final_values[start_of_arcs + VARS_PER_ARC * i];
            let ay = final_values[start_of_arcs + VARS_PER_ARC * i + 1];
            let bx = final_values[start_of_arcs + VARS_PER_ARC * i + 2];
            let by = final_values[start_of_arcs + VARS_PER_ARC * i + 3];
            let cx = final_values[start_of_arcs + VARS_PER_ARC * i + 4];
            let cy = final_values[start_of_arcs + VARS_PER_ARC * i + 5];
            final_arcs.insert(
                arc_label.0.clone(),
                Arc {
                    center: Point { x: cx, y: cy },
                    a: Point { x: ax, y: ay },
                    b: Point { x: bx, y: by },
                },
            );
        }
        Ok(Outcome {
            unsatisfied,
            iterations,
            warnings,
            points: final_points,
            circles: final_circles,
            arcs: final_arcs,
            num_vars,
            lines: self.inner_lines.to_vec(),
            num_eqs,
        })
    }
}

#[derive(Debug)]
pub struct Outcome {
    pub unsatisfied: Vec<usize>,
    pub iterations: usize,
    pub warnings: Vec<Warning>,
    pub points: IndexMap<String, Point>,
    pub circles: IndexMap<String, Circle>,
    pub arcs: IndexMap<String, Arc>,
    pub lines: Vec<(Label, Label)>,
    pub num_vars: usize,
    pub num_eqs: usize,
}

impl Outcome {
    #[cfg(test)]
    pub fn get_point(&self, label: &str) -> Option<Point> {
        self.points.get(label).copied()
    }

    #[cfg(test)]
    pub fn get_circle(&self, label: &str) -> Option<Circle> {
        self.circles.get(label).copied()
    }

    #[cfg(test)]
    pub fn get_arc(&self, label: &str) -> Option<Arc> {
        self.arcs.get(label).copied()
    }
}
