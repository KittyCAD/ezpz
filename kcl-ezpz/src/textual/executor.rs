use std::collections::HashMap;

use indexmap::IndexMap;

use crate::Analysis;
use crate::Config;
use crate::Constraint;
use crate::ConstraintRequest;
use crate::FailureOutcome;
use crate::FreedomAnalysis;
use crate::IdGenerator;
use crate::NoAnalysis;
use crate::SolveOutcome;
use crate::SolveOutcomeAnalysis;
use crate::Warning;
use crate::datatypes;
use crate::datatypes::AngleKind;
use crate::datatypes::CircularArc;
use crate::datatypes::DatumDistance;
use crate::datatypes::DatumPoint;
use crate::datatypes::LineSegment;
use crate::error::TextualError;
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
    /// Build a [ConstraintSystem] which models the system in this problem.
    /// Error means this problem was not properly specified, e.g. it could be
    /// missing a variable used in a constraint.
    pub fn to_constraint_system(&self) -> Result<ConstraintSystem<'_>, TextualError> {
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
                return Err(TextualError::MissingGuess {
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
                return Err(TextualError::MissingGuess {
                    label: center_label,
                });
            };
            // Now, find the guess for its radius.
            let radius_label = format!("{}.radius", circle.0);
            let Some(radius_guess) = guessmap_scalars.remove(&radius_label) else {
                return Err(TextualError::MissingGuess {
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
                return Err(TextualError::MissingGuess {
                    label: center_label,
                });
            };
            let a_label = format!("{}.a", arc.0);
            let Some(a_guess) = guessmap_points.remove(&a_label) else {
                return Err(TextualError::MissingGuess { label: a_label });
            };
            let b_label = format!("{}.b", arc.0);
            let Some(b_guess) = guessmap_points.remove(&b_label) else {
                return Err(TextualError::MissingGuess { label: b_label });
            };
            initial_guesses.push_arc(&mut id_generator, a_guess, b_guess, center_guess);
        }
        if !guessmap_points.is_empty() {
            let labels: Vec<String> = guessmap_points.keys().cloned().collect();
            return Err(TextualError::UnusedGuesses { labels });
        }
        if !guessmap_scalars.is_empty() {
            let labels: Vec<String> = guessmap_scalars.keys().cloned().collect();
            return Err(TextualError::UnusedGuesses { labels });
        }

        // Good. Now we can define all the constraints, referencing the solver variables that
        // were defined in the previous step.
        let mut constraints = Vec::new();
        let datum_point_for_label = |label: &Label| -> Result<DatumPoint, TextualError> {
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
            // Is it an arc's start point (labeled as `.a` in textual format)?
            if let Some(arc_id) = self
                .inner_arcs
                .iter()
                .position(|arc| format!("{}.a", arc.0) == label.0.as_str())
            {
                let start = initial_guesses.arc_ids(arc_id).start;
                return Ok(start.into());
            }
            // Is it an arc's end point (labeled as `.b` in textual format)?
            if let Some(arc_id) = self
                .inner_arcs
                .iter()
                .position(|arc| format!("{}.b", arc.0) == label.0.as_str())
            {
                let end = initial_guesses.arc_ids(arc_id).end;
                return Ok(end.into());
            }
            // Well, it wasn't any of the geometries we recognize.
            Err(TextualError::UndefinedPoint {
                label: label.0.clone(),
            })
        };
        let datum_distance_for_label = |label: &Label| -> Result<DatumDistance, TextualError> {
            if let Some(circle_id) = self
                .inner_circles
                .iter()
                .position(|circ| format!("{}.radius", circ.0) == label.0.as_str())
            {
                let ids = initial_guesses.circle_ids(circle_id);
                return Ok(DatumDistance { id: ids.radius });
            }
            Err(TextualError::UndefinedPoint {
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
                        start: datum_point_for_label(&Label(format!("{arc_label}.a")))?,
                        end: datum_point_for_label(&Label(format!("{arc_label}.b")))?,
                    };
                    constraints.push(Constraint::ArcRadius(circular_arc, *radius));
                }
                Instruction::IsArc(IsArc { arc_label }) => {
                    let arc_label = &arc_label.0;
                    let circular_arc = CircularArc {
                        center: datum_point_for_label(&Label(format!("{arc_label}.center")))?,
                        start: datum_point_for_label(&Label(format!("{arc_label}.a")))?,
                        end: datum_point_for_label(&Label(format!("{arc_label}.b")))?,
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
                        return Err(TextualError::UndefinedPoint {
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
                        return Err(TextualError::UndefinedPoint {
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
                Instruction::PointArcCoincident(PointArcCoincident { point, arc }) => {
                    let p = datum_point_for_label(point)?;
                    let arc_label = &arc.0;
                    let datum_arc = CircularArc {
                        center: datum_point_for_label(&Label(format!("{arc_label}.center")))?,
                        start: datum_point_for_label(&Label(format!("{arc_label}.a")))?,
                        end: datum_point_for_label(&Label(format!("{arc_label}.b")))?,
                    };
                    constraints.push(Constraint::PointArcCoincident(datum_arc, p));
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
                Instruction::ArcLength(arc_length) => {
                    let arc_label = &arc_length.arc.0;
                    let length = arc_length.distance;
                    let circular_arc = CircularArc {
                        center: datum_point_for_label(&Label(format!("{arc_label}.center")))?,
                        start: datum_point_for_label(&Label(format!("{arc_label}.a")))?,
                        end: datum_point_for_label(&Label(format!("{arc_label}.b")))?,
                    };
                    constraints.push(Constraint::ArcLength(circular_arc, length));
                }
            }
        }
        let initial_guesses = initial_guesses.done();

        // At some point, the textual format should support setting priority.
        // For now, set it to max priority.
        let priority = 0;
        let constraints = constraints
            .into_iter()
            .map(|c| ConstraintRequest::new(c, priority))
            .collect();

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

/// A constraint system that ezpz could solve,
/// built from the ezpz text format.
#[derive(Clone)]
pub struct ConstraintSystem<'a> {
    /// Constraints from the text input.
    pub constraints: Vec<ConstraintRequest>,
    initial_guesses: GeometryVariables<DoneState>,
    inner_points: &'a [Label],
    inner_circles: &'a [Label],
    inner_arcs: &'a [Label],
    inner_lines: &'a [(Label, Label)],
}

impl ConstraintSystem<'_> {
    /// Solve, without carrying through metadata about the solve.
    pub fn solve_no_metadata(&self, config: Config) -> Result<SolveOutcome, FailureOutcome> {
        crate::solve_with_priority(&self.constraints, self.initial_guesses.variables(), config)
    }

    fn solve_no_metadata_inner<A: Analysis>(
        &self,
        config: Config,
    ) -> Result<SolveOutcomeAnalysis<A>, FailureOutcome> {
        crate::solve_with_priority_inner(
            &self.constraints,
            self.initial_guesses.variables(),
            config,
        )
    }

    /// Solve, with metadata about the solve.
    pub fn solve(&self) -> Result<Outcome, FailureOutcome> {
        self.solve_with_config(Default::default())
    }

    /// Solve, and analyze the degrees of freedom.
    pub fn solve_with_config_analysis(
        &self,
        config: Config,
    ) -> Result<OutcomeAnalysis, FailureOutcome> {
        let (analysis, outcome) = self.solve_with_config_inner::<FreedomAnalysis>(config)?;
        Ok(OutcomeAnalysis { analysis, outcome })
    }

    /// Solve, but give a non-default config.
    pub fn solve_with_config(&self, config: Config) -> Result<Outcome, FailureOutcome> {
        let (NoAnalysis, outcome) = self.solve_with_config_inner::<NoAnalysis>(config)?;
        Ok(outcome)
    }

    fn solve_with_config_inner<A: Analysis>(
        &self,
        config: Config,
    ) -> Result<(A, Outcome), FailureOutcome> {
        let num_vars = self.initial_guesses.len();
        let num_eqs = self
            .constraints
            .iter()
            .map(|c| c.constraint.residual_dim())
            .sum();
        // Pass into the solver.
        let SolveOutcomeAnalysis {
            analysis,
            outcome:
                SolveOutcome {
                    iterations,
                    warnings,
                    final_values,
                    unsatisfied,
                    priority_solved,
                },
        } = self.solve_no_metadata_inner::<A>(config)?;
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
        Ok((
            analysis,
            Outcome {
                priority_solved,
                unsatisfied,
                iterations,
                warnings,
                points: final_points,
                circles: final_circles,
                arcs: final_arcs,
                num_vars,
                lines: self.inner_lines.to_vec(),
                num_eqs,
            },
        ))
    }
}

/// Outcome of successfully solving a constraint system.
#[derive(Debug)]
pub struct Outcome {
    /// All constraint IDs which couldn't be satisfied.
    pub unsatisfied: Vec<usize>,
    /// How many iterations of the core Newton-Gauss loop this system required.
    pub iterations: usize,
    /// Anything bad that users should know about.
    pub warnings: Vec<Warning>,
    /// Points the user defined, with their final solved values.
    pub points: IndexMap<String, Point>,
    /// Circles the user defined, with their final solved values.
    pub circles: IndexMap<String, Circle>,
    /// Arcs the user defined, with their final solved values.
    pub arcs: IndexMap<String, Arc>,
    /// Lines the user defined, with labels for their two points.
    pub lines: Vec<(Label, Label)>,
    /// Size of the constraint system. Number of variables being solved for.
    pub num_vars: usize,
    /// Size of the constraint system. Number of residual equations.
    pub num_eqs: usize,
    /// The lowest priority solved before the constraint solver stopped.
    /// The constraint solver stops when it cannot solve any more constraints, i.e.
    /// got an error.
    pub priority_solved: u32,
}

/// Outcome of solving an ezpz system, and degrees-of-freedom analysis.
#[derive(Debug)]
pub struct OutcomeAnalysis {
    /// Degrees of freedom analysis
    pub analysis: FreedomAnalysis,
    /// Outcome of solving the constraint system.
    pub outcome: Outcome,
}

impl Outcome {
    /// Look up a point by its label.
    #[cfg(test)]
    pub fn get_point(&self, label: &str) -> Option<Point> {
        self.points.get(label).copied()
    }

    /// Look up a circle by its label.
    #[cfg(test)]
    pub fn get_circle(&self, label: &str) -> Option<Circle> {
        self.circles.get(label).copied()
    }

    /// Look up an arc by its label.
    #[cfg(test)]
    pub fn get_arc(&self, label: &str) -> Option<Arc> {
        self.arcs.get(label).copied()
    }
}

impl OutcomeAnalysis {
    /// Look up a point by its label.
    #[cfg(test)]
    pub fn get_point(&self, label: &str) -> Option<Point> {
        self.outcome.get_point(label)
    }

    /// Look up a circle by its label.
    #[cfg(test)]
    pub fn get_circle(&self, label: &str) -> Option<Circle> {
        self.outcome.get_circle(label)
    }

    /// Look up an arc by its label.
    #[cfg(test)]
    pub fn get_arc(&self, label: &str) -> Option<Arc> {
        self.outcome.get_arc(label)
    }

    /// Are all constraints satisfied?
    #[cfg(test)]
    pub fn is_satisfied(&self) -> bool {
        !self.is_unsatisfied()
    }

    /// Are any constraints not satisfied?
    #[cfg(test)]
    pub fn is_unsatisfied(&self) -> bool {
        !self.outcome.unsatisfied.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::textual::{PointGuess, instruction::FixPointComponent};

    fn empty_problem() -> Problem {
        Problem {
            instructions: Vec::new(),
            inner_points: Vec::new(),
            inner_circles: Vec::new(),
            inner_arcs: Vec::new(),
            inner_lines: Vec::new(),
            point_guesses: Vec::new(),
            scalar_guesses: Vec::new(),
        }
    }

    #[test]
    fn missing_guess_is_reported() {
        let mut problem = empty_problem();
        problem.inner_points.push(Label::from("p"));
        let err = problem
            .to_constraint_system()
            .err()
            .expect("expected missing guess");
        assert!(matches!(err, TextualError::MissingGuess { label } if label == "p"));
    }

    #[test]
    fn unused_guesses_are_detected() {
        let mut problem = empty_problem();
        problem.point_guesses.push(PointGuess {
            point: Label::from("ghost"),
            guess: Point { x: 0.0, y: 0.0 },
        });

        let err = problem
            .to_constraint_system()
            .err()
            .expect("expected unused guess error");
        match err {
            TextualError::UnusedGuesses { labels } => {
                assert_eq!(labels.len(), 1);
                assert_eq!(labels[0], "ghost");
            }
            other => panic!("unexpected error: {other:?}"),
        }
    }

    #[test]
    fn undefined_point_in_instruction_errors() {
        let mut problem = empty_problem();
        problem.inner_points.push(Label::from("p"));
        problem.point_guesses.push(PointGuess {
            point: Label::from("p"),
            guess: Point { x: 0.0, y: 0.0 },
        });
        problem
            .instructions
            .push(Instruction::FixPointComponent(FixPointComponent {
                point: Label::from("missing"),
                component: Component::X,
                value: 2.5,
            }));

        let err = problem
            .to_constraint_system()
            .err()
            .expect("expected undefined point error");
        assert!(matches!(err, TextualError::UndefinedPoint { label } if label == "missing"));
    }
}
