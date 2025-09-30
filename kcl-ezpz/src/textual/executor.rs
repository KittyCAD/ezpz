use std::collections::HashMap;

use indexmap::IndexMap;

use crate::Config;
use crate::Constraint;
use crate::Error;
use crate::FailureOutcome;
use crate::IdGenerator;
use crate::Lint;
use crate::SolveOutcome;
use crate::constraints::AngleKind;
use crate::datatypes;
use crate::datatypes::DatumDistance;
use crate::datatypes::DatumPoint;
use crate::datatypes::LineSegment;
use crate::textual::geometry_variables::GeometryVariables;
use crate::textual::instruction::*;
use crate::textual::{Circle, Component, Label, Point};

use super::Instruction;
use super::Problem;

impl Problem {
    pub fn to_constraint_system(&self) -> Result<ConstraintSystem<'_>, Error> {
        let mut id_generator = IdGenerator::default();
        // First, construct the list of initial guesses,
        // and assign them to solver variables.
        let mut initial_guesses = GeometryVariables::default();
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
            if let Some(point_id) = self.inner_points.iter().position(|p| p == &label.0) {
                let ids = initial_guesses.get_point_ids(point_id);
                return Ok(DatumPoint {
                    x_id: ids.x,
                    y_id: ids.y,
                });
            }
            if let Some(circle_id) = self
                .inner_circles
                .iter()
                .position(|circ| format!("{}.center", circ.0) == label.0.as_str())
            {
                let center = initial_guesses.get_circle_ids(circle_id).center;
                return Ok(DatumPoint {
                    x_id: center.x,
                    y_id: center.y,
                });
            }
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
                let ids = initial_guesses.get_circle_ids(circle_id);
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
                        let ids = initial_guesses.get_point_ids(point_id);
                        let id = match component {
                            Component::X => ids.x,
                            Component::Y => ids.y,
                        };
                        constraints.push(Constraint::Fixed(id, *value));
                    } else if let Some(circle_label) = point.0.strip_suffix(".center") {
                        if let Some(circle_id) =
                            self.inner_circles.iter().position(|p| p.0 == circle_label)
                        {
                            let center = initial_guesses.get_circle_ids(circle_id).center;
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
                    circle,
                    center_component,
                    value,
                }) => {
                    if let Some(circle_id) =
                        self.inner_circles.iter().position(|label| label == circle)
                    {
                        let center = initial_guesses.get_circle_ids(circle_id).center;
                        let id = match center_component {
                            Component::X => center.x,
                            Component::Y => center.y,
                        };
                        constraints.push(Constraint::Fixed(id, *value));
                    } else {
                        return Err(Error::UndefinedPoint {
                            label: circle.0.clone(),
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

        Ok(ConstraintSystem {
            constraints,
            initial_guesses,
            inner_points: &self.inner_points,
            inner_circles: &self.inner_circles,
        })
    }
}

#[derive(Clone)]
pub struct ConstraintSystem<'a> {
    constraints: Vec<Constraint>,
    initial_guesses: GeometryVariables,
    inner_points: &'a [Label],
    inner_circles: &'a [Label],
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
            lints,
            final_values,
        } = self.solve_no_metadata(config)?;
        let num_points = self.inner_points.len();
        let num_circles = self.inner_circles.len();

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
        Ok(Outcome {
            iterations,
            lints,
            points: final_points,
            circles: final_circles,
            num_vars,
            num_eqs,
        })
    }
}

#[derive(Debug)]
pub struct Outcome {
    pub iterations: usize,
    pub lints: Vec<Lint>,
    pub points: IndexMap<String, Point>,
    pub circles: IndexMap<String, Circle>,
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
}
