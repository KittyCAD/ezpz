use std::collections::HashMap;

use crate::Constraint;
use crate::Error;
use crate::IdGenerator;
use crate::Lint;
use crate::SolveOutcome;
use crate::constraints::AngleKind;
use crate::datatypes::DatumPoint;
use crate::datatypes::LineSegment;
use crate::textual::Component;
use crate::textual::Label;
use crate::textual::Point;
use crate::textual::instruction::AngleLine;
use crate::textual::instruction::Distance;
use crate::textual::instruction::FixPointComponent;
use crate::textual::instruction::Horizontal;
use crate::textual::instruction::Parallel;
use crate::textual::instruction::Vertical;

use super::Instruction;
use super::Problem;

impl Problem {
    pub fn solve(&self) -> Result<Outcome, Error> {
        // First, construct the list of initial guesses,
        // and assign them to solver variables.
        let num_points = self.inner_points.len();
        let mut id_generator = IdGenerator::default();
        let mut initial_guesses: Vec<_> = Vec::with_capacity(self.inner_points.len() * 2);
        let mut guessmap = HashMap::new();
        guessmap.extend(
            self.point_guesses
                .iter()
                .map(|pg| (pg.point.0.clone(), pg.guess)),
        );
        for point in &self.inner_points {
            let Some(guess) = guessmap.remove(&point.0) else {
                return Err(Error::MissingGuess {
                    label: point.0.clone(),
                });
            };
            initial_guesses.push((id_generator.next_id(), guess.x));
            initial_guesses.push((id_generator.next_id(), guess.y));
        }
        if !guessmap.is_empty() {
            let labels: Vec<String> = guessmap.keys().cloned().collect();
            return Err(Error::UnusedGuesses { labels });
        }

        // Good. Now we can define all the constraints, referencing the solver variables that
        // were defined in the previous step.
        let mut constraints = Vec::new();
        let datum_point_for_label = |label: &Label| -> Result<DatumPoint, crate::Error> {
            let point_id = self.inner_points.iter().position(|p| p == &label.0).ok_or(
                Error::UndefinedPoint {
                    label: label.0.clone(),
                },
            )?;
            let x_id = initial_guesses[2 * point_id].0;
            let y_id = initial_guesses[2 * point_id + 1].0;
            Ok(DatumPoint { x_id, y_id })
        };

        for instr in &self.instructions {
            match instr {
                Instruction::DeclarePoint(_) => {}
                Instruction::FixPointComponent(FixPointComponent {
                    point,
                    component,
                    value,
                }) => {
                    let point_id = self.inner_points.iter().position(|p| p == point).ok_or(
                        Error::UndefinedPoint {
                            label: point.0.clone(),
                        },
                    )?;
                    let index = match component {
                        Component::X => 2 * point_id,
                        Component::Y => 2 * point_id + 1,
                    };
                    let id = initial_guesses[index].0;
                    constraints.push(Constraint::Fixed(id, *value));
                }
                Instruction::Vertical(Vertical { label }) => {
                    let p0 = datum_point_for_label(&label.0)?;
                    let p1 = datum_point_for_label(&label.1)?;
                    constraints.push(Constraint::Vertical(LineSegment { p0, p1 }));
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

        let num_vars = initial_guesses.len();
        let num_eqs = constraints.iter().map(|c| c.residual_dim()).sum();

        // Pass into the solver.
        let SolveOutcome {
            iterations,
            lints,
            final_values,
        } = crate::solve(constraints, initial_guesses)?;

        let mut final_points = HashMap::with_capacity(num_points);
        for (i, point) in self.inner_points.iter().enumerate() {
            let x_id = 2 * i;
            let y_id = 2 * i + 1;
            let p = Point {
                x: final_values[x_id],
                y: final_values[y_id],
            };
            final_points.insert(point.0.clone(), p);
        }
        Ok(Outcome {
            iterations,
            lints,
            points: final_points,
            num_vars,
            num_eqs,
        })
    }
}

#[derive(Debug)]
pub struct Outcome {
    pub iterations: usize,
    pub lints: Vec<Lint>,
    pub points: HashMap<String, Point>,
    pub num_vars: usize,
    pub num_eqs: usize,
}

impl Outcome {
    pub fn get_point(&self, label: &str) -> Option<Point> {
        self.points.get(label).copied()
    }
}
