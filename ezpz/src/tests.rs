use std::{f64::consts::PI, str::FromStr};

use super::*;
use crate::{
    CircleSide, LineSide,
    datatypes::{
        Angle, AngleKind,
        inputs::{DatumCircle, DatumCircularArc, DatumDistance, DatumLineSegment, DatumPoint},
        outputs::Point,
    },
    textual::{OutcomeAnalysis, Problem},
    vector::V,
};

mod proptests;

fn run(test_case: &str) -> OutcomeAnalysis {
    run_with_config(test_case, Default::default())
}

fn run_with_config(test_case: &str, config: Config) -> OutcomeAnalysis {
    let txt = std::fs::read_to_string(format!("../test_cases/{test_case}/problem.md")).unwrap();
    let problem = parse_problem(&txt);
    let system = problem.to_constraint_system().unwrap();
    system.solve_with_config_analysis(config).unwrap()
}

fn parse_problem(txt: &str) -> Problem {
    match Problem::from_str(txt) {
        Ok(x) => x,
        Err(e) => {
            eprintln!("{e}");
            panic!("Could not parse");
        }
    }
}

#[test]
fn empty() {
    // This constraint references variable 0.
    let constraints = vec![ConstraintRequest::highest_priority(Constraint::Fixed(
        0, 0.0,
    ))];
    // We don't pass any variables, so this should return an error,
    // because the constraint requires variable 0, and it's not given.
    let _e = solve(constraints.as_slice(), Vec::new(), Default::default()).unwrap_err();
}

#[test]
fn it_returns_best_satisfied_solution() {
    // If a lower-priority constraint causes the higher-priority constraints to be unsatisfied,
    // use the previous solution (i.e. the satisfied one, with only higher-priority constraints).

    let mut ids = IdGenerator::default();
    let var = ids.next_id();

    let high_priority = 0;
    let low_priority = 1;
    let constraints = vec![
        ConstraintRequest::new(Constraint::Fixed(var, 0.0), high_priority),
        ConstraintRequest::new(Constraint::Fixed(var, 1.0), low_priority),
        ConstraintRequest::new(Constraint::Fixed(var, 2.0), low_priority),
    ];
    let initial_guesses = vec![(var, 0.5)];
    let solved = solve_analysis(&constraints, initial_guesses, Config::default()).unwrap();
    assert!(solved.outcome.is_satisfied());
    assert_eq!(solved.as_ref().priority_solved, high_priority);
}

#[test]
fn initials_become_finals_if_no_constraints() {
    // If a lower-priority constraint causes the higher-priority constraints to be unsatisfied,
    // use the previous solution (i.e. the satisfied one, with only higher-priority constraints).

    let mut ids = IdGenerator::default();
    let var = ids.next_id();

    let constraints = vec![];
    let initial_guess = 0.5;
    let initial_guesses = vec![(var, initial_guess)];
    let solved = solve_analysis(&constraints, initial_guesses, Config::default()).unwrap();
    assert!(solved.as_ref().is_satisfied());
    assert_eq!(solved.as_ref().final_values, vec![initial_guess]);
}

#[test]
fn priority_solver_reports_original_indices() {
    // Place a lower-priority constraint before higher-priority ones so their indices shift.
    // When the high-priority subset is unsatisfied, the reported indices should still match
    // the original request list.
    let mut ids = IdGenerator::default();
    let var = ids.next_id();

    let high_priority = 0;
    let low_priority = 1;
    let constraints = vec![
        ConstraintRequest::new(Constraint::Fixed(var, 0.0), low_priority),
        ConstraintRequest::new(Constraint::Fixed(var, 1.0), high_priority),
        ConstraintRequest::new(Constraint::Fixed(var, 2.0), high_priority),
    ];
    let initial_guess = vec![(var, 0.5)];

    let solved = solve_analysis(&constraints, initial_guess, Config::default()).unwrap();
    assert_eq!(solved.as_ref().unsatisfied, vec![1, 2]);
    assert_eq!(solved.as_ref().priority_solved, high_priority);
}

#[test]
fn too_many_variables() {
    // If you give too many variables and not enough guesses,
    // there should be a nice error.
    let id = 0;
    let constraints = vec![ConstraintRequest::highest_priority(Constraint::Fixed(
        id, 0.0,
    ))];
    let initial_guess = vec![];

    let err = solve_analysis(&constraints, initial_guess, Config::default())
        .unwrap_err()
        .error;
    assert!(matches!(
        err,
        NonLinearSystemError::MissingGuess {
            constraint_id: 0,
            variable: 0
        }
    ));
}

#[test]
fn coincident() {
    let solved = run("coincident");
    assert!(solved.is_satisfied());
    assert!(!solved.analysis.is_underconstrained());
    // P and Q are coincident, so they should be equal.
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 3.0, y: 3.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 3.0, y: 3.0 });
}

// #[test]
// fn massive() {
//     let solved = run("massive_parallel_system");
//     assert!(solved.is_satisfied());
//     assert!(!solved.analysis.is_underconstrained());
// }

#[test]
fn symmetric() {
    let solved = run("symmetric");
    assert!(solved.is_satisfied());
    assert!(!solved.analysis.is_underconstrained());
    // P and Q are fixed
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 2.0, y: 2.0 });

    // Because the line L is x = y,
    // these points lie symmetric across it.
    assert_points_eq(solved.get_point("a").unwrap(), Point { x: 0.5, y: 0.4 });
    assert_points_eq(solved.get_point("b").unwrap(), Point { x: 0.4, y: 0.5 });
}

#[test]
fn perpdist() {
    let solved = run("perpdist");
    assert!(solved.is_satisfied());
    // P and Q are fixed:
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 2.0, y: 3.0 });
    assert_points_eq(
        solved.get_point("a").unwrap(),
        Point {
            x: 0.10055560181546289,
            y: 1.9536090405127489,
        },
    );
    // A is underdetermined, it has to be a certain distance from the line, but that leaves
    // a range of possible absolute positions it could be at.
    assert!(solved.analysis.is_underconstrained());
    assert_eq!(
        solved.analysis.into_underconstrained(),
        vec![4, 5],
        "P and Q are constrained, but A is not, it could move along the PQ line as long as it stays a fixed perp distance away."
    );
}

#[test]
fn perpdist_negative() {
    // Just like the `perpdist` test case, except the perpendicular distance is negative
    // instead of positive. So the point should be flipped to the other side of the line.
    let solved = run("perpdist_negative");
    assert!(solved.is_satisfied());
    assert!(solved.analysis.is_underconstrained());
    assert_eq!(
        solved.analysis.underconstrained(),
        vec![4, 5],
        "P and Q are constrained, but A is not, it could move along the PQ line as long as it stays a fixed perp distance away."
    );
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 2.0, y: 3.0 });
    assert_points_eq(
        solved.get_point("a").unwrap(),
        Point {
            x: 1.5192717280306194,
            y: 0.476131954511605,
        },
    );
}

#[test]
fn midpoint() {
    let solved = run("midpoint");
    assert!(solved.is_satisfied());
    assert!(!solved.analysis.is_underconstrained());
    // P and Q have a midpoint M.
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 2.0, y: 3.0 });
    assert_points_eq(solved.get_point("m").unwrap(), Point { x: 1.0, y: 1.5 });
}

#[test]
fn underconstrained() {
    let solved = run("underconstrained");
    assert!(solved.analysis.is_underconstrained());
    assert!(solved.is_satisfied());
    assert_eq!(solved.analysis.underconstrained(), vec![0, 1]);
    // p should be whatever the user's initial guess was.
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 1.0, y: 1.0 });
    // q should be what it was constrained to be.
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 0.0, y: 0.0 });
}

#[test]
fn tiny() {
    let solved = run("tiny");
    assert!(solved.is_satisfied());
    assert!(!solved.analysis.is_underconstrained());
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 0.0, y: 0.0 });
}

#[test]
fn inconsistent() {
    // This has inconsistent requirements:
    // p should be (1,4) and it should ALSO be (4,1).
    // Because they can't be simultaneously satisfied, we should find a
    // solution which minimizes the squared error instead.
    let solved = run("inconsistent");
    assert!(!solved.is_satisfied());
    assert!(!solved.analysis.is_underconstrained()); // If anything it's overconstrained not under.
    assert_points_eq(solved.get_point("o").unwrap(), Point { x: 0.0, y: 0.0 });
    // (2.5, 2.5) is midway between the two inconsistent requirement points.
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 2.5, y: 2.5 });
}

#[test]
fn weight_biases_inconsistent_solution() {
    // Two competing Fixed constraints on the same variable at the same priority:
    // target=0 with default weight 1, and target=100 with weight 100. With equal
    // weights the least-squares minimum sits at the midpoint (50). With 100x on
    // the second target the solver should pull almost all the way to 100.
    let mut ids = IdGenerator::default();
    let var_id = ids.next_id();

    let constraints = vec![
        ConstraintRequest::highest_priority(Constraint::Fixed(var_id, 0.0)),
        ConstraintRequest::highest_priority(Constraint::Fixed(var_id, 100.0)).with_weight(100.0),
    ];
    let initial_guesses = vec![(var_id, 50.0)];

    let solved = solve(&constraints, initial_guesses, Config::default()).unwrap();
    let final_value = solved.final_values()[0];
    assert!(
        final_value > 99.0,
        "weighted target should dominate; got {final_value}",
    );

    // Equal weights for comparison: should settle at the midpoint.
    let baseline = vec![
        ConstraintRequest::highest_priority(Constraint::Fixed(var_id, 0.0)),
        ConstraintRequest::highest_priority(Constraint::Fixed(var_id, 100.0)),
    ];
    let baseline_solved = solve(&baseline, vec![(var_id, 50.0)], Config::default()).unwrap();
    assert_nearly_eq(baseline_solved.final_values()[0], 50.0);
}

#[test]
fn circle() {
    let solved = run("circle");
    assert!(solved.is_satisfied());
    assert!(!solved.analysis.is_underconstrained());
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 5.0, y: 5.0 });
    let circle_a = solved.get_circle("a").unwrap();
    // From the problem:
    // circle a
    // radius(a, 3.4)
    // a.center = (0.1, 0.2)
    assert_nearly_eq(circle_a.radius, 3.4);
    assert_points_eq(circle_a.center, Point { x: 0.1, y: 0.2 });
}

#[test]
fn circle_center() {
    // Very similar to test `circle` above,
    // except it gives each constraint on the center separately.
    let solved = run("circle_center");
    assert!(!solved.analysis.is_underconstrained());
    assert!(solved.is_satisfied());
    let circle_a = solved.get_circle("a").unwrap();
    assert_nearly_eq(circle_a.radius, 1.0);
    assert_points_eq(circle_a.center, Point { x: 0.0, y: 0.0 });
}

#[test]
fn circle_tangent() {
    // `tangent(...)` now starts with `LineSide::Undefined`, so the side is inferred
    // from the initial geometry. This fixture starts the circle below the line.
    let solved = run("circle_tangent");
    assert!(solved.is_satisfied());
    assert!(!solved.analysis.is_underconstrained());
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 0.0, y: 3.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 5.0, y: 3.0 });
    let circle_a = solved.get_circle("a").unwrap();
    assert_nearly_eq(circle_a.center.y, 1.5);
    assert_nearly_eq(circle_a.radius, 1.5);
}

#[test]
fn circle_tangent_other_dir() {
    // Just like `circle_tangent` but using line QP instead of PQ. Reversing the
    // line direction reverses the preferred side of the oriented tangent.
    let solved = run("circle_tangent_other_dir");
    assert!(solved.is_satisfied());
    assert!(!solved.analysis.is_underconstrained());
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 0.0, y: 3.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 5.0, y: 3.0 });
    let circle_a = solved.get_circle("a").unwrap();
    assert_nearly_eq(circle_a.center.y, 1.5);
    assert_nearly_eq(circle_a.radius, 1.5);
}

#[test]
fn line_tangent_left_explicit() {
    let mut ids = IdGenerator::default();
    let p0 = DatumPoint::new(&mut ids);
    let p1 = DatumPoint::new(&mut ids);
    let center = DatumPoint::new(&mut ids);
    let radius = DatumDistance::new(ids.next_id());
    let line = DatumLineSegment::new(p0, p1);
    let circle = DatumCircle { center, radius };

    let constraints = vec![
        ConstraintRequest::highest_priority(Constraint::Fixed(p0.id_y(), 3.0)),
        ConstraintRequest::highest_priority(Constraint::Fixed(p1.id_y(), 3.0)),
        ConstraintRequest::highest_priority(Constraint::CircleRadius(circle, 1.5)),
        ConstraintRequest::highest_priority(Constraint::LineTangentToCircle(
            line,
            circle,
            LineSide::Left,
        )),
    ];
    let initial_guesses = vec![
        (p0.id_x(), 0.0),
        (p0.id_y(), 3.0),
        (p1.id_x(), 5.0),
        (p1.id_y(), 3.0),
        (center.id_x(), 2.0),
        (center.id_y(), 1.5),
        (radius.id, 1.5),
    ];

    let solved = solve(&constraints, initial_guesses, Config::default()).unwrap();
    assert!(solved.is_satisfied());
    let solved_circle = solved.final_value_circle(&circle);
    assert_nearly_eq(solved_circle.center.y, 4.5);
    assert_nearly_eq(solved_circle.radius, 1.5);
}

#[test]
fn line_tangent_right_explicit() {
    let mut ids = IdGenerator::default();
    let p0 = DatumPoint::new(&mut ids);
    let p1 = DatumPoint::new(&mut ids);
    let center = DatumPoint::new(&mut ids);
    let radius = DatumDistance::new(ids.next_id());
    let line = DatumLineSegment::new(p0, p1);
    let circle = DatumCircle { center, radius };

    let constraints = vec![
        ConstraintRequest::highest_priority(Constraint::Fixed(p0.id_y(), 3.0)),
        ConstraintRequest::highest_priority(Constraint::Fixed(p1.id_y(), 3.0)),
        ConstraintRequest::highest_priority(Constraint::CircleRadius(circle, 1.5)),
        ConstraintRequest::highest_priority(Constraint::LineTangentToCircle(
            line,
            circle,
            LineSide::Right,
        )),
    ];
    let initial_guesses = vec![
        (p0.id_x(), 0.0),
        (p0.id_y(), 3.0),
        (p1.id_x(), 5.0),
        (p1.id_y(), 3.0),
        (center.id_x(), 2.0),
        (center.id_y(), 4.5),
        (radius.id, 1.5),
    ];

    let solved = solve(&constraints, initial_guesses, Config::default()).unwrap();
    assert!(solved.is_satisfied());
    let solved_circle = solved.final_value_circle(&circle);
    assert_nearly_eq(solved_circle.center.y, 1.5);
    assert_nearly_eq(solved_circle.radius, 1.5);
}

#[test]
fn line_tangent_left_inferred() {
    let mut ids = IdGenerator::default();
    let p0 = DatumPoint::new(&mut ids);
    let p1 = DatumPoint::new(&mut ids);
    let center = DatumPoint::new(&mut ids);
    let radius = DatumDistance::new(ids.next_id());
    let line = DatumLineSegment::new(p0, p1);
    let circle = DatumCircle { center, radius };

    let constraints = vec![
        ConstraintRequest::highest_priority(Constraint::Fixed(p0.id_y(), 3.0)),
        ConstraintRequest::highest_priority(Constraint::Fixed(p1.id_y(), 3.0)),
        ConstraintRequest::highest_priority(Constraint::CircleRadius(circle, 1.5)),
        ConstraintRequest::highest_priority(Constraint::LineTangentToCircle(
            line,
            circle,
            LineSide::Undefined,
        )),
    ];
    let initial_guesses = vec![
        (p0.id_x(), 0.0),
        (p0.id_y(), 3.0),
        (p1.id_x(), 5.0),
        (p1.id_y(), 3.0),
        (center.id_x(), 2.0),
        (center.id_y(), 4.5),
        (radius.id, 1.5),
    ];

    let solved = solve(&constraints, initial_guesses, Config::default()).unwrap();
    assert!(solved.is_satisfied());
    let solved_circle = solved.final_value_circle(&circle);
    assert_nearly_eq(solved_circle.center.y, 4.5);
    assert_nearly_eq(solved_circle.radius, 1.5);
}

#[test]
fn line_tangent_right_inferred() {
    let mut ids = IdGenerator::default();
    let p0 = DatumPoint::new(&mut ids);
    let p1 = DatumPoint::new(&mut ids);
    let center = DatumPoint::new(&mut ids);
    let radius = DatumDistance::new(ids.next_id());
    let line = DatumLineSegment::new(p0, p1);
    let circle = DatumCircle { center, radius };

    let constraints = vec![
        ConstraintRequest::highest_priority(Constraint::Fixed(p0.id_y(), 3.0)),
        ConstraintRequest::highest_priority(Constraint::Fixed(p1.id_y(), 3.0)),
        ConstraintRequest::highest_priority(Constraint::CircleRadius(circle, 1.5)),
        ConstraintRequest::highest_priority(Constraint::LineTangentToCircle(
            line,
            circle,
            LineSide::Undefined,
        )),
    ];
    let initial_guesses = vec![
        (p0.id_x(), 0.0),
        (p0.id_y(), 3.0),
        (p1.id_x(), 5.0),
        (p1.id_y(), 3.0),
        (center.id_x(), 2.0),
        (center.id_y(), 1.5),
        (radius.id, 1.5),
    ];

    let solved = solve(&constraints, initial_guesses, Config::default()).unwrap();
    assert!(solved.is_satisfied());
    let solved_circle = solved.final_value_circle(&circle);
    assert_nearly_eq(solved_circle.center.y, 1.5);
    assert_nearly_eq(solved_circle.radius, 1.5);
}

#[test]
fn circle_tangent_external_inferred() {
    let mut ids = IdGenerator::default();
    let circle_a = DatumCircle {
        center: DatumPoint::new(&mut ids),
        radius: DatumDistance::new(ids.next_id()),
    };
    let circle_b = DatumCircle {
        center: DatumPoint::new(&mut ids),
        radius: DatumDistance::new(ids.next_id()),
    };

    let initial_guesses = vec![
        (circle_a.center.id_x(), 0.0),
        (circle_a.center.id_y(), 0.0),
        (circle_a.radius.id, 2.0),
        (circle_b.center.id_x(), 4.0),
        (circle_b.center.id_y(), 0.0),
        (circle_b.radius.id, 3.0),
    ];
    let constraints = [
        ConstraintRequest::highest_priority(Constraint::Fixed(circle_a.radius.id, 2.0)),
        ConstraintRequest::highest_priority(Constraint::Fixed(circle_b.radius.id, 3.0)),
        ConstraintRequest::highest_priority(Constraint::CircleTangentToCircle(
            circle_a,
            circle_b,
            CircleSide::Undefined,
        )),
    ];

    let outcome = solve(&constraints, initial_guesses, Config::default()).unwrap();
    assert!(outcome.is_satisfied());
    let center_a = outcome.final_value_point(&circle_a.center);
    let center_b = outcome.final_value_point(&circle_b.center);
    assert_nearly_eq(center_a.euclidean_distance(center_b), 5.0);
}

#[test]
fn circle_tangent_internal_inferred() {
    let mut ids = IdGenerator::default();
    let circle_a = DatumCircle {
        center: DatumPoint::new(&mut ids),
        radius: DatumDistance::new(ids.next_id()),
    };
    let circle_b = DatumCircle {
        center: DatumPoint::new(&mut ids),
        radius: DatumDistance::new(ids.next_id()),
    };

    let initial_guesses = vec![
        (circle_a.center.id_x(), 0.0),
        (circle_a.center.id_y(), 0.0),
        (circle_a.radius.id, 5.0),
        (circle_b.center.id_x(), 1.0),
        (circle_b.center.id_y(), 0.0),
        (circle_b.radius.id, 2.0),
    ];
    let constraints = [
        ConstraintRequest::highest_priority(Constraint::Fixed(circle_a.radius.id, 5.0)),
        ConstraintRequest::highest_priority(Constraint::Fixed(circle_b.radius.id, 2.0)),
        ConstraintRequest::highest_priority(Constraint::CircleTangentToCircle(
            circle_a,
            circle_b,
            CircleSide::Undefined,
        )),
    ];

    let outcome = solve(&constraints, initial_guesses, Config::default()).unwrap();
    assert!(outcome.is_satisfied());
    let center_a = outcome.final_value_point(&circle_a.center);
    let center_b = outcome.final_value_point(&circle_b.center);
    assert_nearly_eq(center_a.euclidean_distance(center_b), 3.0);
}

#[test]
fn two_rectangles() {
    let solved = run("two_rectangles");
    assert!(solved.is_satisfied());
    assert!(!solved.analysis.is_underconstrained());
    // This forms two rectangles.
    assert_points_eq(solved.get_point("p0").unwrap(), Point { x: 1.0, y: 1.0 });
    assert_points_eq(solved.get_point("p1").unwrap(), Point { x: 5.0, y: 1.0 });
    assert_points_eq(solved.get_point("p2").unwrap(), Point { x: 5.0, y: 4.0 });
    assert_points_eq(solved.get_point("p3").unwrap(), Point { x: 1.0, y: 4.0 });
    // Second rectangle
    assert_points_eq(solved.get_point("p4").unwrap(), Point { x: 2.0, y: 2.0 });
    assert_points_eq(solved.get_point("p5").unwrap(), Point { x: 6.0, y: 2.0 });
    assert_points_eq(solved.get_point("p6").unwrap(), Point { x: 6.0, y: 6.0 });
    assert_points_eq(solved.get_point("p7").unwrap(), Point { x: 2.0, y: 6.0 });
}

#[test]
fn angle_constraints() {
    for file in ["angle_parallel", "angle_parallel_manual"] {
        let solved = run(file);
        assert!(solved.is_satisfied());
        assert!(!solved.analysis.is_underconstrained());
        assert_points_eq(solved.get_point("p0").unwrap(), Point { x: 0.0, y: 0.0 });
        assert_points_eq(solved.get_point("p1").unwrap(), Point { x: 4.0, y: 4.0 });
        assert_points_eq(solved.get_point("p2").unwrap(), Point { x: 0.0, y: 0.0 });
        assert_points_eq(solved.get_point("p3").unwrap(), Point { x: 4.0, y: 4.0 });
    }
}

#[test]
fn perpendicular() {
    let solved = run("perpendicular");
    assert!(solved.is_satisfied());
    assert!(!solved.analysis.is_underconstrained());
    assert_points_eq(solved.get_point("p0").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("p1").unwrap(), Point { x: 0.0, y: 4.0 });
    assert_points_eq(solved.get_point("p2").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("p3").unwrap(), Point { x: 4.0, y: 0.0 });
}

#[test]
fn nonsquare() {
    let solved = run("nonsquare");
    assert!(solved.is_satisfied());
    assert!(!solved.analysis.is_underconstrained());
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 0.0, y: 0.0 });
}

#[test]
fn square() {
    let solved = run("square");
    assert!(solved.is_satisfied());
    assert!(!solved.analysis.is_underconstrained());
    assert_nearly_eq(
        solved.get_point("a").unwrap().y - solved.get_point("c").unwrap().y,
        solved.get_point("b").unwrap().y - solved.get_point("d").unwrap().y,
    );
    assert_nearly_eq(
        solved.get_point("a").unwrap().x - solved.get_point("c").unwrap().x,
        solved.get_point("d").unwrap().x - solved.get_point("b").unwrap().x,
    );
}

#[test]
fn parallelogram() {
    let solved = run("parallelogram");
    // The paralallelogram has two vertical lines AB and CD.
    // A and B are fully determined, but C and D are free.
    assert!(solved.analysis.is_underconstrained());
    // A = 0 and 1
    // B = 2 and 3
    // CD are 4, 5, 6 and 7, and aren't constrained.
    assert_eq!(solved.analysis.underconstrained(), vec![4, 5, 6, 7]);
    assert_nearly_eq(
        solved.get_point("a").unwrap().y - solved.get_point("c").unwrap().y,
        solved.get_point("b").unwrap().y - solved.get_point("d").unwrap().y,
    );
    assert_nearly_eq(
        solved.get_point("a").unwrap().x - solved.get_point("c").unwrap().x,
        solved.get_point("b").unwrap().x - solved.get_point("d").unwrap().x,
    );
}

#[test]
fn underdetermined_lines() {
    // This should solve for a horizontal line from (0,0) to (4,0), then
    // a vertical line from (4,0) to (4,4). Note that the length of the second
    // line is not specified; we're relying on regularisation to push our solution
    // towards its start point.
    let solved = run("underdetermined_lines");
    assert!(solved.analysis.is_underconstrained());
    assert_eq!(
        solved.analysis.underconstrained(),
        vec![5],
        "P0 and P1 are constrained, but P2 is only fixed in the X direction, not Y"
    );
    assert!(solved.is_satisfied());
    assert_points_eq(solved.get_point("p0").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("p1").unwrap(), Point { x: 4.0, y: 0.0 });
    assert_points_eq(solved.get_point("p2").unwrap(), Point { x: 4.0, y: 4.0 });
}

#[test]
fn arc_radius() {
    let solved = run("arc_radius");
    assert!(solved.is_satisfied());
    assert!(solved.analysis.is_underconstrained());
    assert_eq!(
        solved.analysis.underconstrained(),
        vec![
            // P is vars 0,1, and P is totally unconstrained.
            0, 1,
            // The arc's endpoint A (2, 3) and B (4, 5) are unconstrained, they can be anywhere
            // as long as they're the right distance from the arc's center.
            // But the center (6, 7) is fully constrained.
            2, 3, 4, 5
        ],
        "Center of arc is fixed, but the other 2 points can vary."
    );
    let arc = solved.get_arc("a").unwrap();
    assert_points_eq(arc.center, Point { x: 0.0, y: 0.0 });
    assert_nearly_eq(5.0, arc.a.euclidean_distance(Default::default()));
    assert_nearly_eq(5.0, arc.b.euclidean_distance(Default::default()));
}

/// Point-Arc coincident constraint.
#[test]
fn parc_coincident() {
    let solved = run("parc_coincident");
    assert!(solved.is_satisfied());
    assert!(solved.analysis.is_underconstrained());
    let arc = solved.get_arc("a").unwrap();
    let origin = Point { x: 0.0, y: 0.0 };
    assert_points_eq(arc.center, origin);
    assert_nearly_eq(5.0, arc.a.euclidean_distance(origin));
    assert_nearly_eq(5.0, arc.b.euclidean_distance(origin));
    let point = solved.get_point("p").unwrap();
    assert_nearly_eq(5.0, arc.center.euclidean_distance(point));
}

#[test]
fn arc_equidistant() {
    let solved = run("arc_equidistant");
    assert!(solved.is_satisfied());
    assert!(solved.analysis.is_underconstrained());
    assert_eq!(
        solved.analysis.underconstrained(),
        vec![
            // P is vars 0,1, and P is totally unconstrained.
            0, 1,
            // The arc's endpoint A (2, 3) and B (4, 5) are unconstrained, they can be anywhere
            // as long as they're the right distance from the arc's center.
            // But the center (6, 7) is fully constrained.
            2, 3, 4, 5
        ],
        "Center of arc is fixed, but the other 2 points can vary."
    );
    let arc = solved.get_arc("a").unwrap();
    assert_points_eq(arc.center, Point { x: 0.0, y: 0.0 });
    assert_nearly_eq(
        arc.a.euclidean_distance(arc.center),
        arc.b.euclidean_distance(arc.center),
    );
}

#[test]
fn chamfer_square() {
    let solved = run("chamfer_square");
    assert!(solved.is_satisfied());
    assert!(!solved.analysis.is_underconstrained());
    assert_points_eq(solved.get_point("a").unwrap(), Point { x: 0.0, y: 40.0 });
    assert_points_eq(solved.get_point("b").unwrap(), Point { x: 30.0, y: 40.0 });
    assert_points_eq(solved.get_point("c").unwrap(), Point { x: 40.0, y: 30.0 });
    assert_points_eq(solved.get_point("d").unwrap(), Point { x: 40.0, y: 0.0 });
    assert_points_eq(solved.get_point("e").unwrap(), Point { x: 0.0, y: 0.0 });
}

#[test]
fn arc_length() {
    let solved = run("arc_length");
    assert!(solved.is_satisfied());
}

/// Test that mirrors the `trim_arc2_left_side` scenario from modeling-app.
///
/// This test represents a trim operation where:
/// - Two arcs intersect
/// - arc1 should remain unchanged (its parameters shouldn't change when `PointArcCoincident` is added)
/// - arc2 should be trimmed to end at the intersection point
/// - When `PointArcCoincident` is added for the intersection point on arc2, arc1's parameters should remain unchanged
///
/// This test mirrors the KCL code from the modeling-app test:
/// ```
/// sketch(on = YZ) {
///   arc1 = sketch2::arc(start = [var 0mm, var 5mm], end = [var 0mm, var -5mm], center = [var 30mm, var 0mm])
///   arc2 = sketch2::arc(start = [var 5mm, var 0mm], end = [var -5mm, var 0mm], center = [var 0mm, var -30mm])
/// }
/// ```
#[test]
fn test_trim_arc2_left_side_arc1_should_remain_fixed() {
    let mut ids = IdGenerator::default();

    // Create two arcs matching the trim test scenario:
    // arc1: start = (0, 5), end = (0, -5), center = (30, 0)
    // arc2: start = (5, 0), end = (-5, 0), center = (0, -30)
    let arc1_center = DatumPoint::new(&mut ids);
    let arc1_start = DatumPoint::new(&mut ids);
    let arc1_end = DatumPoint::new(&mut ids);
    let arc1 = DatumCircularArc {
        center: arc1_center,
        start: arc1_start,
        end: arc1_end,
    };

    let arc2_center = DatumPoint::new(&mut ids);
    let arc2_start = DatumPoint::new(&mut ids);
    let arc2_end = DatumPoint::new(&mut ids);
    let arc2 = DatumCircularArc {
        center: arc2_center,
        start: arc2_start,
        end: arc2_end,
    };

    // Expected values for arc1 (matching the modeling-app test)
    // These should remain unchanged when PointArcCoincident is added
    let arc1_center_x_expected = 30.0;
    let arc1_center_y_expected = 0.0;
    let arc1_start_x_expected = 0.0;
    let arc1_start_y_expected = 5.0;
    let arc1_end_x_expected = 0.0;
    let arc1_end_y_expected = -5.0;

    // Initial guesses matching the trim test scenario
    let initial_guesses = vec![
        // arc1: start = (0, 5), end = (0, -5), center = (30, 0)
        (arc1_center.id_x(), 30.0),
        (arc1_center.id_y(), 0.0),
        (arc1_start.id_x(), 0.0),
        (arc1_start.id_y(), 5.0),
        (arc1_end.id_x(), 0.0),
        (arc1_end.id_y(), -5.0),
        // arc2: start = (5, 0), end = (-5, 0), center = (0, -30)
        (arc2_center.id_x(), 0.0),
        (arc2_center.id_y(), -30.0),
        (arc2_start.id_x(), 5.0),
        (arc2_start.id_y(), 0.0),
        // arc2.end will become the intersection point after trimming
        (arc2_end.id_x(), -5.0),
        (arc2_end.id_y(), 0.0),
    ];

    // Now solve with PointArcCoincident added (this is what the trim operation does)
    // The bug is that adding PointArcCoincident causes arc1's parameters to change
    // even though they should remain at the expected values
    let constraints_with_trim = vec![
        // Define both arcs
        ConstraintRequest::highest_priority(Constraint::Arc(arc1)),
        ConstraintRequest::highest_priority(Constraint::Arc(arc2)),
        // Fix arc1's parameters to the expected values (matching the modeling-app test)
        // These should NOT change when PointArcCoincident is added
        ConstraintRequest::highest_priority(Constraint::Fixed(
            arc1_center.id_x(),
            arc1_center_x_expected,
        )),
        ConstraintRequest::highest_priority(Constraint::Fixed(
            arc1_center.id_y(),
            arc1_center_y_expected,
        )),
        ConstraintRequest::highest_priority(Constraint::Fixed(
            arc1_start.id_x(),
            arc1_start_x_expected,
        )),
        ConstraintRequest::highest_priority(Constraint::Fixed(
            arc1_start.id_y(),
            arc1_start_y_expected,
        )),
        ConstraintRequest::highest_priority(Constraint::Fixed(
            arc1_end.id_x(),
            arc1_end_x_expected,
        )),
        ConstraintRequest::highest_priority(Constraint::Fixed(
            arc1_end.id_y(),
            arc1_end_y_expected,
        )),
        // Fix arc2's start and center
        ConstraintRequest::highest_priority(Constraint::Fixed(arc2_center.id_x(), 0.0)),
        ConstraintRequest::highest_priority(Constraint::Fixed(arc2_center.id_y(), -30.0)),
        ConstraintRequest::highest_priority(Constraint::Fixed(arc2_start.id_x(), 5.0)),
        ConstraintRequest::highest_priority(Constraint::Fixed(arc2_start.id_y(), 0.0)),
        // The intersection point (arc2.end) should be on arc2 (this is what causes the bug)
        // In the trim scenario, arc2.end becomes the intersection point
        ConstraintRequest::highest_priority(Constraint::PointArcCoincident(arc2, arc2_end)),
        // Also add that arc2.end is on arc1 (matching the coincident constraint
        // in the modeling-app test: sketch2::coincident([arc2.end, arc1]))
        ConstraintRequest::highest_priority(Constraint::PointArcCoincident(arc1, arc2_end)),
    ];

    let trim_outcome = solve(&constraints_with_trim, initial_guesses, Config::default())
        .expect("trim constraint system should converge");

    assert!(
        trim_outcome.is_satisfied(),
        "the constraint should be satisfied"
    );

    // Verify that arc1's parameters remain at their fixed values
    // The bug on kurt-point-on-arc is that these fixed values are violated
    // when PointArcCoincident is added, even though they're explicitly fixed
    assert_nearly_eq(
        trim_outcome.final_values[arc1_center.id_x() as usize],
        arc1_center_x_expected,
    );
    assert_nearly_eq(
        trim_outcome.final_values[arc1_center.id_y() as usize],
        arc1_center_y_expected,
    );
    assert_nearly_eq(
        trim_outcome.final_values[arc1_start.id_x() as usize],
        arc1_start_x_expected,
    );
    assert_nearly_eq(
        trim_outcome.final_values[arc1_start.id_y() as usize],
        arc1_start_y_expected,
    );
    assert_nearly_eq(
        trim_outcome.final_values[arc1_end.id_x() as usize],
        arc1_end_x_expected,
    );
    assert_nearly_eq(
        trim_outcome.final_values[arc1_end.id_y() as usize],
        arc1_end_y_expected,
    );
}

fn solve_arc_length_case(
    arc_center_x: f64,
    arc_center_y: f64,
    arc_radius: f64,
    arc_start_radians: f64,
    desired_arc_length: f64,
    arc_end_guess: Point,
) -> (SolveOutcome, DatumCircularArc) {
    let mut ids = IdGenerator::default();
    let center = DatumPoint::new(&mut ids);
    let start = DatumPoint::new(&mut ids);
    let end = DatumPoint::new(&mut ids);
    let arc = DatumCircularArc { center, start, end };

    let arc_start = Point {
        x: arc_center_x + libm::cos(arc_start_radians) * arc_radius,
        y: arc_center_y + libm::sin(arc_start_radians) * arc_radius,
    };

    let initial_guesses = vec![
        (arc.center.id_x(), arc_center_x),
        (arc.center.id_y(), arc_center_y),
        (arc.start.id_x(), arc_start.x),
        (arc.start.id_y(), arc_start.y),
        (arc.end.id_x(), arc_end_guess.x),
        (arc.end.id_y(), arc_end_guess.y),
    ];

    let requests: Vec<_> = vec![
        Constraint::Arc(arc),
        Constraint::Fixed(arc.center.id_x(), arc_center_x),
        Constraint::Fixed(arc.center.id_y(), arc_center_y),
        Constraint::Fixed(arc.start.id_x(), arc_start.x),
        Constraint::Fixed(arc.start.id_y(), arc_start.y),
        Constraint::ArcLength(arc, desired_arc_length),
    ]
    .into_iter()
    .map(ConstraintRequest::highest_priority)
    .collect();

    let outcome =
        solve(&requests, initial_guesses, Config::default()).expect("arc length case should solve");

    (outcome, arc)
}

#[test]
fn arc_length_ccw_over_pi() {
    let arc_center_x = 0.0;
    let arc_center_y = 0.0;
    let arc_radius = 1.0;
    let arc_start_radians = 0.0;
    let desired_arc_length = 1.5 * PI;
    let arc_end_guess = Point { x: 0.0, y: -1.0 };

    let (outcome, arc) = solve_arc_length_case(
        arc_center_x,
        arc_center_y,
        arc_radius,
        arc_start_radians,
        desired_arc_length,
        arc_end_guess,
    );

    assert!(outcome.is_satisfied());

    let solved_end_x = outcome.final_values[arc.end.id_x() as usize];
    let solved_end_y = outcome.final_values[arc.end.id_y() as usize];
    let solved_end = Point {
        x: solved_end_x,
        y: solved_end_y,
    };
    let center_point = Point {
        x: arc_center_x,
        y: arc_center_y,
    };
    let end_distance = solved_end.euclidean_distance(center_point);
    assert_nearly_eq(end_distance, arc_radius);

    let two_pi = 2.0 * PI;
    let end_radians =
        libm::atan2(solved_end_y - arc_center_y, solved_end_x - arc_center_x).rem_euclid(two_pi);
    let ccw_delta = (end_radians - arc_start_radians).rem_euclid(two_pi);
    let actual_arc_length = arc_radius * ccw_delta;
    assert_nearly_eq(actual_arc_length, desired_arc_length);
}

#[test]
fn arc_length_near_zero() {
    let arc_center_x = -2.0;
    let arc_center_y = 3.0;
    let arc_radius = 5.0;
    let arc_start_radians = 0.25 * PI;
    let desired_arc_length = 1.0e-3;
    let arc_end_guess = Point {
        x: arc_center_x + libm::cos(arc_start_radians + 1.0e-2) * arc_radius,
        y: arc_center_y + libm::sin(arc_start_radians + 1.0e-2) * arc_radius,
    };

    let (outcome, arc) = solve_arc_length_case(
        arc_center_x,
        arc_center_y,
        arc_radius,
        arc_start_radians,
        desired_arc_length,
        arc_end_guess,
    );

    assert!(outcome.is_satisfied());

    let solved_end_x = outcome.final_values[arc.end.id_x() as usize];
    let solved_end_y = outcome.final_values[arc.end.id_y() as usize];
    let end_radians =
        libm::atan2(solved_end_y - arc_center_y, solved_end_x - arc_center_x).rem_euclid(2.0 * PI);
    let ccw_delta = (end_radians - arc_start_radians).rem_euclid(2.0 * PI);
    let actual_arc_length = arc_radius * ccw_delta;
    assert_nearly_eq(actual_arc_length, desired_arc_length);
}

#[test]
fn arc_length_near_full_circle() {
    let arc_center_x = 1.0;
    let arc_center_y = -1.0;
    let arc_radius = 2.5;
    let arc_start_radians = 0.0;
    let desired_arc_length = 2.0 * PI * arc_radius - 1.0e-3;
    let arc_end_guess = Point {
        x: arc_center_x + libm::cos(-1.0e-2) * arc_radius,
        y: arc_center_y + libm::sin(-1.0e-2) * arc_radius,
    };

    let (outcome, arc) = solve_arc_length_case(
        arc_center_x,
        arc_center_y,
        arc_radius,
        arc_start_radians,
        desired_arc_length,
        arc_end_guess,
    );

    assert!(outcome.is_satisfied());

    let solved_end_x = outcome.final_values[arc.end.id_x() as usize];
    let solved_end_y = outcome.final_values[arc.end.id_y() as usize];
    let end_radians =
        libm::atan2(solved_end_y - arc_center_y, solved_end_x - arc_center_x).rem_euclid(2.0 * PI);
    let ccw_delta = (end_radians - arc_start_radians).rem_euclid(2.0 * PI);
    let actual_arc_length = arc_radius * ccw_delta;
    assert_nearly_eq(actual_arc_length, desired_arc_length);
}

#[test]
fn arc_length_degenerate_warns() {
    let mut ids = IdGenerator::default();
    let center = DatumPoint::new(&mut ids);
    let start = DatumPoint::new(&mut ids);
    let end = DatumPoint::new(&mut ids);
    let arc = DatumCircularArc { center, start, end };

    let initial_guesses = vec![
        (arc.center.id_x(), 0.0),
        (arc.center.id_y(), 0.0),
        (arc.start.id_x(), 0.0),
        (arc.start.id_y(), 0.0),
        (arc.end.id_x(), 1.0),
        (arc.end.id_y(), 0.0),
    ];

    let requests: Vec<_> = vec![
        Constraint::Fixed(arc.center.id_x(), 0.0),
        Constraint::Fixed(arc.center.id_y(), 0.0),
        Constraint::Fixed(arc.start.id_x(), 0.0),
        Constraint::Fixed(arc.start.id_y(), 0.0),
        Constraint::ArcLength(arc, 1.0),
    ]
    .into_iter()
    .map(ConstraintRequest::highest_priority)
    .collect();

    let outcome = solve(&requests, initial_guesses, Config::default())
        .expect("degenerate arc length case should solve");

    assert!(
        outcome
            .warnings
            .iter()
            .any(|warning| matches!(warning.content, WarningContent::Degenerate))
    );
}

#[test]
fn strange_nonconvergence() {
    use crate::datatypes::inputs::DatumPoint;
    let p = DatumPoint { x_id: 0, y_id: 1 };
    let q = DatumPoint { x_id: 2, y_id: 3 };
    let r = DatumPoint { x_id: 4, y_id: 5 };
    let s = DatumPoint { x_id: 6, y_id: 7 };
    let t = DatumPoint { x_id: 8, y_id: 9 };

    let requests = [
        ConstraintRequest::highest_priority(Constraint::Fixed(0, 0.0)),
        ConstraintRequest::highest_priority(Constraint::Fixed(1, 0.0)),
        ConstraintRequest::highest_priority(Constraint::PointsCoincident(r, s)),
        ConstraintRequest::highest_priority(Constraint::PointsCoincident(q, p)),
        ConstraintRequest::highest_priority(Constraint::LinesEqualLength(
            DatumLineSegment { p0: q, p1: r },
            DatumLineSegment { p0: s, p1: t },
        )),
    ];
    let initial_guesses = vec![
        (0, 0.0),
        (1, -0.02),
        (2, -3.39),
        (3, -0.38),
        (4, -2.76),
        (5, 4.83),
        (6, -1.54),
        (7, 5.21),
        (8, -1.15),
        (9, 2.75),
    ];
    let outcome = solve(
        &requests,
        initial_guesses,
        Config::default().with_max_iterations(31),
    );
    let iterations = outcome.unwrap().iterations;
    assert_eq!(iterations, 2);
}

#[test]
fn warnings() {
    let txt = "# constraints
point p
point q
p.x = 0
p.y = 0
q.y = 0
vertical(p, q)
point r
point s
r.x = 0
s.x = 0
s.y = 0
lines_at_angle(p, q, r, s, 0rad)

# guesses
p roughly (3, 4)
q roughly (5, 6)
r roughly (3, 4)
s roughly (5, 6)
";
    let problem = Problem::from_str(txt).unwrap();
    let solved = problem.to_constraint_system().unwrap().solve().unwrap();
    assert!(!solved.warnings.is_empty());
    assert!(solved.warnings.contains(&Warning {
        about_constraint: Some(7),
        content: WarningContent::ShouldBeParallel(Angle::from_radians(0.0))
    }));
}

#[track_caller]
fn assert_points_eq(l: Point, r: Point) {
    let dist = l.euclidean_distance(r);
    assert!(dist < EPSILON, "LHS was {l}, RHS was {r}, dist was {dist}");
}

#[track_caller]
pub fn assert_nearly_eq(l: f64, r: f64) {
    let diff = (l - r).abs();
    assert!(
        diff < EPSILON,
        "LHS was {l}, RHS was {r}, difference was {diff}"
    );
}

#[track_caller]
fn assert_point_on_arc_ccw(point: Point, center: Point, start: Point, end: Point) {
    let radius = start.euclidean_distance(center);
    assert_nearly_eq(point.euclidean_distance(center), radius);

    let s = V {
        x: start.x - center.x,
        y: start.y - center.y,
    };
    let e = V {
        x: end.x - center.x,
        y: end.y - center.y,
    };
    let p = V {
        x: point.x - center.x,
        y: point.y - center.y,
    };

    let two_pi = 2.0 * PI;
    let a_sp = s.signed_angle(p).rem_euclid(two_pi);
    let a_se = s.signed_angle(e).rem_euclid(two_pi);

    let a_sp = if a_sp > two_pi - EPSILON { 0.0 } else { a_sp };
    assert!(
        a_sp <= a_se + EPSILON,
        "point {point} is not on the CCW arc from {start} to {end} around {center}"
    );
}

fn solve_point_arc_coincident_with_fixed_arc(
    center_point: Point,
    start_point: Point,
    end_point: Point,
    initial_point: Point,
) -> Point {
    let mut ids = IdGenerator::default();
    let center = DatumPoint::new(&mut ids);
    let start = DatumPoint::new(&mut ids);
    let end = DatumPoint::new(&mut ids);
    let point = DatumPoint::new(&mut ids);
    let arc = DatumCircularArc { center, start, end };

    let constraints = vec![
        ConstraintRequest::highest_priority(Constraint::PointArcCoincident(arc, point)),
        ConstraintRequest::highest_priority(Constraint::Fixed(center.id_x(), center_point.x)),
        ConstraintRequest::highest_priority(Constraint::Fixed(center.id_y(), center_point.y)),
        ConstraintRequest::highest_priority(Constraint::Fixed(start.id_x(), start_point.x)),
        ConstraintRequest::highest_priority(Constraint::Fixed(start.id_y(), start_point.y)),
        ConstraintRequest::highest_priority(Constraint::Fixed(end.id_x(), end_point.x)),
        ConstraintRequest::highest_priority(Constraint::Fixed(end.id_y(), end_point.y)),
    ];
    let initial_guesses = vec![
        (center.id_x(), center_point.x),
        (center.id_y(), center_point.y),
        (start.id_x(), start_point.x),
        (start.id_y(), start_point.y),
        (end.id_x(), end_point.x),
        (end.id_y(), end_point.y),
        (point.id_x(), initial_point.x),
        (point.id_y(), initial_point.y),
    ];

    let outcome = solve(&constraints, initial_guesses, Config::default())
        .expect("fixed arc point-arc coincident case should solve");
    assert!(outcome.is_satisfied(), "constraint should be satisfied");

    Point {
        x: outcome.final_values[point.id_x() as usize],
        y: outcome.final_values[point.id_y() as usize],
    }
}

/// Regression test for old point-on-arc behavior where the angular residuals were zeroed out once
/// the point was already on the circle containing the arc.
///
/// In this setup the initial point starts on the containing circle but outside the arc's angular
/// interval. The previous implementation could therefore converge immediately without moving the
/// point onto the arc itself.
#[test]
fn point_arc_coincident_old_incorrect_convergence_1() {
    let center = Point { x: 0.0, y: 0.0 };
    let start = Point { x: 1.0, y: 0.0 };
    let end = Point { x: 0.0, y: 1.0 };
    let solved_point =
        solve_point_arc_coincident_with_fixed_arc(center, start, end, Point { x: 0.0, y: -1.0 });
    assert_point_on_arc_ccw(solved_point, center, start, end);
    assert_points_eq(solved_point, start);
}

/// Regression test for old point-on-arc behavior where competing angular residuals could cancel
/// each other out.
///
/// The initial point starts well away from the arc in a configuration where the previous residual
/// formulation could settle onto the containing circle while still leaving the point outside the
/// arc interval.
#[test]
fn point_arc_coincident_old_incorrect_convergence_2() {
    let center = Point { x: 0.0, y: 0.0 };
    let start = Point { x: 1.0, y: 0.0 };
    let end = Point { x: 0.0, y: 1.0 };
    let solved_point =
        solve_point_arc_coincident_with_fixed_arc(center, start, end, Point { x: -3.0, y: -3.0 });
    assert_point_on_arc_ccw(solved_point, center, start, end);
    assert_points_eq(solved_point, start);
}

/// Regression test for the bug that motivated this fix.
///
/// The bug report was: adding `point_arc_coincident(point, arc)` can cause the solver to
/// "jump" to a very different configuration even when the point is already basically on the arc.
///
/// This test reproduces that bug by:
/// - Solving a baseline problem without the `point_arc_coincident` constraint
/// - Solving the same problem with the constraint added
/// - Verifying that the initial guess is already close to the arc (distance-from-radius < 0.5)
/// - Asserting that adding the constraint doesn't cause dramatic changes
///
/// This test encodes the stability goal: "don't move much if already almost satisfied".
#[test]
fn point_basically_already_on_arc_should_not_cause_much_change_in_sketch() {
    // First, solve without the point_arc_coincident constraint to get a baseline
    let txt_without = std::fs::read_to_string(
        "../test_cases/arc_line_coincident_bug/problem_without_arc_constraint.md",
    )
    .unwrap();
    let problem_without = parse_problem(&txt_without);
    let system_without = problem_without.to_constraint_system().unwrap();
    let solved_without = system_without
        .solve_with_config_analysis(Default::default())
        .unwrap();

    // Now solve with the point_arc_coincident constraint
    let solved_with = run("arc_line_coincident_bug");

    // Initial guesses
    let initial_line3_start = Point { x: 4.32, y: 3.72 };
    let initial_line3_end = Point { x: 1.06, y: -3.26 };
    let initial_line4_start = Point { x: -2.32, y: -2.96 };
    let initial_line4_end = Point { x: -7.01, y: -2.77 };
    let initial_arc_center = Point { x: 1.06, y: -3.26 };
    let initial_arc_a = Point { x: -1.44, y: -0.99 };
    let initial_arc_b = Point { x: 2.49, y: -0.2 };

    // Get the solved values
    let solved_line3_start = solved_with.get_point("line3start").unwrap();
    let solved_line3_end = solved_with.get_point("line3end").unwrap();
    let solved_line4_start = solved_with.get_point("line4start").unwrap();
    let solved_line4_end = solved_with.get_point("line4end").unwrap();
    let solved_arc = solved_with.get_arc("arc1").unwrap();

    // Calculate how far line4_start is from the arc in the initial guess
    // The arc center is at (1.06, -3.26) and arc.a is at (-1.44, -0.99)
    // So the radius is the distance from center to arc.a
    let initial_arc_radius = initial_arc_center.euclidean_distance(initial_arc_a);
    let initial_line4_start_to_center = initial_line4_start.euclidean_distance(initial_arc_center);
    let initial_distance_from_arc = (initial_line4_start_to_center - initial_arc_radius).abs();

    // Verify that line4_start is already very close to the arc (within a reasonable tolerance)
    // This should be a small value, showing the point is already basically on the arc
    assert!(
        initial_distance_from_arc < 0.5,
        "line4_start should be close to the arc initially. Distance from arc: {}",
        initial_distance_from_arc
    );

    // Calculate how much the solution changed from the initial guesses
    let _change_line3_start = solved_line3_start.euclidean_distance(initial_line3_start);
    let _change_line3_end = solved_line3_end.euclidean_distance(initial_line3_end);
    let change_line4_start = solved_line4_start.euclidean_distance(initial_line4_start);
    let _change_line4_end = solved_line4_end.euclidean_distance(initial_line4_end);
    let _change_arc_center = solved_arc.center.euclidean_distance(initial_arc_center);
    let _change_arc_a = solved_arc.a.euclidean_distance(initial_arc_a);
    let _change_arc_b = solved_arc.b.euclidean_distance(initial_arc_b);

    // The bug is that these changes are dramatically large even though line4_start
    // is already basically on the arc. We expect the solver to make minimal changes.
    // Debug logs intentionally removed to keep tests quiet by default.

    // Compare with the solution without the constraint
    let solved_without_line3_start = solved_without.get_point("line3start").unwrap();
    let solved_without_line3_end = solved_without.get_point("line3end").unwrap();
    let solved_without_line4_start = solved_without.get_point("line4start").unwrap();
    let solved_without_line4_end = solved_without.get_point("line4end").unwrap();
    let solved_without_arc = solved_without.get_arc("arc1").unwrap();

    let _diff_line3_start = solved_line3_start.euclidean_distance(solved_without_line3_start);
    let _diff_line3_end = solved_line3_end.euclidean_distance(solved_without_line3_end);
    let _diff_line4_start = solved_line4_start.euclidean_distance(solved_without_line4_start);
    let _diff_line4_end = solved_line4_end.euclidean_distance(solved_without_line4_end);
    let _diff_arc_center = solved_arc
        .center
        .euclidean_distance(solved_without_arc.center);
    let _diff_arc_a = solved_arc.a.euclidean_distance(solved_without_arc.a);
    let _diff_arc_b = solved_arc.b.euclidean_distance(solved_without_arc.b);

    // Debug logs intentionally removed to keep tests quiet by default.

    // The test demonstrates the bug: adding the constraint causes dramatic changes
    // This assertion will fail if the bug is present, showing the dramatic difference
    // We use a threshold that's much larger than the initial distance from the arc
    let max_expected_change = initial_distance_from_arc * 10.0;
    assert!(
        change_line4_start <= max_expected_change,
        "BUG REPRODUCED: Adding point_arc_coincident constraint caused line4_start to move by {:.6}, \
         but it was only {:.6} away from the arc initially. This is a dramatic change that shouldn't be necessary.",
        change_line4_start,
        initial_distance_from_arc
    );
}

/// Test the "other side" of the stability goal: when a point is NOT on the circle or not in range,
/// the constraint SHOULD move it meaningfully.
///
/// This test verifies that:
/// - If the point is initially outside the angular range (e.g., near the arc center or outside
///   the angular span), the constraint should cause it to move significantly onto the arc
/// - The point ends up both on the circle (correct radius) and within the angular range
/// - If the initial angular violation is large, movement should be at least ~30% of the radius
///
/// This test encodes the correctness goal: "do move meaningfully if far from satisfied".
/// Together with `point_basically_already_on_arc_should_not_cause_much_change_in_sketch`,
/// these tests ensure the constraint is both stable (doesn't over-correct) and effective
/// (does correct when needed).
#[test]
fn arc_center_point_coincident() {
    let solved = run("arc_center_point_coincident");

    // Initial guesses from the problem file
    let initial_line4_start = Point { x: -1.16, y: -2.63 };
    let initial_arc_center = Point { x: 0.55, y: -3.31 };

    // Get the solved values
    let solved_line4_start = solved.get_point("line4start").unwrap();
    let solved_arc = solved.get_arc("arc1").unwrap();

    // Check initial angular position relative to the arc
    let initial_arc_a = Point { x: 2.25, y: -3.99 };
    let initial_arc_b = Point { x: 1.43, y: -1.71 };

    // Calculate cross products to check if point is in angular range initially
    let cx = initial_arc_center.x;
    let cy = initial_arc_center.y;
    let ax = initial_arc_a.x;
    let ay = initial_arc_a.y;
    let bx = initial_arc_b.x;
    let by = initial_arc_b.y;
    let px = initial_line4_start.x;
    let py = initial_line4_start.y;

    let initial_start_cross = (ax - cx) * (cy - py) - (ay - cy) * (cx - px);
    let initial_end_cross = (bx - cx) * (cy - py) - (by - cy) * (cx - px);

    // Debug logs intentionally removed to keep tests quiet by default.

    // The point should initially NOT be in the angular range
    // (either start_cross > 0 or end_cross >= 0)
    let initially_in_range = initial_start_cross <= 0.0 && initial_end_cross < 0.0;
    assert!(
        !initially_in_range,
        "line4_start should initially NOT be in the angular range. start_cross: {}, end_cross: {}",
        initial_start_cross, initial_end_cross
    );

    // Calculate how much the point moved
    let movement = solved_line4_start.euclidean_distance(initial_line4_start);
    // Debug logs intentionally removed to keep tests quiet by default.

    // Verify the point is now on the arc (at the correct radius)
    let arc_radius = solved_arc.center.euclidean_distance(solved_arc.a);
    let point_to_center_dist = solved_line4_start.euclidean_distance(solved_arc.center);
    let distance_from_arc = (point_to_center_dist - arc_radius).abs();
    // Debug logs intentionally removed to keep tests quiet by default.

    // The point should be on the arc (within tolerance)
    assert!(
        distance_from_arc < 0.01,
        "line4_start should be on the arc after solving. Distance from arc: {}, arc radius: {}",
        distance_from_arc,
        arc_radius
    );

    // The point should have moved to get into the angular range.
    // If it was only slightly outside (start_cross small), movement might be small.
    // But if it was far outside, it should move significantly.
    // For now, just verify it ends up in the angular range (checked below).
    // If the initial violation was large, we expect significant movement.
    if initial_start_cross > 0.1 {
        let min_expected_movement = arc_radius * 0.3; // At least 30% of the radius for large violations
        assert!(
            movement > min_expected_movement,
            "line4_start should have moved significantly when initially far outside angular range. \
             Movement: {}, minimum expected: {} (30% of arc radius {}), initial start_cross: {}",
            movement,
            min_expected_movement,
            arc_radius,
            initial_start_cross
        );
    }

    // Also verify the point is within the angular range by checking cross products
    let cx = solved_arc.center.x;
    let cy = solved_arc.center.y;
    let ax = solved_arc.a.x;
    let ay = solved_arc.a.y;
    let bx = solved_arc.b.x;
    let by = solved_arc.b.y;
    let px = solved_line4_start.x;
    let py = solved_line4_start.y;

    // For a CCW arc, the point should be:
    // - CCW from the start vector (start_cross < 0)
    // - Before the end (end is CCW from point, so end_cross < 0)
    let start_cross = (ax - cx) * (cy - py) - (ay - cy) * (cx - px);
    let end_cross = (bx - cx) * (cy - py) - (by - cy) * (cx - px);

    // Debug logs intentionally removed to keep tests quiet by default.

    // Allow small tolerance for numerical precision, but point should be clearly in range
    assert!(
        start_cross < 0.01,
        "Point should be CCW from start angle (or very close to boundary). start_cross: {}",
        start_cross
    );
    assert!(
        end_cross < 1e-6,
        "Point should be before end angle (end is CCW from point). end_cross: {}",
        end_cross
    );
}

#[test]
fn lines_at_angle_isolated() {
    use crate::datatypes::inputs::{DatumLineSegment, DatumPoint};

    struct TestCase {
        points: [[f64; 2]; 4],
        angle: f64,
        expected_iters: usize,
    }

    let test_cases = [
        TestCase {
            points: [[0.0, 0.0], [1.0, 0.0], [0.0, 0.0], [0.0, 2.0]],
            angle: 0.5 * PI,
            expected_iters: 0,
        },
        TestCase {
            points: [[0.0, 0.0], [1.0, 0.0], [0.0, 0.0], [0.0, 2.0]],
            angle: -0.5 * PI,
            expected_iters: 0,
        },
        TestCase {
            points: [[0.0, 0.0], [1.0, 0.0], [0.0, 0.0], [2.0, 0.0]],
            angle: 0.0,
            expected_iters: 0,
        },
        TestCase {
            points: [[0.0, 0.0], [1.0, 0.0], [0.0, 0.0], [2.0, 0.0]],
            angle: PI,
            expected_iters: 0,
        },
        TestCase {
            points: [[0.0, 0.0], [-1.0, 0.0], [0.0, 0.0], [2.0, 0.0]],
            angle: 0.0,
            expected_iters: 0,
        },
        TestCase {
            points: [[0.0, 0.0], [-1.0, 0.0], [0.0, 0.0], [2.0, 0.0]],
            angle: PI,
            expected_iters: 0,
        },
        TestCase {
            points: [[0.0, 0.0], [1.0, 0.0], [0.0, 0.0], [0.0, 2.0]],
            angle: 0.0,
            expected_iters: 3,
        },
        TestCase {
            points: [[0.0, 0.0], [1.0, 0.0], [0.0, 0.0], [0.0, 2.0]],
            angle: PI,
            expected_iters: 3,
        },
        TestCase {
            points: [[0.0, 0.0], [0.0, 1.0], [0.0, 0.0], [0.0, 2.0]],
            angle: 0.5 * PI,
            expected_iters: 3,
        },
        TestCase {
            points: [[0.0, 0.0], [0.0, 1.0], [0.0, 0.0], [0.0, 2.0]],
            angle: -0.5 * PI,
            expected_iters: 3,
        },
    ];

    let p0 = DatumPoint { x_id: 0, y_id: 1 };
    let p1 = DatumPoint { x_id: 2, y_id: 3 };
    let p2 = DatumPoint { x_id: 4, y_id: 5 };
    let p3 = DatumPoint { x_id: 6, y_id: 7 };
    let line0 = DatumLineSegment { p0, p1 };
    let line1 = DatumLineSegment { p0: p2, p1: p3 };

    for test_case in test_cases {
        let constraints = [ConstraintRequest::highest_priority(
            Constraint::LinesAtAngle(
                line0,
                line1,
                AngleKind::Other(Angle::from_radians(test_case.angle)),
            ),
        )];
        let initial_guesses: Vec<_> = test_case
            .points
            .into_iter()
            .enumerate()
            .flat_map(|(point_idx, [x, y])| {
                [((point_idx as Id) * 2, x), ((point_idx as Id) * 2 + 1, y)]
            })
            .collect();

        let outcome = solve(
            &constraints,
            initial_guesses,
            Config::default().with_max_iterations(100),
        )
        .unwrap_or_else(|err| panic!("failed for angle {}: {err:?}", test_case.angle));

        assert!(outcome.is_satisfied());
        assert_eq!(
            outcome.iterations(),
            test_case.expected_iters,
            "unexpected iteration count for angle {}",
            test_case.angle
        );
    }
}

#[test]
fn lines_angle_sign_check() {
    use crate::datatypes::inputs::{DatumLineSegment, DatumPoint};

    struct TestCase {
        vars: [[f64; 2]; 3],
        angle: f64,
        expected_iters: usize,
    }

    let test_cases = [
        TestCase {
            vars: [[0.0, 0.0], [1.0, 0.0], [2.0, 1.0]],
            angle: 0.1 * PI,
            expected_iters: 1,
        },
        TestCase {
            vars: [[0.0, 0.0], [1.0, 0.0], [2.0, 1.0]],
            angle: -0.1 * PI,
            expected_iters: 1,
        },
    ];

    let p0 = DatumPoint { x_id: 0, y_id: 1 };
    let p1 = DatumPoint { x_id: 2, y_id: 3 };
    let p2 = DatumPoint { x_id: 4, y_id: 5 };
    let line0 = DatumLineSegment { p0, p1 };
    let line1 = DatumLineSegment { p0: p1, p1: p2 };

    for test_case in test_cases {
        let constraints = [
            ConstraintRequest::highest_priority(Constraint::Fixed(p0.id_x(), 0.0)),
            ConstraintRequest::highest_priority(Constraint::Fixed(p0.id_y(), 0.0)),
            ConstraintRequest::highest_priority(Constraint::Fixed(p1.id_x(), 1.0)),
            ConstraintRequest::highest_priority(Constraint::Fixed(p1.id_y(), 0.0)),
            ConstraintRequest::highest_priority(Constraint::LinesAtAngle(
                line0,
                line1,
                AngleKind::Other(Angle::from_radians(test_case.angle)),
            )),
        ];
        let initial_guesses: Vec<_> = test_case
            .vars
            .into_iter()
            .enumerate()
            .flat_map(|(point_idx, [x, y])| {
                [((point_idx as Id) * 2, x), ((point_idx as Id) * 2 + 1, y)]
            })
            .collect();

        let outcome = solve(
            &constraints,
            initial_guesses,
            Config::default().with_max_iterations(100),
        )
        .unwrap_or_else(|err| panic!("failed for angle {}: {err:?}", test_case.angle));

        assert!(outcome.is_satisfied());
        assert_eq!(
            outcome.iterations(),
            test_case.expected_iters,
            "unexpected iteration count for angle {}",
            test_case.angle
        );

        // Ensure the resulting angle actually matches
        {
            let p0 = outcome.final_value_point(&p0);
            let p1 = outcome.final_value_point(&p1);
            let p2 = outcome.final_value_point(&p2);
            let u = V::new(p1.x - p0.x, p1.y - p0.y);
            let v = V::new(p2.x - p1.x, p2.y - p1.y);
            assert_nearly_eq(u.signed_angle(v), test_case.angle);
        }
    }
}

/// Returns the signed angle from (p1 - p0) to (p2 - p0), reading variable
/// ids 0..5 = `[p0_x, p0_y, p1_x, p1_y, p2_x, p2_y]` from `vals`.
fn points_at_angle_from_vals(vals: &[f64]) -> f64 {
    let u = V::new(vals[2] - vals[0], vals[3] - vals[1]);
    let v = V::new(vals[4] - vals[0], vals[5] - vals[1]);
    u.signed_angle(v)
}

#[test]
fn points_at_angle_already_satisfied() {
    // Cases where the geometry already satisfies the constraint: expect 0 Newton iterations.
    struct TestCase {
        p1: [f64; 2], // first arm endpoint; vertex is always at origin
        p2: [f64; 2], // second arm endpoint
        angle: f64,
    }

    let test_cases = [
        TestCase {
            p1: [1.0, 0.0],
            p2: [0.0, 2.0],
            angle: 0.5 * PI,
        },
        TestCase {
            p1: [1.0, 0.0],
            p2: [0.0, -2.0],
            angle: -0.5 * PI,
        },
        TestCase {
            p1: [1.0, 0.0],
            p2: [3.0, 0.0],
            angle: 0.0,
        },
        TestCase {
            p1: [1.0, 0.0],
            p2: [-2.0, 0.0],
            angle: PI,
        },
        TestCase {
            p1: [2.0, 0.0],
            p2: [1.0, 1.0],
            angle: 0.25 * PI,
        },
    ];

    let vertex = DatumPoint { x_id: 0, y_id: 1 };
    let p1 = DatumPoint { x_id: 2, y_id: 3 };
    let p2 = DatumPoint { x_id: 4, y_id: 5 };

    for tc in &test_cases {
        let constraints = [ConstraintRequest::highest_priority(
            Constraint::PointsAtAngle(
                vertex,
                p1,
                p2,
                AngleKind::Other(Angle::from_radians(tc.angle)),
            ),
        )];
        let initial_guesses = vec![
            (0, 0.0),
            (1, 0.0),
            (2, tc.p1[0]),
            (3, tc.p1[1]),
            (4, tc.p2[0]),
            (5, tc.p2[1]),
        ];
        let outcome = solve(
            &constraints,
            initial_guesses,
            Config::default().with_max_iterations(100),
        )
        .unwrap_or_else(|e| panic!("failed for angle {}: {e:?}", tc.angle));
        assert!(outcome.is_satisfied());
        assert_eq!(
            outcome.iterations(),
            0,
            "angle {} should already be satisfied (0 iterations)",
            tc.angle
        );
    }
}

#[test]
fn points_at_angle_degenerate() {
    let vertex = DatumPoint { x_id: 0, y_id: 1 };
    let p1 = DatumPoint { x_id: 2, y_id: 3 };
    let p2 = DatumPoint { x_id: 4, y_id: 5 };

    let constraints = [ConstraintRequest::highest_priority(
        Constraint::PointsAtAngle(vertex, p1, p2, AngleKind::Other(Angle::from_degrees(180.0))),
    )];
    let initial_guesses = vec![
        (0, 0.0),
        (1, 0.0),
        (2, 13.0),
        (3, 13.0),
        (4, 13.0),
        (5, 13.0),
    ];
    let outcome = solve(
        &constraints,
        initial_guesses,
        Config::default().with_max_iterations(100),
    )
    .unwrap();
    assert!(outcome.warnings.first().unwrap().content == WarningContent::Degenerate);
}

#[test]
fn points_at_angle_unique_solution() {
    // PointsAtAngle has exactly one solution unlike LinesAtAngle which has two solutions for each
    // arm that differ by π
    //
    // Concretely, with target angle π/4 and u = (1,0):
    //   - v = (1,1) direction satisfies PointsAtAngle
    //   - v = (-1,-1) direction satisfies LinesAtAngle but not PointsAtAngle
    //
    // Starting from either initial condition, PointsAtAngle must converge to angle π/4.
    let vertex = DatumPoint { x_id: 0, y_id: 1 };
    let p1 = DatumPoint { x_id: 2, y_id: 3 };
    let p2 = DatumPoint { x_id: 4, y_id: 5 };

    let target_angle = 0.25 * PI;

    let constraints = [
        // Fix vertex and first arm
        ConstraintRequest::highest_priority(Constraint::Fixed(0, 0.0)),
        ConstraintRequest::highest_priority(Constraint::Fixed(1, 0.0)),
        ConstraintRequest::highest_priority(Constraint::Fixed(2, 1.0)),
        ConstraintRequest::highest_priority(Constraint::Fixed(3, 0.0)),
        ConstraintRequest::highest_priority(Constraint::PointsAtAngle(
            vertex,
            p1,
            p2,
            AngleKind::Other(Angle::from_radians(target_angle)),
        )),
    ];

    // p2 starts on the correct side: direction (1,1), angle π/4 so already satisfied
    let guesses_correct: Vec<(u32, f64)> =
        vec![(0, 0.0), (1, 0.0), (2, 1.0), (3, 0.0), (4, 1.0), (5, 1.0)];

    // p2 starts on the π-shifted side: direction (-1,-1), angle -3π/4.
    // This is the other zero of LinesAtAngle, but it should not satisfy PointsAtAngle.
    let guesses_shifted: Vec<(u32, f64)> =
        vec![(0, 0.0), (1, 0.0), (2, 1.0), (3, 0.0), (4, -1.0), (5, -1.0)];

    let outcome_correct = solve(
        &constraints,
        guesses_correct,
        Config::default().with_max_iterations(100),
    )
    .unwrap();
    let outcome_shifted = solve(
        &constraints,
        guesses_shifted,
        Config::default().with_max_iterations(100),
    )
    .unwrap();

    assert!(outcome_correct.is_satisfied());
    assert!(outcome_shifted.is_satisfied());

    // Both must converge to target_angle, not to target_angle - π.
    assert_nearly_eq(
        points_at_angle_from_vals(&outcome_correct.final_values),
        target_angle,
    );
    assert_nearly_eq(
        points_at_angle_from_vals(&outcome_shifted.final_values),
        target_angle,
    );
}

#[test]
fn points_at_angle_sign_distinguishable() {
    // Checks that PointsAtAngle respects the sign of the specified angle i.e. using +θ and -θ
    // should place the second arm on opposite sides of the first
    let vertex = DatumPoint { x_id: 0, y_id: 1 };
    let p1 = DatumPoint { x_id: 2, y_id: 3 };
    let p2 = DatumPoint { x_id: 4, y_id: 5 };
    let theta = 0.25 * PI;

    // (target_angle, initial_p2)
    let cases: &[(f64, [f64; 2])] = &[
        (theta, [1.0, 0.0]),
        (-theta, [1.0, 0.0]),
        (theta, [0.0, 1.0]),
        (-theta, [0.0, 1.0]),
        (theta, [-1.0, 0.0]),
        (-theta, [-1.0, 0.0]),
        (theta, [0.0, -1.0]),
        (-theta, [0.0, -1.0]),
    ];

    for &(target_angle, init_p2) in cases {
        let constraints = [
            // Fix vertex, first arm, and length of second arm
            ConstraintRequest::highest_priority(Constraint::Fixed(0, 0.0)),
            ConstraintRequest::highest_priority(Constraint::Fixed(1, 0.0)),
            ConstraintRequest::highest_priority(Constraint::Fixed(2, 1.0)),
            ConstraintRequest::highest_priority(Constraint::Fixed(3, 0.0)),
            ConstraintRequest::highest_priority(Constraint::Distance(vertex, p2, 1.0)),
            ConstraintRequest::highest_priority(Constraint::PointsAtAngle(
                vertex,
                p1,
                p2,
                AngleKind::Other(Angle::from_radians(target_angle)),
            )),
        ];
        let initial_guesses: Vec<(u32, f64)> = vec![
            (0, 0.0),
            (1, 0.0),
            (2, 1.0),
            (3, 0.0),
            (4, init_p2[0]),
            (5, init_p2[1]),
        ];

        let outcome = solve(
            &constraints,
            initial_guesses,
            Config::default().with_max_iterations(100),
        )
        .unwrap_or_else(|e| panic!("failed for angle {target_angle}: {e:?}"));
        assert!(outcome.is_satisfied());
        assert_nearly_eq(
            points_at_angle_from_vals(&outcome.final_values),
            target_angle,
        );
    }
}
