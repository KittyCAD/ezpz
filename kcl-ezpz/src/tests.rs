use std::{f64::consts::PI, str::FromStr};

use super::*;
use crate::{
    datatypes::{
        Angle,
        inputs::{DatumCircularArc, DatumPoint},
        outputs::Point,
    },
    textual::{OutcomeAnalysis, Problem},
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
    // There's two possible ways to put the circle, either at y=4.5 or y=1.5
    // Because the tangent constraint is directional, using PQ will always put it at
    // y=4.5. We test the other solution in the `circle_tangent_other_dir` test.
    let solved = run("circle_tangent");
    assert!(solved.is_satisfied());
    assert!(!solved.analysis.is_underconstrained());
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 0.0, y: 3.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 5.0, y: 3.0 });
    let circle_a = solved.get_circle("a").unwrap();
    assert_nearly_eq(circle_a.center.y, 4.5);
}

#[test]
fn circle_tangent_other_dir() {
    // Just like `circle_tangent` but using line QP instead of PQ, to test the
    // other case of tangent direction.
    let solved = run("circle_tangent_other_dir");
    assert!(solved.is_satisfied());
    assert!(!solved.analysis.is_underconstrained());
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 0.0, y: 3.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 5.0, y: 3.0 });
    let circle_a = solved.get_circle("a").unwrap();
    assert_nearly_eq(circle_a.center.y, 1.5);
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
            datatypes::inputs::DatumLineSegment { p0: q, p1: r },
            datatypes::inputs::DatumLineSegment { p0: s, p1: t },
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

/// Test that reproduces a bug where adding a `point_arc_coincident` constraint
/// causes the solver to produce dramatically different results from the initial guesses,
/// even when the point is already basically on the arc.
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
        change_line4_start, initial_distance_from_arc
    );
}

/// Test that when a point is initially at the arc center (not on the arc's circumference
/// within the angular range), the `point_arc_coincident` constraint should cause it to
/// move significantly to a point on the arc within the angular range.
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
