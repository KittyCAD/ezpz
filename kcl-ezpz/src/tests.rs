use std::str::FromStr;

use super::*;
use crate::{
    datatypes::Angle,
    textual::{OutcomeAnalysis, Point, Problem},
};

mod proptests;

fn run(test_case: &str) -> OutcomeAnalysis {
    let txt = std::fs::read_to_string(format!("../test_cases/{test_case}/problem.md")).unwrap();
    let problem = parse_problem(&txt);
    let system = problem.to_constraint_system().unwrap();
    system
        .solve_with_config_analysis(Config::default())
        .unwrap()
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
    let _e = crate::solve_with_priority(constraints.as_slice(), Vec::new(), Default::default())
        .unwrap_err();
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
    let solved =
        crate::solve_with_priority_analysis(&constraints, initial_guesses, Config::default())
            .unwrap();
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
    let solved =
        crate::solve_with_priority_analysis(&constraints, initial_guesses, Config::default())
            .unwrap();
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

    let solved =
        crate::solve_with_priority_analysis(&constraints, initial_guess, Config::default())
            .unwrap();
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

    let err = crate::solve_with_priority_analysis(&constraints, initial_guess, Config::default())
        .unwrap_err()
        .error;
    assert!(matches!(
        err,
        Error::NonLinearSystemError(NonLinearSystemError::MissingGuess {
            constraint_id: 0,
            variable: 0
        })
    ));
}

#[test]
fn coincident() {
    let solved = run("coincident");
    assert!(solved.is_satisfied());
    // P and Q are coincident, so they should be equal.
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 3.0, y: 3.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 3.0, y: 3.0 });
}

#[test]
fn massive() {
    let solved = run("massive_parallel_system");
    assert!(solved.is_satisfied());
    assert!(!solved.analysis.is_underconstrained);
}

#[test]
fn symmetric() {
    let solved = run("symmetric");
    assert!(solved.is_satisfied());
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
}

#[test]
fn perpdist_negative() {
    // Just like the `perpdist` test case, except the perpendicular distance is negative
    // instead of positive. So the point should be flipped to the other side of the line.
    let solved = run("perpdist_negative");
    assert!(solved.is_satisfied());
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
    // P and Q have a midpoint M.
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 2.0, y: 3.0 });
    assert_points_eq(solved.get_point("m").unwrap(), Point { x: 1.0, y: 1.5 });
}

#[test]
fn underconstrained() {
    let solved = run("underconstrained");
    assert!(solved.analysis.is_underconstrained);
    assert!(solved.is_satisfied());
    // p should be whatever the user's initial guess was.
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 1.0, y: 1.0 });
    // q should be what it was constrained to be.
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 0.0, y: 0.0 });
}

#[test]
fn tiny() {
    let solved = run("tiny");
    assert!(solved.is_satisfied());
    assert!(!solved.analysis.is_underconstrained);
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 0.0, y: 0.0 });
}

#[test]
fn tiny_no_regularization() {
    let solved = run_with_config(
        "tiny",
        Config {
            max_iterations: 25,
            ..Default::default()
        },
    );
    assert!(solved.is_satisfied());
    assert!(!solved.analysis.is_underconstrained);
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
    assert!(!solved.analysis.is_underconstrained); // If anything it's overconstrained not under.
    assert_points_eq(solved.get_point("o").unwrap(), Point { x: 0.0, y: 0.0 });
    // (2.5, 2.5) is midway between the two inconsistent requirement points.
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 2.5, y: 2.5 });
}

#[test]
fn circle() {
    let solved = run("circle");
    assert!(solved.is_satisfied());
    assert!(!solved.analysis.is_underconstrained);
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
    assert!(!solved.analysis.is_underconstrained);
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
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 0.0, y: 3.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 5.0, y: 3.0 });
    let circle_a = solved.get_circle("a").unwrap();
    assert_nearly_eq(circle_a.center.y, 1.5);
}

#[test]
fn two_rectangles() {
    let solved = run("two_rectangles");
    assert!(solved.is_satisfied());
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
    assert_points_eq(solved.get_point("p0").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("p1").unwrap(), Point { x: 0.0, y: 4.0 });
    assert_points_eq(solved.get_point("p2").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("p3").unwrap(), Point { x: 4.0, y: 0.0 });
}

#[test]
fn nonsquare() {
    let solved = run("nonsquare");
    assert!(solved.is_satisfied());
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 0.0, y: 0.0 });
}

#[test]
fn square() {
    let solved = run("square");
    assert!(solved.is_satisfied());
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
    assert!(solved.is_satisfied());
    assert_points_eq(solved.get_point("p0").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("p1").unwrap(), Point { x: 4.0, y: 0.0 });
    assert_points_eq(solved.get_point("p2").unwrap(), Point { x: 4.0, y: 4.0 });
}

#[test]
fn arc_radius() {
    let solved = run("arc_radius");
    assert!(solved.is_satisfied());
    let arc = solved.get_arc("a").unwrap();
    assert_points_eq(arc.center, Point { x: 0.0, y: 0.0 });
    assert_nearly_eq(5.0, arc.a.euclidean_distance(Default::default()));
    assert_nearly_eq(5.0, arc.b.euclidean_distance(Default::default()));
}

#[test]
fn arc_equidistant() {
    let solved = run("arc_equidistant");
    assert!(solved.is_satisfied());
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
    assert_points_eq(solved.get_point("a").unwrap(), Point { x: 0.0, y: 40.0 });
    assert_points_eq(solved.get_point("b").unwrap(), Point { x: 30.0, y: 40.0 });
    assert_points_eq(solved.get_point("c").unwrap(), Point { x: 40.0, y: 30.0 });
    assert_points_eq(solved.get_point("d").unwrap(), Point { x: 40.0, y: 0.0 });
    assert_points_eq(solved.get_point("e").unwrap(), Point { x: 0.0, y: 0.0 });
}

#[test]
fn strange_nonconvergence() {
    use crate::datatypes::DatumPoint;
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
            crate::datatypes::LineSegment { p0: q, p1: r },
            crate::datatypes::LineSegment { p0: s, p1: t },
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
    let outcome = crate::solve_with_priority(
        &requests,
        initial_guesses,
        Config {
            max_iterations: 31,
            ..Default::default()
        },
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
fn assert_nearly_eq(l: f64, r: f64) {
    let diff = (l - r).abs();
    assert!(
        diff < EPSILON,
        "LHS was {l}, RHS was {r}, difference was {diff}"
    );
}
