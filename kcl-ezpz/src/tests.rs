use std::str::FromStr;

use super::*;
use crate::textual::{Outcome, Point, Problem};

fn run(test_case: &str) -> Outcome {
    let txt = std::fs::read_to_string(format!("../test_cases/{test_case}/problem.txt")).unwrap();
    let problem = parse_problem(&txt);
    let system = problem.to_constraint_system().unwrap();
    system.solve().unwrap()
}

fn run_with_config(test_case: &str, config: Config) -> Outcome {
    let txt = std::fs::read_to_string(format!("../test_cases/{test_case}/problem.txt")).unwrap();
    let problem = parse_problem(&txt);
    let system = problem.to_constraint_system().unwrap();
    system.solve_with_config(config).unwrap()
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
    let constraints = vec![Constraint::Fixed(0, 0.0)];
    // We don't pass any variables, so this should return an error,
    // because the constraint requires variable 0, and it's not given.
    let _e = crate::solve(constraints.as_slice(), Vec::new(), Default::default()).unwrap_err();
}

#[test]
fn coincident() {
    let solved = run("coincident");
    // P and Q are coincident, so they should be equal.
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 3.0, y: 3.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 3.0, y: 3.0 });
}

#[test]
fn underconstrained() {
    let solved = run("underconstrained");
    // p should be whatever the user's initial guess was.
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 1.0, y: 1.0 });
    // q should be what it was constrained to be.
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 0.0, y: 0.0 });
}

#[test]
fn tiny() {
    let solved = run("tiny");
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 0.0, y: 0.0 });
}

#[test]
fn tiny_no_regularization() {
    let solved = run_with_config(
        "tiny",
        Config {
            regularization_enabled: false,
        },
    );
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
    assert_points_eq(solved.get_point("o").unwrap(), Point { x: 0.0, y: 0.0 });
    // (2.5, 2.5) is midway between the two inconsistent requirement points.
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 2.5, y: 2.5 });
}

#[test]
fn circle() {
    let solved = run("circle");
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
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 0.0, y: 3.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 5.0, y: 3.0 });
    let circle_a = solved.get_circle("a").unwrap();
    assert_nearly_eq(circle_a.center.y, 1.5);
}

#[test]
fn two_rectangles() {
    let solved = run("two_rectangles");
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
        assert_points_eq(solved.get_point("p0").unwrap(), Point { x: 0.0, y: 0.0 });
        assert_points_eq(solved.get_point("p1").unwrap(), Point { x: 4.0, y: 4.0 });
        assert_points_eq(solved.get_point("p2").unwrap(), Point { x: 0.0, y: 0.0 });
        assert_points_eq(solved.get_point("p3").unwrap(), Point { x: 4.0, y: 4.0 });
    }
}

#[test]
fn perpendicular() {
    let solved = run("perpendicular");
    assert_points_eq(solved.get_point("p0").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("p1").unwrap(), Point { x: 0.0, y: 4.0 });
    assert_points_eq(solved.get_point("p2").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("p3").unwrap(), Point { x: 4.0, y: 0.0 });
}

#[test]
fn nonsquare() {
    let solved = run("nonsquare");
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 0.0, y: 0.0 });
}

#[test]
fn square() {
    let solved = run("square");
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
    assert_points_eq(solved.get_point("p0").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("p1").unwrap(), Point { x: 4.0, y: 0.0 });
    assert_points_eq(solved.get_point("p2").unwrap(), Point { x: 4.0, y: 4.0 });
}

#[test]
fn arc_radius() {
    let solved = run("arc_radius");
    let arc = solved.get_arc("a").unwrap();
    assert_points_eq(arc.center, Point { x: 0.0, y: 0.0 });
    assert_points_eq(arc.a, Point { x: 0.0, y: 5.0 });
    assert_points_eq(arc.b, Point { x: 5.0, y: 0.0 });
}

#[test]
fn arc_equidistant() {
    let solved = run("arc_equidistant");
    let arc = solved.get_arc("a").unwrap();
    assert_points_eq(arc.center, Point { x: 0.0, y: 0.0 });
    assert_nearly_eq(
        arc.a.euclidean_distance(arc.center),
        arc.b.euclidean_distance(arc.center),
    );
}

#[test]
fn lints() {
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
    assert!(!solved.lints.is_empty());
    assert_eq!(
        solved.lints,
        vec![Lint {
            about_constraint: Some(7),
            content: content_for_angle(true, 0.0),
        }]
    );
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
