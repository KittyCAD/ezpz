use std::str::FromStr;

use super::*;
use crate::textual::{Point, Problem};

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
fn coincident() {
    let txt = include_str!("../../test_cases/coincident/problem.txt");
    let problem = parse_problem(txt);
    assert_eq!(problem.points(), vec!["p", "q"]);
    let solved = problem.to_constraint_system().unwrap().solve().unwrap();
    // P and Q are coincident, so they should be equal.
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 3.0, y: 3.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 3.0, y: 3.0 });
}

#[test]
fn underconstrained() {
    // Constrains q but not p, so the system is underdetermined.
    let txt = include_str!("../../test_cases/underconstrained/problem.txt");
    let problem = parse_problem(txt);
    assert_eq!(problem.points(), vec!["p", "q"]);
    let solved = problem.to_constraint_system().unwrap().solve().unwrap();
    // p should be whatever the user's initial guess was.
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 1.0, y: 1.0 });
    // q should be what it was constrained to be.
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 0.0, y: 0.0 });
}

#[test]
fn tiny() {
    let txt = include_str!("../../test_cases/tiny/problem.txt");
    let problem = parse_problem(txt);
    assert_eq!(problem.instructions.len(), 6);
    assert_eq!(problem.points(), vec!["p", "q"]);
    let solved = problem.to_constraint_system().unwrap().solve().unwrap();
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 0.0, y: 0.0 });
}

#[test]
fn inconsistent() {
    // This has inconsistent requirements:
    // p should be (1,4) and it should ALSO be (4,1).
    // Because they can't be simultaneously satisfied, we should find a
    // solution which minimizes the squared error instead.
    let txt = include_str!("../../test_cases/inconsistent/problem.txt");
    let problem = parse_problem(txt);
    let solved = problem.to_constraint_system().unwrap().solve().unwrap();
    assert_points_eq(solved.get_point("o").unwrap(), Point { x: 0.0, y: 0.0 });
    // (2.5, 2.5) is midway between the two inconsistent requirement points.
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 2.5, y: 2.5 });
}

#[test]
fn circle() {
    let txt = include_str!("../../test_cases/circle/problem.txt");
    let problem = parse_problem(txt);
    assert_eq!(problem.points(), vec!["p"]);
    assert_eq!(problem.circles(), vec!["a"]);
    let solved = problem.to_constraint_system().unwrap().solve().unwrap();
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
    let txt = include_str!("../../test_cases/circle_center/problem.txt");
    let problem = parse_problem(txt);
    let solved = problem.to_constraint_system().unwrap().solve().unwrap();
    let circle_a = solved.get_circle("a").unwrap();
    assert_nearly_eq(circle_a.radius, 1.0);
    assert_points_eq(circle_a.center, Point { x: 0.0, y: 0.0 });
}

#[test]
fn circle_tangent() {
    // There's two possible ways to put the circle, either at y=4.5 or y=1.5
    // Because the tangent constraint is directional, using PQ will always put it at
    // y=4.5. We test the other solution in the `circle_tangent_other_dir` test.
    let txt = include_str!("../../test_cases/circle_tangent/problem.txt");
    let problem = parse_problem(txt);
    assert_eq!(problem.points(), vec!["p", "q"]);
    assert_eq!(problem.circles(), vec!["a"]);
    let solved = problem.to_constraint_system().unwrap().solve().unwrap();
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 0.0, y: 3.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 5.0, y: 3.0 });
    let circle_a = solved.get_circle("a").unwrap();
    assert_nearly_eq(circle_a.center.y, 4.5);
}

#[test]
fn circle_tangent_other_dir() {
    // Just like `circle_tangent` but using line QP instead of PQ, to test the
    // other case of tangent direction.
    let txt = include_str!("../../test_cases/circle_tangent_other_dir/problem.txt");
    let problem = parse_problem(txt);
    assert_eq!(problem.points(), vec!["p", "q"]);
    assert_eq!(problem.circles(), vec!["a"]);
    let solved = problem.to_constraint_system().unwrap().solve().unwrap();
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 0.0, y: 3.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 5.0, y: 3.0 });
    let circle_a = solved.get_circle("a").unwrap();
    assert_nearly_eq(circle_a.center.y, 1.5);
}

#[test]
fn rectangle() {
    let txt = include_str!("../../test_cases/two_rectangles/problem.txt");
    let problem = Problem::from_str(txt).unwrap();
    let solved = problem.to_constraint_system().unwrap().solve().unwrap();
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
    for file in [
        include_str!("../../test_cases/angle_parallel/problem.txt"),
        include_str!("../../test_cases/angle_parallel_manual/problem.txt"),
    ] {
        let problem = Problem::from_str(file).unwrap();
        let solved = problem.to_constraint_system().unwrap().solve().unwrap();
        assert_points_eq(solved.get_point("p0").unwrap(), Point { x: 0.0, y: 0.0 });
        assert_points_eq(solved.get_point("p1").unwrap(), Point { x: 4.0, y: 4.0 });
        assert_points_eq(solved.get_point("p2").unwrap(), Point { x: 0.0, y: 0.0 });
        assert_points_eq(solved.get_point("p3").unwrap(), Point { x: 4.0, y: 4.0 });
    }
}

#[test]
fn perpendiculars() {
    let txt = include_str!("../../test_cases/perpendicular/problem.txt");
    let problem = Problem::from_str(txt).unwrap();
    let solved = problem.to_constraint_system().unwrap().solve().unwrap();
    assert_points_eq(solved.get_point("p0").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("p1").unwrap(), Point { x: 0.0, y: 4.0 });
    assert_points_eq(solved.get_point("p2").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("p3").unwrap(), Point { x: 4.0, y: 0.0 });
}

#[test]
fn nonsquare() {
    let txt = include_str!("../../test_cases/nonsquare/problem.txt");
    let problem = Problem::from_str(txt).unwrap();
    let solved = problem.to_constraint_system().unwrap().solve().unwrap();
    assert_points_eq(solved.get_point("p").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("q").unwrap(), Point { x: 0.0, y: 0.0 });
}

#[test]
fn square() {
    let txt = include_str!("../../test_cases/square/problem.txt");
    let problem = Problem::from_str(txt).unwrap();
    let solved = problem.to_constraint_system().unwrap().solve().unwrap();
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
    let txt = include_str!("../../test_cases/parallelogram/problem.txt");
    let problem = Problem::from_str(txt).unwrap();
    let solved = problem.to_constraint_system().unwrap().solve().unwrap();
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

#[test]
fn underdetermined_lines() {
    // This should solve for a horizontal line from (0,0) to (4,0), then
    // a vertical line from (4,0) to (4,4). Note that the length of the second
    // line is not specified; we're relying on regularisation to push our solution
    // towards its start point.
    let txt = include_str!("../../test_cases/underdetermined_lines/problem.txt");
    let problem = Problem::from_str(txt).unwrap();
    let solved = problem.to_constraint_system().unwrap().solve().unwrap();
    assert_points_eq(solved.get_point("p0").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("p1").unwrap(), Point { x: 4.0, y: 0.0 });
    assert_points_eq(solved.get_point("p2").unwrap(), Point { x: 4.0, y: 4.0 });
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
