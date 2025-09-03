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
fn tiny() {
    let txt = include_str!("../../test_cases/tiny/problem.txt");
    let problem = parse_problem(txt);
    assert_eq!(problem.instructions.len(), 6);
    assert_eq!(problem.points(), vec!["p", "q"]);
    let solved = problem.to_constraint_system().unwrap().solve().unwrap();
    assert_eq!(solved.get_point("p").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_eq!(solved.get_point("q").unwrap(), Point { x: 0.0, y: 0.0 });
}

#[test]
fn circle() {
    let txt = include_str!("../../test_cases/circle/problem.txt");
    let problem = parse_problem(txt);
    assert_eq!(problem.points(), vec!["a.center"]);
    let solved = problem.to_constraint_system().unwrap().solve().unwrap();
    // assert_eq!(solved.get_point("p").unwrap(), Point { x: 0.0, y: 0.0 });
    // assert_eq!(solved.get_point("q").unwrap(), Point { x: 0.0, y: 0.0 });
}

#[test]
fn rectangle() {
    let txt = include_str!("../../test_cases/two_rectangles/problem.txt");
    let problem = Problem::from_str(txt).unwrap();
    let solved = problem.to_constraint_system().unwrap().solve().unwrap();
    // This forms two rectangles.
    assert_eq!(solved.get_point("p0").unwrap(), Point { x: 1.0, y: 1.0 });
    assert_eq!(solved.get_point("p1").unwrap(), Point { x: 5.0, y: 1.0 });
    assert_eq!(solved.get_point("p2").unwrap(), Point { x: 5.0, y: 4.0 });
    assert_eq!(solved.get_point("p3").unwrap(), Point { x: 1.0, y: 4.0 });
    // Second rectangle
    assert_eq!(solved.get_point("p4").unwrap(), Point { x: 2.0, y: 2.0 });
    assert_eq!(solved.get_point("p5").unwrap(), Point { x: 6.0, y: 2.0 });
    assert_eq!(solved.get_point("p6").unwrap(), Point { x: 6.0, y: 6.0 });
    assert_eq!(solved.get_point("p7").unwrap(), Point { x: 2.0, y: 6.0 });
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
