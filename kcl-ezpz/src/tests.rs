use super::*;
use crate::textual::{Point, Problem};

#[test]
fn simple() {
    let mut txt = include_str!("../../test_cases/tiny/problem.txt");
    let problem = Problem::parse(&mut txt).unwrap();
    assert_eq!(problem.instructions.len(), 6);
    assert_eq!(problem.points(), vec!["p", "q"]);
    let solved = problem.solve().unwrap();
    assert_eq!(solved.get_point("p").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_eq!(solved.get_point("q").unwrap(), Point { x: 0.0, y: 0.0 });
}

#[test]
fn rectangle() {
    let mut txt = include_str!("../../test_cases/rectangle/problem.txt");
    let problem = Problem::parse(&mut txt).unwrap();
    let solved = problem.solve().unwrap();
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
    for mut file in [
        include_str!("../../test_cases/angle_parallel/problem.txt"),
        include_str!("../../test_cases/angle_parallel_manual/problem.txt"),
    ] {
        let problem = Problem::parse(&mut file).unwrap();
        let solved = problem.solve().unwrap();
        assert_points_eq(solved.get_point("p0").unwrap(), Point { x: 0.0, y: 0.0 });
        assert_points_eq(solved.get_point("p1").unwrap(), Point { x: 4.0, y: 4.0 });
        assert_points_eq(solved.get_point("p2").unwrap(), Point { x: 8.0, y: 8.0 });
    }
}

#[test]
fn perpendiculars() {
    let mut txt = include_str!("../../test_cases/perpendicular/problem.txt");
    let problem = Problem::parse(&mut txt).unwrap();
    let solved = problem.solve().unwrap();
    assert_points_eq(solved.get_point("p0").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("p1").unwrap(), Point { x: 0.0, y: 4.0 });
    assert_points_eq(solved.get_point("p2").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("p3").unwrap(), Point { x: 4.0, y: 0.0 });
}

#[track_caller]
fn assert_points_eq(l: Point, r: Point) {
    let dist = l.euclidean_distance(r);
    assert!(dist < EPSILON, "LHS was {l}, RHS was {r}, dist was {dist}");
}
