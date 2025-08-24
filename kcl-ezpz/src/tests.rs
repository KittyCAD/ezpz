use super::*;
use crate::textual::{Point, Problem};

#[test]
fn simple() {
    let problem = Problem::parse(
        &mut "\
    # constraints
    point p
    point q
    p = (0, 0)
    q.y = 0
    vertical(p, q)

    # guesses
    p roughly (3, 4)
    q roughly (5, 6)
    ",
    )
    .unwrap();
    assert_eq!(problem.instructions.len(), 6);
    assert_eq!(problem.points(), vec!["p", "q"]);
    let solved = problem.solve().unwrap();
    assert_eq!(solved.get_point("p").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_eq!(solved.get_point("q").unwrap(), Point { x: 0.0, y: 0.0 });
}

#[test]
fn rectangle() {
    let mut txt = "\
    # constraints
    point p0
    point p1
    point p2
    point p3
    p0 = (1,1)
    horizontal(p0, p1)
    horizontal(p2, p3)
    vertical(p1, p2)
    vertical(p3, p0)
    distance(p0, p1, 4)
    distance(p0, p3, 3)
    point p4
    point p5
    point p6
    point p7
    p4 = (2, 2)
    horizontal(p4, p5)
    horizontal(p6, p7)
    vertical(p5, p6)
    vertical(p7, p4)
    distance(p4, p5, 4)
    distance(p4, p7, 4)

    # guesses
    p0 roughly (1,1)
    p1 roughly (4.5,1.5)
    p2 roughly (4.0,3.5)
    p3 roughly (1.5,3.0)
    p4 roughly (2,2)
    p5 roughly (5.5,3.5)
    p6 roughly (5,4.5)
    p7 roughly (2.5,4)
    ";
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
    let mut txt = "\
    # constraints
    point p0
    point p1
    point p2
    p0 = (0, 0)
    parallel(p0, p1, p1, p2)
    distance(p0, p1, sqrt(32))
    distance(p1, p2, sqrt(32))
    p1.x = 4

    # guesses
    p0 roughly (0,0)
    p1 roughly (3,3)
    p2 roughly (6,6)
    ";

    let problem = Problem::parse(&mut txt).unwrap();
    let solved = problem.solve().unwrap();
    assert_points_eq(solved.get_point("p0").unwrap(), Point { x: 0.0, y: 0.0 });
    assert_points_eq(solved.get_point("p1").unwrap(), Point { x: 4.0, y: 4.0 });
    assert_points_eq(solved.get_point("p2").unwrap(), Point { x: 8.0, y: 8.0 });
}

#[track_caller]
fn assert_points_eq(l: Point, r: Point) {
    let dist = l.euclidean_distance(r);
    assert!(dist < EPSILON, "LHS was {l}, RHS was {r}, dist was {dist}");
}
