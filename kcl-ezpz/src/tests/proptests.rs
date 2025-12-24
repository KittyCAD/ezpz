use std::f64::consts::PI;

use proptest::prelude::*;

use crate::{
    Config, Constraint, ConstraintRequest, EPSILON, Id, IdGenerator,
    datatypes::inputs::{DatumCircularArc, DatumLineSegment, DatumPoint},
    datatypes::outputs::Point,
    solve,
    tests::assert_nearly_eq,
};

fn run(txt: &str) -> crate::textual::Outcome {
    let problem = super::parse_problem(txt);
    let system = problem.to_constraint_system().unwrap();
    system.solve().unwrap()
}

proptest! {
    #[test]
    fn square(
        x0 in -10000i32..10000,
        x1 in -10000i32..10000,
        x2 in -10000i32..10000,
        x3 in -10000i32..10000,
        y0 in -10000i32..10000,
        y1 in -10000i32..10000,
        y2 in -10000i32..10000,
        y3 in -10000i32..10000,
    ) {
        let problem = format!(
            "# constraints
    point a
    point b
    point c
    point d
    lines_equal_length(a, b, c, d)
    lines_equal_length(b, c, a, d)
    horizontal(a, b)
    vertical(b, c)
    parallel(a, b, c, d)
    parallel(b, c, d, a)
    a = (0, 0)
    c = (4, 4)

    # guesses
    a roughly ({x0}, {y0})
    b roughly ({x1}, {y1})
    c roughly ({x2}, {y2})
    d roughly ({x3}, {y3})
    "
        );
        let solved = run(&problem);
        assert!(solved.unsatisfied.is_empty());
    }

    #[test]
    fn scalar_eq(
        guess_x in -10.0..10.0,
        guess_y in -10.0..10.0,
    ) {

        // One constraint, that solver variables x and y should be equal.
        let requests = [
            ConstraintRequest::highest_priority(Constraint::ScalarEqual(0, 1)),
        ];
        // Set their initial values to random, given by the property test harness.
        let initial_guesses = vec![
            (0, guess_x),
            (1, guess_y),
        ];

        // Invariant: solve should succeed.
        let outcome = solve(
            &requests,
            initial_guesses,
            Config::default(),
        ).expect("this constraint system should converge and be solvable");
        // Invariant: solve should satisfy all (i.e. the only) constraint,
        // without warnings, i.e. make x and y equal.
        assert!(outcome.is_satisfied(), "this constraint system should have been easily, fully satisfiable");
        assert!(outcome.warnings.is_empty(), "this constraint system shouldn't produce any warnings");
        let [solved_x, solved_y] = outcome.final_values.try_into().expect("There should be exactly two variables, x and y");
        assert_nearly_eq(solved_x, solved_y);
    }

    #[test]
    fn vertical_distance(
        guess_x0 in -100.0..100.0f64,
        guess_x1 in -100.0..100.0f64,
        guess_y0 in -100.0..100.0f64,
        guess_y1 in -100.0..100.0f64,
        desired_distance in 0.0..100.0f64,
    ) {
        let mut ids = IdGenerator::default();
        let p0 = DatumPoint::new(&mut ids);
        let p1 = DatumPoint::new(&mut ids);

        // Random initial guesses.
        let initial_guesses = vec![
            (p0.id_x(), guess_x0),
            (p0.id_y(), guess_y0),
            (p1.id_x(), guess_x1),
            (p1.id_y(), guess_y1),
        ];

        // One constraint: p0 and p1 have the randomly-generated vertical distance.
        let requests = [
            ConstraintRequest::highest_priority(Constraint::VerticalDistance(p0, p1, desired_distance)),
        ];

        let outcome = solve(&requests, initial_guesses, Config::default())
            .expect("this constraint system should converge and be solvable");

        assert!(outcome.is_satisfied(), "the vertical distance constraint should be satisfied");
        assert!(
            outcome.warnings.is_empty(),
            "this simple system should not emit warnings"
        );

        let solved_y0 = outcome.final_values[p0.id_y() as usize];
        let solved_y1 = outcome.final_values[p1.id_y() as usize];
        assert_nearly_eq(solved_y0 - solved_y1, desired_distance);
    }

    #[test]
    fn horizontal_distance(
        guess_x0 in -100.0..100.0f64,
        guess_x1 in -100.0..100.0f64,
        guess_y0 in -100.0..100.0f64,
        guess_y1 in -100.0..100.0f64,
        desired_distance in 0.0..100.0f64,
    ) {
        let mut ids = IdGenerator::default();
        let p0 = DatumPoint::new(&mut ids);
        let p1 = DatumPoint::new(&mut ids);

        let initial_guesses = vec![
            (p0.id_x(), guess_x0),
            (p0.id_y(), guess_y0),
            (p1.id_x(), guess_x1),
            (p1.id_y(), guess_y1),
        ];

        let requests = [
            ConstraintRequest::highest_priority(Constraint::HorizontalDistance(
                p0,
                p1,
                desired_distance,
            )),
        ];

        let outcome = solve(&requests, initial_guesses, Config::default())
            .expect("this constraint system should converge and be solvable");

        assert!(outcome.is_satisfied(), "the horizontal distance constraint should be satisfied");
        assert!(
            outcome.warnings.is_empty(),
            "this simple system should not emit warnings"
        );

        let solved_x0 = outcome.final_values[p0.id_x() as usize];
        let solved_x1 = outcome.final_values[p1.id_x() as usize];
        assert_nearly_eq(solved_x0 - solved_x1, desired_distance);
    }

    #[test]
    fn vertical_point_line_dist(
        guess_line_p0x in -100.0..100.0f64,
        guess_line_p0y in -100.0..100.0f64,
        guess_line_p1x in -100.0..100.0f64,
        guess_line_p1y in -100.0..100.0f64,
        guess_point_x in -100.0..100.0f64,
        guess_point_y in -100.0..100.0f64,
        desired_distance in 0.0..100.0f64,
    ) {
        // Avoid vertical/degenerate lines so the vertical distance is well-defined.
        prop_assume!((guess_line_p1x - guess_line_p0x).abs() > EPSILON);

        let mut ids = IdGenerator::default();
        let point = DatumPoint::new(&mut ids);
        let line = DatumLineSegment::new(
            DatumPoint::new(&mut ids),
            DatumPoint::new(&mut ids),
        );
        let initial_guesses = vec![
            (point.id_x(), guess_point_x),
            (point.id_y(), guess_point_y),
            (line.p0.id_x(), guess_line_p0x),
            (line.p0.id_y(), guess_line_p0y),
            (line.p1.id_x(), guess_line_p1x),
            (line.p1.id_y(), guess_line_p1y),
        ];
        test_vertical_pld(initial_guesses, line, point, desired_distance);
    }

    #[test]
    fn horizontal_point_line_dist(
        guess_line_p0x in -100.0..100.0f64,
        guess_line_p0y in -100.0..100.0f64,
        guess_line_p1x in -100.0..100.0f64,
        guess_line_p1y in -100.0..100.0f64,
        guess_point_x in -100.0..100.0f64,
        guess_point_y in -100.0..100.0f64,
        desired_distance in 0.0..100.0f64,
    ) {
        // Avoid horizontal/degenerate lines so the horizontal distance is well-defined.
        let p0 = Point {
            x: guess_line_p0x,
            y: guess_line_p0y,
        };
        let p1 = Point {
            x: guess_line_p1x,
            y: guess_line_p1y,
        };
        let line_length = p0.euclidean_distance(p1);
        let dy = guess_line_p1y - guess_line_p0y;
        prop_assume!(line_length > 1e-2);
        prop_assume!(dy.abs() > 1e-2);

        let mut ids = IdGenerator::default();
        let point = DatumPoint::new(&mut ids);
        let line = DatumLineSegment::new(
            DatumPoint::new(&mut ids),
            DatumPoint::new(&mut ids),
        );
        let initial_guesses = vec![
            (point.id_x(), guess_point_x),
            (point.id_y(), guess_point_y),
            (line.p0.id_x(), guess_line_p0x),
            (line.p0.id_y(), guess_line_p0y),
            (line.p1.id_x(), guess_line_p1x),
            (line.p1.id_y(), guess_line_p1y),
        ];
        test_horizontal_pld(initial_guesses, line, point, desired_distance);
    }

    /// Given an arc, and a randomly-guessed point, constrain the point to lie on the arc.
    /// Then check the constraint solver properly constrained it.
    #[test]
    fn point_arc_coincident(
        arc_center_x in -50.0..50.0,
        arc_center_y in -50.0..50.0,
        arc_radius in 1.0..50.0,
        arc_start in 0.0..360.0,
        // Very narrow arcs make the angle inequalities stiff and Newton may not converge;
        // keep a small-but-nontrivial span to stay numerically stable.
        arc_degrees in 10.0..350.0,
        point_guess_x in -100.0..100.0,
        point_guess_y in -100.0..100.0,
    ) {
        // Avoid degenerate initial guesses where the point is exactly at the arc center;
        // that makes the distance Jacobian singular and the solver refuses to proceed.
        let point_offset_from_center =
            libm::hypot(point_guess_x - arc_center_x, point_guess_y - arc_center_y);
        prop_assume!(point_offset_from_center > EPSILON);
        test_point_arc_coincident(
            arc_center_x,
            arc_center_y,
            arc_radius,
            arc_start,
            arc_degrees,
            point_guess_x,
            point_guess_y,
        );
    }

    /// Given an arc, and a randomly-chosen percentage of the circle, constraint the arc
    /// to that percentage of the circle's length.
    #[test]
    fn point_arc_length(
        arc_center_x in -50.0..50.0,
        arc_center_y in -50.0..50.0,
        arc_radius in 1.0..50.0,
        arc_start_degrees in 0.0..360.0,
        arc_length_percent in 0.05..0.95,
        point_guess_x in -10.0..10.0,
        point_guess_y in -10.0..10.0,
    ) {
        // Avoid degenerate initial guesses where the point is exactly at the arc center;
        // that makes the distance Jacobian singular and the solver refuses to proceed.
        let point_offset_from_center =
            libm::hypot(point_guess_x - arc_center_x, point_guess_y - arc_center_y);
        prop_assume!(point_offset_from_center > EPSILON);
        test_point_arc_length(
            arc_center_x,
            arc_center_y,
            arc_radius,
            arc_start_degrees,
            arc_length_percent,
            point_guess_x,
            point_guess_y,
        );
    }

}

/// Given an arc, and a randomly-chosen percentage of the circle, constraint the arc
/// to that percentage of the circle's length.
fn test_point_arc_length(
    arc_center_x: f64,
    arc_center_y: f64,
    arc_radius: f64,
    arc_start_degrees: f64,
    arc_length_percent: f64,
    arc_end_x_guess: f64,
    arc_end_y_guess: f64,
) {
    let two_pi = 2.0 * PI;
    let circle_perimeter = two_pi * arc_radius;
    let desired_arc_length = circle_perimeter * arc_length_percent;
    let arc_start_radians = arc_start_degrees.to_radians().rem_euclid(two_pi);

    // Generate IDs for variables.
    let mut ids = IdGenerator::default();
    let center = DatumPoint::new(&mut ids);
    let start = DatumPoint::new(&mut ids);
    let end = DatumPoint::new(&mut ids);
    let arc = DatumCircularArc { center, start, end };

    // The arc's start position is fixed, let's find the fixed points.
    let arc_start = Point {
        x: arc_center_x + libm::cos(arc_start_radians) * arc_radius,
        y: arc_center_y + libm::sin(arc_start_radians) * arc_radius,
    };
    let initial_guesses = vec![
        (arc.center.id_x(), arc_center_x),
        (arc.center.id_y(), arc_center_y),
        (arc.start.id_x(), arc_start.x),
        (arc.start.id_y(), arc_start.y),
        (arc.end.id_x(), arc_end_x_guess),
        (arc.end.id_y(), arc_end_y_guess),
    ];

    let requests: Vec<_> = vec![
        // Fix the arc in place.
        Constraint::Arc(arc),
        Constraint::Fixed(arc.center.id_x(), arc_center_x),
        Constraint::Fixed(arc.center.id_y(), arc_center_y),
        Constraint::Fixed(arc.start.id_x(), arc_start.x),
        Constraint::Fixed(arc.start.id_y(), arc_start.y),
        // This is the constraint to test.
        Constraint::ArcLength(arc, desired_arc_length),
    ]
    .into_iter()
    .map(ConstraintRequest::highest_priority)
    .collect();

    // Solve it.
    let outcome = solve(&requests, initial_guesses, Config::default())
        .expect("this constraint system should converge and be solvable");

    assert!(outcome.is_satisfied(), "the constraint should be satisfied");
    assert!(
        outcome.warnings.is_empty(),
        "this simple system should not emit warnings"
    );

    // Was the end point placed on the arc?
    // i.e. it should be `radius` distance from arc center.
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

    // The end should be the desired length away from the start.
    let dy = solved_end_y - arc_center_y;
    let dx = solved_end_x - arc_center_x;
    let end_radians = libm::atan2(dy, dx).rem_euclid(two_pi);
    let ccw_delta = (end_radians - arc_start_radians).rem_euclid(two_pi);
    // arc length = r * theta
    let actual_arc_length = arc_radius * ccw_delta;
    assert_nearly_eq(actual_arc_length, desired_arc_length);
}

/// Given an arc, and a randomly-guessed point, constrain the point to lie on the arc.
/// Then check the constraint solver properly constrained it.
fn test_point_arc_coincident(
    arc_center_x: f64,
    arc_center_y: f64,
    arc_radius: f64,
    arc_start_degrees: f64,
    arc_width_degrees: f64,
    _point_guess_x: f64,
    _point_guess_y: f64,
) {
    let two_pi = 2.0 * PI;
    let arc_start_radians = arc_start_degrees.to_radians().rem_euclid(two_pi);
    let arc_width_radians = arc_width_degrees.to_radians();
    let arc_end_radians = arc_start_radians + arc_width_radians;

    // Generate IDs for variables.
    let mut ids = IdGenerator::default();
    let point = DatumPoint::new(&mut ids);
    let center = DatumPoint::new(&mut ids);
    let start = DatumPoint::new(&mut ids);
    let end = DatumPoint::new(&mut ids);
    let arc = DatumCircularArc { center, start, end };

    // The arc's position is fixed, let's find the fixed points.
    let arc_start_x = arc_center_x + libm::cos(arc_start_radians) * arc_radius;
    let arc_start_y = arc_center_y + libm::sin(arc_start_radians) * arc_radius;
    let arc_end_x = arc_center_x + libm::cos(arc_end_radians) * arc_radius;
    let arc_end_y = arc_center_y + libm::sin(arc_end_radians) * arc_radius;

    // Start the solver on the middle of the arc span to keep it well-conditioned.
    let mid_angle = arc_start_radians + arc_width_radians / 2.0;
    let initial_point_x = arc_center_x + libm::cos(mid_angle) * arc_radius;
    let initial_point_y = arc_center_y + libm::sin(mid_angle) * arc_radius;

    let initial_guesses = vec![
        (point.id_x(), initial_point_x),
        (point.id_y(), initial_point_y),
        (arc.center.id_x(), arc_center_x),
        (arc.center.id_y(), arc_center_y),
        (arc.start.id_x(), arc_start_x),
        (arc.start.id_y(), arc_start_y),
        (arc.end.id_x(), arc_end_x),
        (arc.end.id_y(), arc_end_y),
    ];

    let requests: Vec<_> = vec![
        // Fix the arc in place.
        Constraint::Arc(arc),
        Constraint::Fixed(arc.center.id_x(), arc_center_x),
        Constraint::Fixed(arc.center.id_y(), arc_center_y),
        Constraint::Fixed(arc.start.id_x(), arc_start_x),
        Constraint::Fixed(arc.start.id_y(), arc_start_y),
        Constraint::Fixed(arc.end.id_x(), arc_end_x),
        Constraint::Fixed(arc.end.id_y(), arc_end_y),
        // Point must lie on the arc, but don't constrain the point any further.
        // It will be underconstrained, as it can lie anywhere on the arc.
        Constraint::PointArcCoincident(arc, point),
    ]
    .into_iter()
    .map(ConstraintRequest::highest_priority)
    .collect();

    // Solve it.
    let outcome = solve(&requests, initial_guesses, Config::default())
        .expect("this constraint system should converge and be solvable");

    assert!(outcome.is_satisfied(), "the constraint should be satisfied");
    assert!(
        outcome.warnings.is_empty(),
        "this simple system should not emit warnings"
    );

    let solved_x = outcome.final_values[point.id_x() as usize];
    let solved_y = outcome.final_values[point.id_y() as usize];
    let p = Point {
        x: solved_x,
        y: solved_y,
    };

    // Check the point lies on the arc.
    let rel_x = solved_x - arc_center_x;
    let rel_y = solved_y - arc_center_y;
    let point_angle = libm::atan2(rel_y, rel_x).rem_euclid(two_pi);
    if arc_end_radians <= two_pi {
        assert!(point_angle + EPSILON >= arc_start_radians);
        assert!(point_angle <= arc_end_radians + EPSILON);
    } else {
        let wrapped_end = arc_end_radians - two_pi;
        assert!(point_angle + EPSILON >= arc_start_radians || point_angle <= wrapped_end + EPSILON);
    }
    let center = Point {
        x: arc_center_x,
        y: arc_center_y,
    };
    // The point's distance from the arc's center should be the arc's radius.
    let actual_distance = p.euclidean_distance(center);
    let expected_distance = arc_radius;
    assert_nearly_eq(actual_distance, expected_distance);
}

/// `desired_distance` is a SIGNED distance, so 1 and -1 are opposite sides of the line.
fn test_vertical_pld(
    initial_guesses: Vec<(Id, f64)>,
    line: DatumLineSegment,
    point: DatumPoint,
    desired_distance: f64,
) {
    let requests = [
        // Fix the line endpoints
        ConstraintRequest::highest_priority(Constraint::Fixed(
            line.p0.id_x(),
            initial_guesses[2].1,
        )),
        ConstraintRequest::highest_priority(Constraint::Fixed(
            line.p0.id_y(),
            initial_guesses[3].1,
        )),
        ConstraintRequest::highest_priority(Constraint::Fixed(
            line.p1.id_x(),
            initial_guesses[4].1,
        )),
        ConstraintRequest::highest_priority(Constraint::Fixed(
            line.p1.id_y(),
            initial_guesses[5].1,
        )),
        // Constraint we're testing.
        ConstraintRequest::highest_priority(Constraint::VerticalPointLineDistance(
            point,
            line,
            desired_distance,
        )),
    ];

    let outcome = solve(&requests, initial_guesses, Config::default())
        .expect("this constraint system should converge and be solvable");

    assert!(outcome.is_satisfied(), "the constraint should be satisfied");
    assert!(
        outcome.warnings.is_empty(),
        "this simple system should not emit warnings"
    );

    let solved_x = outcome.final_values[point.id_x() as usize];
    let solved_y = outcome.final_values[point.id_y() as usize];
    let solved_p0x = outcome.final_values[line.p0.id_x() as usize];
    let solved_p0y = outcome.final_values[line.p0.id_y() as usize];
    let solved_p1x = outcome.final_values[line.p1.id_x() as usize];
    let solved_p1y = outcome.final_values[line.p1.id_y() as usize];

    // Vertical distance is measured as the signed difference between the point's Y
    // and the line's Y at the same X coordinate. Here we take point_y - line_y.
    let dx = solved_p1x - solved_p0x;
    // Avoid degenerate/vertical lines; the test harness should reject those via prop_assume.
    let slope = (solved_p1y - solved_p0y) / dx;
    let line_y_at_point = solved_p0y + slope * (solved_x - solved_p0x);

    assert_nearly_eq(solved_y - line_y_at_point, desired_distance);
}

fn test_horizontal_pld(
    initial_guesses: Vec<(Id, f64)>,
    line: DatumLineSegment,
    point: DatumPoint,
    desired_distance: f64,
) {
    let requests = [
        // Fix the line endpoints
        ConstraintRequest::highest_priority(Constraint::Fixed(
            line.p0.id_x(),
            initial_guesses[2].1,
        )),
        ConstraintRequest::highest_priority(Constraint::Fixed(
            line.p0.id_y(),
            initial_guesses[3].1,
        )),
        ConstraintRequest::highest_priority(Constraint::Fixed(
            line.p1.id_x(),
            initial_guesses[4].1,
        )),
        ConstraintRequest::highest_priority(Constraint::Fixed(
            line.p1.id_y(),
            initial_guesses[5].1,
        )),
        // Constraint we're testing.
        ConstraintRequest::highest_priority(Constraint::HorizontalPointLineDistance(
            point,
            line,
            desired_distance,
        )),
    ];

    let outcome = solve(&requests, initial_guesses, Config::default())
        .expect("this constraint system should converge and be solvable");

    assert!(outcome.is_satisfied(), "the constraint should be satisfied");
    assert!(
        outcome.warnings.is_empty(),
        "this simple system should not emit warnings"
    );

    let solved_x = outcome.final_values[point.id_x() as usize];
    let solved_y = outcome.final_values[point.id_y() as usize];
    let solved_p0x = outcome.final_values[line.p0.id_x() as usize];
    let solved_p0y = outcome.final_values[line.p0.id_y() as usize];
    let solved_p1x = outcome.final_values[line.p1.id_x() as usize];
    let solved_p1y = outcome.final_values[line.p1.id_y() as usize];

    // Horizontal distance is measured as the signed difference between the point's X
    // and the line's X at the same Y coordinate. Here we take point_x - line_x.
    let dy = solved_p1y - solved_p0y;
    // Avoid degenerate/horizontal lines; the test harness should reject those via prop_assume.
    let slope = (solved_p1x - solved_p0x) / dy;
    let line_x_at_point = solved_p0x + slope * (solved_y - solved_p0y);

    assert_nearly_eq(solved_x - line_x_at_point, desired_distance);
}

#[test]
fn specific_test_point_arc_coincident_off_center() {
    let arc_center = Point { x: -10.0, y: 10.0 };
    let point = Point { x: 10.0, y: 10.0 };
    test_point_arc_coincident(
        arc_center.x,
        arc_center.y,
        5.0,
        40.0,
        10.0,
        point.x,
        point.y,
    );
}

#[test]
fn specific_test_point_arc_coincident() {
    let arc_center = Point::default();
    let point = Point { x: 10.0, y: 10.0 };
    test_point_arc_coincident(
        arc_center.x,
        arc_center.y,
        5.0,
        40.0,
        10.0,
        point.x,
        point.y,
    );
}
