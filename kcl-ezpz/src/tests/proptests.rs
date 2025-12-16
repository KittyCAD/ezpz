use proptest::prelude::*;

use crate::{
    Config, Constraint, ConstraintRequest, IdGenerator, datatypes::DatumPoint, solve_with_priority,
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
        let outcome = solve_with_priority(
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

        let outcome = solve_with_priority(&requests, initial_guesses, Config::default())
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
}
