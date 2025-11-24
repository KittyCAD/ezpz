use proptest::prelude::*;

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
}
