//! A basic example for how to use the constraint solver.
use kcl_ezpz::{
    Config, Constraint, ConstraintRequest, IdGenerator, datatypes::inputs::DatumPoint, solve,
};

fn main() {
    // Define the geometry.
    // These entities don't have known positions or dimensions yet, the solver
    // will place them for us.
    let mut ids = IdGenerator::default();
    let p = DatumPoint::new(&mut ids);
    let q = DatumPoint::new(&mut ids);

    // Define constraints on the geometric entities (their dimensions and relation to each other).
    let requests = [
        // Fix P to the origin
        ConstraintRequest::highest_priority(Constraint::Fixed(p.id_x(), 0.0)),
        ConstraintRequest::highest_priority(Constraint::Fixed(p.id_y(), 0.0)),
        // P and Q should be 4 units apart.
        ConstraintRequest::highest_priority(Constraint::Distance(p, q, 4.0)),
    ];

    // Provide some initial guesses to the solver for their locations.
    let initial_guesses = vec![
        (p.id_x(), 0.0),
        (p.id_y(), -0.02),
        (q.id_x(), 4.39),
        (q.id_y(), 4.38),
    ];

    // Run the solver!
    let outcome = solve(&requests, initial_guesses, Config::default());

    // Check the outcome.
    match outcome {
        Ok(solution) => {
            assert!(solution.is_satisfied());
            let solved_p = solution.final_value_point(&p);
            let solved_q = solution.final_value_point(&q);
            println!("P = ({}, {})", solved_p.x, solved_p.y);
            println!("Q = ({}, {})", solved_q.x, solved_q.y);
        }
        Err(e) => {
            eprintln!("ezpz could not solve this constraint system: {}", e.error);
        }
    }
}
