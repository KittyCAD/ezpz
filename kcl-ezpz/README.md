# Ezpz

This is a 2D constraint solver, for use in CAD or graphics applications.

## Usage
```rust
use kcl_ezpz::{Config, solve, Constraint, ConstraintRequest, datatypes::inputs::DatumPoint, IdGenerator};

// Define the geometry.
// These entities don't have known positions or dimensions yet, the solver
// will place them for us.
let mut ids = IdGenerator::default();
let p = DatumPoint::new(&mut ids);
let q = DatumPoint::new(&mut ids);

// Define constraints on the geometry.
// These could constraint the entities themselves
// (e.g. the position of a point or the radius of a circle),
// or their relationship to each other
// (e.g. these two lines must be parallel, or this point must lie on this arc).
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
let outcome = solve(
    &requests,
    initial_guesses,
    // You can customize the config, but for this example, we'll just use the default.
    Config::default(),
);

// Check the outcome.
match outcome {
  Ok(solution) => {
    // If you give incompatible constraints, then your constraints cannot possibly
    // be satisfied. But in this example, there should be a solution.
    assert!(solution.is_satisfied());
    assert!(solution.unsatisfied().is_empty());
    let solved_p = solution.final_value_point(&p);
    let solved_q = solution.final_value_point(&q);
    println!("P = ({}, {})", solved_p.x, solved_p.y);
    println!("Q = ({}, {})", solved_q.x, solved_q.y);
  }
  Err(e) => {
    eprintln!("ezpz could not solve this constraint system: {}", e.error);
  }
}
```

## Constraint problem files

ezpz defines a text format for writing out constraint problems. You don't have to use this format -- you can use the Rust library directly -- but it's a very convenient format. It looks like this:

```md
# constraints
point p
point q
p.x = 0
p.y = 0
q.y = 0
vertical(p, q)

# guesses
p roughly (3, 4)
q roughly (5, 6)
```

There's two sections, Constraints and Guesses. You define each point (like `p` and `q`) and once defined, you can write constraints that use them. For example, you can fix a point's X or Y component (`p.x = 0`). Or you can relate two points, e.g. `vertical(p, q)`.

For more examples, see the [`test_cases/`](https://github.com/KittyCAD/ezpz/tree/main/test_cases) directory.
