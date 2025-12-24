# Ezpz

This is a 2D constraint solver, for use in CAD or graphics applications.

## Usage
```rust
    use kcl_ezpz::datatypes::DatumPoint;
    use kcl_ezpz::{Config, solve, Constraint, ConstraintRequest};
    let p = DatumPoint { x_id: 0, y_id: 1 };
    let q = DatumPoint { x_id: 2, y_id: 3 };
    let r = DatumPoint { x_id: 4, y_id: 5 };
    let s = DatumPoint { x_id: 6, y_id: 7 };


    let requests = [
        // Fix P to the origin
        ConstraintRequest::highest_priority(Constraint::Fixed(p.id_x(), 0.0)),
        ConstraintRequest::highest_priority(Constraint::Fixed(p.id_y(), 0.0)),
        // P and Q should be 4 units apart.
        ConstraintRequest::highest_priority(Constraint::Distance(p, q, 4.0)),
    ];
    let initial_guesses = vec![
        (0, 0.0),
        (1, -0.02),
        (2, 4.39),
        (3, 4.38),
    ];
    let outcome = solve(
        &requests,
        initial_guesses,
        Config::default(),
    );
    match outcome {
      Ok(solution) => {
        assert!(solution.is_satisfied());
        let (px, py) = (
          solution.final_values()[q.id_x() as usize],
          solution.final_values()[q.id_y() as usize],
        );
        let (qx, qy) = (
          solution.final_values()[q.id_x() as usize],
          solution.final_values()[q.id_y() as usize],
        );
        println!("P = ({px}, {py})");
        println!("Q = ({qx}, {qy})");
      }
      Err(e) => {
        eprintln!("ezpz could not solve this constraint system: {}", e.error);
      }
    }

```

## Constraint problem files

ezpz defines a text format for writing out constraint problems. You don't have to use this format -- you can use the Rust library directly -- but it's a very convenient format. It looks like this:

```ignore
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
