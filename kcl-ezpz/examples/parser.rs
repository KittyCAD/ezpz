//! Parse the ezpz text format, which describes a constraint system,
//! then solve that system.
use kcl_ezpz::{datatypes::outputs::Point, textual::Problem};
use std::str::FromStr;

const EPSILON: f64 = 1e-5;

fn main() {
    let file = "\
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
";
    let problem = Problem::from_str(file).unwrap();
    let system = problem.to_constraint_system().unwrap();
    let solution = system.solve().unwrap();

    // Check the outcome.
    assert!(solution.unsatisfied.is_empty());
    assert_points_eq(solution.get_point("p").unwrap(), Point { x: 3.0, y: 3.0 });
    assert_points_eq(solution.get_point("q").unwrap(), Point { x: 3.0, y: 3.0 });
}

#[track_caller]
fn assert_points_eq(l: Point, r: Point) {
    let dist = l.euclidean_distance(r);
    assert!(dist < EPSILON, "LHS was {l}, RHS was {r}, dist was {dist}");
}
