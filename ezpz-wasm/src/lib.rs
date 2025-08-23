use kcl_ezpz::{
    Constraint, IdGenerator,
    datatypes::{DatumPoint, LineSegment},
    solve,
};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub fn hello() -> i32 {
    33
}

#[wasm_bindgen]
pub fn test_faer() -> f64 {
    use faer::mat;

    let a = mat![
        [1.0, 5.0, 9.0], //
        [2.0, 6.0, 10.0],
        [3.0, 7.0, 11.0],
        [4.0, 8.0, 12.0f64],
    ];

    a[(0, 0)]
}

#[wasm_bindgen]
pub fn benchmark() -> Vec<f64> {
    let mut id_generator = IdGenerator::default();
    let p0 = DatumPoint::new(&mut id_generator);
    let p1 = DatumPoint::new(&mut id_generator);
    let p2 = DatumPoint::new(&mut id_generator);
    let p3 = DatumPoint::new(&mut id_generator);
    let line0_bottom = LineSegment::new(p0, p1);
    let line0_right = LineSegment::new(p1, p2);
    let line0_top = LineSegment::new(p2, p3);
    let line0_left = LineSegment::new(p3, p0);
    // Second square (upper case IDs)
    let p5 = DatumPoint::new(&mut id_generator);
    let p6 = DatumPoint::new(&mut id_generator);
    let p7 = DatumPoint::new(&mut id_generator);
    let line1_bottom = LineSegment::new(p2, p5);
    let line1_right = LineSegment::new(p5, p6);
    let line1_top = LineSegment::new(p6, p7);
    let line1_left = LineSegment::new(p7, p2);
    // First square (lower case IDs)
    let constraints0 = vec![
        Constraint::Fixed(p0.id_x(), 1.0),
        Constraint::Fixed(p0.id_y(), 1.0),
        Constraint::Horizontal(line0_bottom),
        Constraint::Horizontal(line0_top),
        Constraint::Vertical(line0_left),
        Constraint::Vertical(line0_right),
        Constraint::Distance(p0, p1, 4.0),
        Constraint::Distance(p0, p3, 3.0),
    ];

    // Start p at the origin, and q at (1,9)
    let initial_guesses = vec![
        // First square.
        (p0.id_x(), 1.0),
        (p0.id_y(), 1.0),
        (p1.id_x(), 4.5),
        (p1.id_y(), 1.5),
        (p2.id_x(), 4.0),
        (p2.id_y(), 3.5),
        (p3.id_x(), 1.5),
        (p3.id_y(), 3.0),
        // Second square.
        (p5.id_x(), 5.5),
        (p5.id_y(), 3.5),
        (p6.id_x(), 5.0),
        (p6.id_y(), 4.5),
        (p7.id_x(), 2.5),
        (p7.id_y(), 4.0),
    ];

    let constraints1 = vec![
        Constraint::Horizontal(line1_bottom),
        Constraint::Horizontal(line1_top),
        Constraint::Vertical(line1_left),
        Constraint::Vertical(line1_right),
        Constraint::Distance(p2, p5, 4.0),
        Constraint::Distance(p2, p7, 4.0),
    ];

    let mut constraints = constraints0;
    constraints.extend(constraints1);
    let actual = solve(constraints.clone(), initial_guesses.clone()).unwrap();
    actual.final_values
    // vec![]
}
