use super::*;
use crate::{
    datatypes::{DatumPoint, LineSegment},
    textual::{Point, Problem},
};

#[test]
fn simple() {
    let problem = Problem::parse(
        &mut "\
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
    p0.x = 1
    p0.y = 1
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
    p4.x = 2
    p4.y = 2
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
    let mut id_generator = IdGenerator::default();
    // It has 4 points.
    let p0 = DatumPoint::new(&mut id_generator);
    let p1 = DatumPoint::new(&mut id_generator);
    let p2 = DatumPoint::new(&mut id_generator);
    let line0 = LineSegment::new(p0, p1);
    let line1 = LineSegment::new(p1, p2);
    let constraints = vec![
        // p0 is the origin
        Constraint::Fixed(p0.id_x(), 0.0),
        Constraint::Fixed(p0.id_y(), 0.0),
        // Both lines are parallel
        Constraint::lines_parallel([line0, line1]),
        // Both lines are the same distance
        Constraint::Distance(p0, p1, 32.0f64.sqrt()),
        Constraint::Distance(p1, p2, 32.0f64.sqrt()),
        Constraint::Fixed(p1.id_x(), 4.0),
    ];

    let initial_guesses = vec![
        (p0.id_x(), 0.0),
        (p0.id_y(), 0.0),
        (p1.id_x(), 3.0f64.sqrt()),
        (p1.id_y(), 3.0f64.sqrt()),
        (p2.id_x(), 6.0f64.sqrt()),
        (p2.id_y(), 6.0f64.sqrt()),
    ];

    let actual = solve(constraints, initial_guesses).unwrap();

    let expected = [(0.0, 0.0), (4.0, 4.0), (8.0, 8.0)];
    assert!(actual.iterations <= 10);
    let actual_points = [
        (actual.final_values[0], actual.final_values[1]),
        (actual.final_values[2], actual.final_values[3]),
        (actual.final_values[4], actual.final_values[5]),
    ];
    for i in 0..3 {
        assert!((expected[i].0 - actual_points[i].0).abs() < 0.000001);
        assert!((expected[i].1 - actual_points[i].1).abs() < 0.000001);
    }

    if std::env::var("GNUPLOT_EZPZ_TEST").is_ok() {
        pop_gnuplot_window(
            "angle test",
            vec![
                ((actual.final_values[0], actual.final_values[1]), "p0"),
                ((actual.final_values[2], actual.final_values[3]), "p1"),
                ((actual.final_values[4], actual.final_values[5]), "p2"),
            ],
        );
    }
}

/// Open a `gnuplot` window displaying these points in a 2D scatter plot.
fn pop_gnuplot_window(chart_name: &str, points: Vec<((f64, f64), &str)>) {
    let gnuplot_program = gnuplot(chart_name, points);
    let mut child = std::process::Command::new("gnuplot")
        .args(["-persist", "-"])
        .stdin(std::process::Stdio::piped())
        .spawn()
        .expect("failed to start gnuplot");

    {
        let stdin = child.stdin.as_mut().expect("failed to open stdin");
        use std::io::Write;
        stdin
            .write_all(gnuplot_program.as_bytes())
            .expect("failed to write to stdin");
    }
    let _ = child.wait();
}

/// Write a gnuplot program to show these points in a 2D scatter plot.
fn gnuplot(chart_name: &str, points: Vec<((f64, f64), &str)>) -> String {
    let all_points = points
        .iter()
        .map(|((x, y), _label)| format!("{x} {y}"))
        .collect::<Vec<_>>()
        .join("\n");
    let all_labels = points
        .iter()
        .map(|((x, y), label)| format!("set label \"{label} ({x}, {y})\" at {x},{y} offset 1,1"))
        .collect::<Vec<_>>()
        .join("\n");
    let components = points
        .into_iter()
        .flat_map(|((x, y), _label)| [x, y])
        .collect::<Vec<_>>();
    let min = components.iter().cloned().fold(f64::NAN, f64::min);
    let max = components.iter().cloned().fold(f64::NAN, f64::max);
    format!(
        "set title \"{chart_name}\"
set xlabel \"X\"
set ylabel \"Y\"
set grid

set xrange [{min}:{max}]
set yrange [{min}:{max}]

# Plot the points
plot \"-\" with points pointtype 7 pointsize 2 title \"Points\"
{all_points}
e

# Add labels for each point
{all_labels}

# Refresh plot to show labels
replot
"
    )
}
