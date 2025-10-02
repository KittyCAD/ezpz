use kcl_ezpz::textual::{Arc, Circle, Outcome};

const NORMAL_POINT: &str = "0x3C7A89";
const CIRCLE_COLOR: &str = "0x9FA2B2";
const RADIUS_COLOR: &str = "0x2E4756";
const ARC_COLOR: &str = "0x16262E";
// use DBC2CF next.

type PointToDraw = (f64, f64, &'static str);

use crate::Cli;

pub fn save_png(cli: &Cli, soln: &Outcome, output_path: String) {
    let mut gnuplot_program = String::new();
    let chart_name = cli.chart_name();
    let points = points_from_soln(soln);
    let circles = circles_from_soln(soln);
    let arcs = arcs_from_soln(soln);
    gnuplot_program.push_str(&gnuplot(&chart_name, points, circles, arcs, &output_path));
    gnuplot_program.push_str("unset output"); // closes file
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

fn points_from_soln(soln: &Outcome) -> Vec<(PointToDraw, String)> {
    let mut points: Vec<_> = soln
        .points
        .iter()
        .map(|(label, pt)| ((pt.x, pt.y, NORMAL_POINT), label.clone()))
        .collect();
    points.extend(soln.circles.iter().map(|(label, circle)| {
        (
            (circle.center.x, circle.center.y, CIRCLE_COLOR),
            format!("{}.center", label),
        )
    }));
    points
}

fn circles_from_soln(soln: &Outcome) -> Vec<(Circle, String)> {
    soln.circles
        .iter()
        .map(|(label, pt)| (*pt, label.clone()))
        .collect()
}

fn arcs_from_soln(soln: &Outcome) -> Vec<(Arc, String)> {
    soln.arcs
        .iter()
        .map(|(label, pt)| (*pt, label.clone()))
        .collect()
}

/// Write a gnuplot program to show these points in a 2D scatter plot.
fn gnuplot(
    chart_name: &str,
    points: Vec<(PointToDraw, String)>,
    circles: Vec<(Circle, String)>,
    arcs: Vec<(Arc, String)>,
    output_path: &str,
) -> String {
    let all_points = points
        .iter()
        .map(|((x, y, color), _label)| format!("{x:.2} {y:.2} {color}"))
        .chain(arcs.iter().flat_map(|arc| {
            let ax = arc.0.a.x;
            let ay = arc.0.a.y;
            let bx = arc.0.b.x;
            let by = arc.0.b.y;
            let cx = arc.0.center.x;
            let cy = arc.0.center.y;
            [
                format!("{ax:.2} {ay:.2} {ARC_COLOR}"),
                format!("{bx:.2} {by:.2} {ARC_COLOR}"),
                format!("{cx:.2} {cy:.2} {ARC_COLOR}"),
            ]
        }));

    let all_points = all_points.collect::<Vec<_>>().join("\n");
    let arc_labels = arcs.iter().flat_map(|(arc, label)| {
        let ax = arc.a.x;
        let ay = arc.a.y;
        let bx = arc.b.x;
        let by = arc.b.y;
        let cx = arc.center.x;
        let cy = arc.center.y;
        [
            format!("set label \"{label}.a\\n({ax:.2}, {ay:.2})\" at {ax:.2},{ay:.2} offset 1,1"),
            format!("set label \"{label}.b\\n({bx:.2}, {by:.2})\" at {bx:.2},{by:.2} offset 1,1"),
            format!(
                "set label \"{label}.center\\n({cx:.2}, {cy:.2})\" at {cx:.2},{cy:.2} offset 1,1"
            ),
        ]
    });
    let all_labels = points
        .iter()
        .map(|((x, y, _color), label)| {
            format!("set label \"{label}\\n({x:.2}, {y:.2})\" at {x:.2},{y:.2} offset 1,1")
        })
        .chain(arc_labels)
        .collect::<Vec<_>>()
        .join("\n");
    let all_circles: String = circles
        .iter()
        .enumerate()
        .map(|(i, (circ, _label))| {
            let cx=circ.center.x;
            let cy=circ.center.y;
            let radius=circ.radius;
            let i=i+1;
            format!("set object {i} circle at {cx},{cy} size first {radius} front lw 2 fc rgb {CIRCLE_COLOR} fs empty border rgb {CIRCLE_COLOR}\n")
        })
        .collect();
    let n = circles.len();
    let ratio = 0.8;
    let ratio2 = 1.0 - ratio;
    let all_radii: String = circles
        .iter()
        .enumerate()
        .map(|(i, (circ, label))| {
            let i = i + n + 1;
            let cx = circ.center.x;
            let cy = circ.center.y;
            let r = circ.radius;
            let theta = -10.0f64.to_radians();
            let px = cx + r * libm::cos(theta);
            let py = cy + r * libm::sin(theta);
            let mpx = cx*ratio2 + px*ratio;
            let mpy = cy*ratio2 + py*ratio;
            format!("set object {i} polygon from {cx},{cy} to {px},{py} lw 1 lc rgb {RADIUS_COLOR}\nset label \"{label}.radius\\n= {r:0.2}\" at {mpx},{mpy} center")
        })
        .collect();

    // Get the furthest X and Y component in each direction,
    // so we can establish the span of the graph.
    let (mut xs, mut ys): (Vec<_>, Vec<_>) =
        points.into_iter().map(|((x, y, _), _label)| (x, y)).unzip();
    for circle in circles {
        xs.push(circle.0.center.x + circle.0.radius);
        ys.push(circle.0.center.y + circle.0.radius);
        xs.push(circle.0.center.x - circle.0.radius);
        ys.push(circle.0.center.y - circle.0.radius);
    }
    for arc in &arcs {
        xs.push(arc.0.center.x);
        ys.push(arc.0.center.y);
        xs.push(arc.0.a.x);
        ys.push(arc.0.a.y);
        xs.push(arc.0.b.x);
        ys.push(arc.0.b.y);
    }
    let padding = 1.0;
    let min_x = xs.iter().cloned().fold(f64::NAN, f64::min) - padding;
    let max_x = xs.iter().cloned().fold(f64::NAN, f64::max) + padding;
    let min_y = ys.iter().cloned().fold(f64::NAN, f64::min) - padding;
    let max_y = ys.iter().cloned().fold(f64::NAN, f64::max) + padding;

    let display = format!(
        "set terminal pngcairo size 600,600 enhanced font 'Verdana,12'\nset output \"{output_path}\"\n"
    );

    format!(
        "\
{display}
# `noenhance` stops _ in path names being interpreted as subscript
set title \"Solution to {chart_name}\" noenhance 
set xlabel \"X\"
set ylabel \"Y\"
set grid
set size ratio -1
unset key

{all_circles}
{all_radii}

set xrange [{min_x}:{max_x}]
set yrange [{min_y}:{max_y}]

# Add labels for each point
{all_labels}

# Plot the points
plot \"-\" using 1:2:3 with points pointtype 7 pointsize 2 lc rgb variable title \"Points\"
{all_points}
e

# Refresh plot to show labels
replot
"
    )
}
