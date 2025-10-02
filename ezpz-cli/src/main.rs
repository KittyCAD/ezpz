use std::{
    hint::black_box,
    io::{self, Read},
    path::PathBuf,
    str::FromStr,
    time::Duration,
};

use clap::Parser;
use kcl_ezpz::{
    FailureOutcome, Lint,
    textual::{Arc, Circle, Outcome, Point, Problem},
};

const NUM_ITERS_BENCHMARK: u32 = 100;
const NORMAL_POINT: &str = "0x3C7A89";
const CIRCLE_COLOR: &str = "0x9FA2B2";
const RADIUS_COLOR: &str = "0x2E4756";
const ARC_COLOR: &str = "0x16262E";
// use DBC2CF next.

type PointToDraw = (f64, f64, &'static str);

#[derive(Parser)]
#[command(name="ezpz", version, about, long_about = None)]
struct Cli {
    /// Path to the problem file.
    /// Use '-' for stdin.
    #[arg(short = 'f', long)]
    filepath: PathBuf,

    /// Open the results in gnuplot if solve was successful.
    #[arg(long, default_value_t = false)]
    gnuplot: bool,

    /// Save results as a PNG if solve was successful.
    #[arg(short = 'o', long = "gnuplot-png-path")]
    gnuplot_png_path: Option<PathBuf>,

    /// Show the final values assigned to each point.
    #[arg(long = "show-points")]
    show_points: bool,
}

impl Cli {
    fn chart_name(&self) -> String {
        if self.filepath.display().to_string() == "-" {
            "EZPZ".to_owned()
        } else {
            self.filepath.display().to_string()
        }
    }
}

fn main() {
    let cli = Cli::parse();
    let soln = match main_inner(&cli) {
        Ok(soln) => soln,
        Err(e) => {
            eprintln!("Error: {e}");
            std::process::exit(1);
        }
    };
    let soln = match soln {
        Ok(o) => o,
        Err(outcome) => {
            print_failure_output(outcome);
            std::process::exit(1);
        }
    };
    handle_output(soln, cli)
}

fn handle_output(soln: (Outcome, Duration), cli: Cli) {
    print_output(&soln, cli.show_points);
    if let Some(ref p) = cli.gnuplot_png_path {
        let output_path = p.display().to_string();
        save_gnuplot_png(&cli, &soln.0, output_path);
    }
    if cli.gnuplot {
        pop_gnuplot_window(&cli, &soln.0);
    }
}

fn save_gnuplot_png(cli: &Cli, soln: &Outcome, output_path: String) {
    let mut gnuplot_program = String::new();
    let chart_name = cli.chart_name();
    let points = points_from_soln(soln);
    let circles = circles_from_soln(soln);
    let arcs = arcs_from_soln(soln);
    gnuplot_program.push_str(&gnuplot(
        &chart_name,
        points,
        circles,
        arcs,
        GnuplotMode::WriteFile(output_path),
    ));
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

/// Open a `gnuplot` window displaying these points in a 2D scatter plot.
fn pop_gnuplot_window(cli: &Cli, soln: &Outcome) {
    let chart_name = cli.chart_name();
    let points = points_from_soln(soln);
    let circles = circles_from_soln(soln);
    let arcs = arcs_from_soln(soln);
    let gnuplot_program = gnuplot(&chart_name, points, circles, arcs, GnuplotMode::PopWindow);
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

enum GnuplotMode {
    PopWindow,
    WriteFile(String),
}

/// Write a gnuplot program to show these points in a 2D scatter plot.
fn gnuplot(
    chart_name: &str,
    points: Vec<(PointToDraw, String)>,
    circles: Vec<(Circle, String)>,
    arcs: Vec<(Arc, String)>,
    mode: GnuplotMode,
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

    let display = match mode {
        GnuplotMode::PopWindow => "set term qt font \"Verdana\"\n".to_owned(),
        GnuplotMode::WriteFile(output_path) => format!(
            "set terminal pngcairo size 600,600 enhanced font 'Verdana,12'\nset output \"{output_path}\"\n"
        ),
    };
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

type RunResult = Result<(Outcome, Duration), FailureOutcome>;

fn main_inner(cli: &Cli) -> Result<RunResult, String> {
    let constraint_txt = read_problem(cli)?;
    let parsed = Problem::from_str(&constraint_txt)?;

    // Ensure problem can be solved
    let now = std::time::Instant::now();
    let constraint_system = parsed.to_constraint_system().map_err(|e| e.to_string())?;
    let solved = match constraint_system.solve() {
        Ok(o) => o,
        Err(e) => return Ok(Err(e)),
    };

    // It succeeded. Benchmark its perf
    let constraint_system = parsed.to_constraint_system().map_err(|e| e.to_string())?;
    for _ in 0..NUM_ITERS_BENCHMARK {
        black_box(constraint_system.solve()).unwrap();
    }
    let elapsed = now.elapsed();
    let duration_per_iter = elapsed / NUM_ITERS_BENCHMARK;
    Ok(Ok((solved, duration_per_iter)))
}

/// Prints the output nicely to stdout.
fn print_output((outcome, duration): &(Outcome, Duration), show_points: bool) {
    let Outcome {
        iterations,
        lints,
        points,
        circles,
        arcs,
        num_vars,
        num_eqs,
    } = outcome;
    print_lints(lints);
    print_problem_size(*num_vars, *num_eqs);
    println!("Iterations needed: {iterations}");
    print_performance(*duration);
    if show_points {
        println!("Points:");
        for (label, Point { x, y }) in points {
            println!("\t{label}: ({x:.2}, {y:.2})",);
        }
        println!("Circles:");
        for (label, kcl_ezpz::textual::Circle { radius, center }) in circles {
            let Point { x, y } = center;
            println!("\t{label}: center = ({x:.2}, {y:.2}), radius = {radius:.2}",);
        }
        println!("Arcs:");
        for (label, kcl_ezpz::textual::Arc { a, b, center }) in arcs {
            let Point { x, y } = center;
            let ax = a.x;
            let ay = a.y;
            let bx = b.x;
            let by = b.y;
            println!(
                "\t{label}: center = ({x:.2}, {y:.2}), a = ({ax:.2}, {ay:.2}), b = ({bx:.2}, {by:.2})",
            );
        }
    }
}

fn print_performance(duration: Duration) {
    use colored::Colorize;
    let time = format!("{}μs", duration.as_micros());
    println!("Solved in {time} (mean over {NUM_ITERS_BENCHMARK} iterations)");
    let solves_per_second = Duration::from_secs(1).as_micros() / duration.as_micros();
    let solves_per_second = if solves_per_second <= 60 {
        solves_per_second.to_string().red()
    } else {
        solves_per_second.to_string().normal()
    };
    println!("i.e. {solves_per_second} solves per second");
}

fn print_lints(lints: &[Lint]) {
    use colored::Colorize;
    if !lints.is_empty() {
        println!("Lints:");
        for lint in lints {
            println!("\t{}", lint.content.yellow());
        }
    }
}

fn print_problem_size(num_vars: usize, num_eqs: usize) {
    use colored::Colorize;
    print!("Problem size: ");
    if num_vars != num_eqs {
        let l = format!("{num_eqs} rows, {num_vars} vars");
        println!("{}", l.yellow());
    } else {
        println!("{num_eqs} rows, {num_vars} vars");
    }
}

fn print_failure_output(outcome: FailureOutcome) {
    use colored::Colorize;
    let FailureOutcome {
        error,
        lints,
        num_vars,
        num_eqs,
    } = outcome;
    print_lints(&lints);
    print_problem_size(num_vars, num_eqs);
    eprintln!("{}: {}", "Could not solve system".red(), error);
    if num_eqs > num_vars {
        eprintln!("Your system might be overconstrained. Try removing constraints.");
    } else {
        eprintln!("You might have contradictory constraints.");
    }
}

/// Read the EZPZ problem text from a file or stdin, depending on user args.
/// They pass a filename, or '-' for stdin, as the first CLI arg.
fn read_problem(cli: &Cli) -> Result<String, String> {
    // Read from file
    if cli.filepath != PathBuf::from("-") {
        return std::fs::read_to_string(&cli.filepath).map_err(|e| e.to_string());
    }

    // Read from stdin
    let mut constraint_txt = String::with_capacity(100);
    let mut stdin = io::stdin();
    stdin
        .read_to_string(&mut constraint_txt)
        .map_err(|e| e.to_string())?;
    Ok(constraint_txt)
}

#[cfg(test)]
mod tests {
    use std::process::{Command, Stdio};

    use crate::{Cli, handle_output, main_inner};

    #[test]
    fn test_tiny_inner() {
        for case in ["tiny", "arc_radius", "circle"] {
            let cli = Cli {
                filepath: format!("../test_cases/{case}/problem.txt").into(),
                gnuplot: Default::default(),
                gnuplot_png_path: Some("test_image.png".into()),
                show_points: true,
            };
            let soln = main_inner(&cli).unwrap().unwrap();
            handle_output(soln, cli);
        }
    }

    #[test]
    fn test_tiny() {
        let out = Command::new("cargo")
            .args([
                "run",
                "--quiet",
                "--",
                "-f",
                "../test_cases/tiny/problem.txt",
            ])
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap()
            .wait_with_output()
            .unwrap();
        assert!(out.status.success());
        let stdout = String::from_utf8(out.stdout).unwrap();
        assert!(stdout.contains("Problem size: 4 rows, 4 vars"));
    }

    #[test]
    fn test_arc() {
        let out = Command::new("cargo")
            .args([
                "run",
                "--quiet",
                "--",
                "-f",
                "../test_cases/arc_radius/problem.txt",
            ])
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap()
            .wait_with_output()
            .unwrap();
        assert!(out.status.success());
        let stdout = String::from_utf8(out.stdout).unwrap();
        assert!(stdout.contains("Problem size: 4 rows, 8 vars"));
    }
}
