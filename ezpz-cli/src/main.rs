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
    textual::{Circle, Outcome, Point, Problem},
};

const NUM_ITERS_BENCHMARK: u32 = 100;
const NORMAL_POINT: &str = "0x6CA6C1";
const CIRCLE_POINT: &str = "0x000000";

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
    gnuplot_program.push_str(&gnuplot(
        &chart_name,
        points,
        circles,
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
            (circle.center.x, circle.center.y, CIRCLE_POINT),
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

/// Open a `gnuplot` window displaying these points in a 2D scatter plot.
fn pop_gnuplot_window(cli: &Cli, soln: &Outcome) {
    let chart_name = cli.chart_name();
    let points = points_from_soln(soln);
    let circles = circles_from_soln(soln);
    let gnuplot_program = gnuplot(&chart_name, points, circles, GnuplotMode::PopWindow);
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
    mode: GnuplotMode,
) -> String {
    let all_points = points
        .iter()
        .map(|((x, y, color), _label)| format!("{x:.2} {y:.2} {color}"))
        .collect::<Vec<_>>()
        .join("\n");
    let all_labels = points
        .iter()
        .map(|((x, y, _color), label)| {
            format!("set label \"{label} ({x:.2}, {y:.2})\" at {x:.2},{y:.2} offset 1,1")
        })
        .collect::<Vec<_>>()
        .join("\n");
    let circles: String = circles
        .into_iter()
        .enumerate()
        .map(|(i, (circ, _label))| {
            let cx=circ.center.x;
            let cy=circ.center.y;
            let radius=circ.radius;
            let i=i+1;
            format!("set object {i} circle at {cx},{cy} size {radius} front lw 2 lc rgb {CIRCLE_POINT} fillstyle empty\n")
        })
        .collect();
    let components = points
        .into_iter()
        .flat_map(|((x, y, _), _label)| [x, y])
        .collect::<Vec<_>>();
    let min = components.iter().cloned().fold(f64::NAN, f64::min) - 1.0;
    let max = components.iter().cloned().fold(f64::NAN, f64::max) + 1.0;
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
set title \"{chart_name}\" noenhance 
set xlabel \"X\"
set ylabel \"Y\"
set grid
unset key

{circles}

set xrange [{min}:{max}]
set yrange [{min}:{max}]

# Plot the points
plot \"-\" using 1:2:3 with points pointtype 7 pointsize 2 lc rgb variable title \"Points\"
{all_points}
e

# Add labels for each point
{all_labels}

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
        error: _,
        lints,
        num_vars,
        num_eqs,
    } = outcome;
    print_lints(&lints);
    print_problem_size(num_vars, num_eqs);
    eprintln!("{}", "Could not solve system".red());
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
