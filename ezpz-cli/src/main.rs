use std::{
    hint::black_box,
    io::{self, Read},
    path::PathBuf,
    time::Duration,
};

use clap::Parser;
use kcl_ezpz::textual::{Outcome, Point, Problem};

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
    #[arg(short = 'o')]
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

const NUM_ITERS_BENCHMARK: u32 = 100;

fn main() {
    let cli = Cli::parse();
    let soln = match main_inner(&cli) {
        Ok(soln) => soln,
        Err(e) => {
            eprintln!("Error: {e}");
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
    let points = soln
        .points
        .iter()
        .map(|(label, pt)| ((pt.x, pt.y), label.as_str()))
        .collect();
    gnuplot_program.push_str(&gnuplot(
        &chart_name,
        points,
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

/// Open a `gnuplot` window displaying these points in a 2D scatter plot.
fn pop_gnuplot_window(cli: &Cli, soln: &Outcome) {
    let chart_name = cli.chart_name();
    let points = soln
        .points
        .iter()
        .map(|(label, pt)| ((pt.x, pt.y), label.as_str()))
        .collect();
    let gnuplot_program = gnuplot(&chart_name, points, GnuplotMode::PopWindow);
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
fn gnuplot(chart_name: &str, points: Vec<((f64, f64), &str)>, mode: GnuplotMode) -> String {
    let all_points = points
        .iter()
        .map(|((x, y), _label)| format!("{x:.2} {y:.2}"))
        .collect::<Vec<_>>()
        .join("\n");
    let all_labels = points
        .iter()
        .map(|((x, y), label)| {
            format!("set label \"{label} ({x:.2}, {y:.2})\" at {x:.2},{y:.2} offset 1,1")
        })
        .collect::<Vec<_>>()
        .join("\n");
    let components = points
        .into_iter()
        .flat_map(|((x, y), _label)| [x, y])
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

fn main_inner(cli: &Cli) -> Result<(Outcome, Duration), String> {
    let constraint_txt = read_problem(cli)?;
    let parsed = Problem::parse(&mut constraint_txt.as_str()).map_err(|e| e.to_string())?;

    // Ensure problem can be solved
    let now = std::time::Instant::now();
    let constraint_system = parsed.to_constraint_system().map_err(|e| e.to_string())?;
    let solved = constraint_system.solve().map_err(|e| e.to_string())?;

    // It succeeded. Benchmark its perf
    for _ in 0..NUM_ITERS_BENCHMARK {
        let constraint_system = parsed.to_constraint_system().map_err(|e| e.to_string())?;
        let _ = black_box(constraint_system.solve().map_err(|e| e.to_string()))?;
    }
    let elapsed = now.elapsed();
    let duration_per_iter = elapsed / NUM_ITERS_BENCHMARK;
    Ok((solved, duration_per_iter))
}

/// Prints the output nicely to stdout.
fn print_output((outcome, duration): &(Outcome, Duration), show_points: bool) {
    use colored::Colorize;
    let Outcome {
        iterations,
        lints,
        points,
        num_vars,
        num_eqs,
    } = outcome;
    if !lints.is_empty() {
        println!("Lints:");
        for lint in lints {
            println!("\t{}", lint.content);
        }
    }
    print!("Problem size: ");
    if num_vars != num_eqs {
        let l = format!("{num_eqs} rows, {num_vars} vars");
        println!("{}", l.yellow());
    } else {
        println!("{num_eqs} rows, {num_vars} vars");
    }
    println!("Iterations needed: {iterations}");
    let time = format!("{}μs", duration.as_micros());
    println!("Solved in {time} (mean over {NUM_ITERS_BENCHMARK} iterations)");
    let solves_per_second = Duration::from_secs(1).as_micros() / duration.as_micros();
    let solves_per_second = if solves_per_second <= 60 {
        solves_per_second.to_string().red()
    } else {
        solves_per_second.to_string().normal()
    };
    println!("i.e. {solves_per_second} solves per second");
    if show_points {
        println!("Points:");
        for (label, Point { x, y }) in points {
            println!("\t{label}: ({x:.2}, {y:.2})",);
        }
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
