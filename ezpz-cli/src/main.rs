use std::{
    hint::black_box,
    io::{self, Read},
    path::PathBuf,
    str::FromStr,
    time::Duration,
};

use clap::Parser;
use kcl_ezpz::{
    Constraint, FailureOutcome, Warning,
    textual::{Outcome, Point, Problem},
};

mod visualize;

const NUM_ITERS_BENCHMARK: u32 = 100;

#[derive(Parser)]
#[command(name="ezpz", version, about, long_about = None)]
struct Cli {
    /// Path to the problem file.
    /// Use '-' for stdin.
    #[arg(short = 'f', long)]
    filepath: PathBuf,

    /// Save results as a PNG if solve was successful.
    #[arg(short = 'o', long = "image-path")]
    image_path: Option<String>,

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
    if let Err(e) = handle_output(soln, cli) {
        eprintln!("Error: {e}");
        std::process::exit(1);
    }
}

fn handle_output(soln: RunOutcome, cli: Cli) -> anyhow::Result<()> {
    print_output(&soln, cli.show_points);
    if let Some(ref p) = cli.image_path {
        let output_path = p.to_string();
        visualize::save_png(&cli, &soln.0, output_path)?;
    }
    Ok(())
}

type RunOutcome = (Outcome, Duration, Vec<Constraint>);
type RunResult = Result<RunOutcome, FailureOutcome>;

fn main_inner(cli: &Cli) -> Result<RunResult, String> {
    let constraint_txt = read_problem(cli)?;
    let parsed = Problem::from_str(&constraint_txt)?;

    // Ensure problem can be solved
    let now = std::time::Instant::now();
    let constraint_system = parsed.to_constraint_system().map_err(|e| e.to_string())?;
    let constraints = constraint_system.constraints.clone();
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
    Ok(Ok((solved, duration_per_iter, constraints)))
}

/// Prints the output nicely to stdout.
fn print_output((outcome, duration, constraints): &RunOutcome, show_points: bool) {
    let Outcome {
        iterations,
        warnings,
        points,
        circles,
        arcs,
        num_vars,
        num_eqs,
        lines: _, // these are only used for visuals
        unsatisfied,
    } = outcome;
    print_warnings(warnings);
    print_unsatisfied(unsatisfied, constraints);
    print_problem_size(*num_vars, *num_eqs);
    println!("Iterations needed: {iterations}");
    print_performance(*duration);
    if show_points {
        println!("Points:");
        for (label, Point { x, y }) in points {
            println!("\t{label}: ({x:.2}, {y:.2})",);
        }
        if !circles.is_empty() {
            println!("Circles:");
            for (label, kcl_ezpz::textual::Circle { radius, center }) in circles {
                let Point { x, y } = center;
                println!("\t{label}: center = ({x:.2}, {y:.2}), radius = {radius:.2}",);
            }
        }
        if !arcs.is_empty() {
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

fn print_warnings(warnings: &[Warning]) {
    use colored::Colorize;
    if !warnings.is_empty() {
        println!("Warnings:");
        for lint in warnings {
            println!("\t{}", lint.content.to_string().yellow());
        }
    }
}

fn print_unsatisfied(unsatisfied: &[usize], constraints: &[Constraint]) {
    use colored::Colorize;
    if !unsatisfied.is_empty() {
        let err = "Not all constraints were satisfied:".red();
        println!("{err}");
        for constraint_index in unsatisfied {
            let constraint = constraints[*constraint_index];
            println!("\t{constraint_index}: {constraint:?}");
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
        warnings,
        num_vars,
        num_eqs,
    } = outcome;
    print_warnings(&warnings);
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
    if &cli.filepath != "-" {
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
                filepath: format!("../test_cases/{case}/problem.md").into(),
                image_path: Some("test_image.png".to_owned()),
                show_points: true,
            };
            let soln = main_inner(&cli).unwrap().unwrap();
            handle_output(soln, cli).unwrap();
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
                "../test_cases/tiny/problem.md",
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
                "../test_cases/arc_radius/problem.md",
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
