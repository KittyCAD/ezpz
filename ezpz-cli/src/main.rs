use std::{
    hint::black_box,
    io::{self, Read},
    time::Duration,
};

use kcl_ezpz::textual::{Outcome, Problem};

const NUM_ITERS_BENCHMARK: u32 = 100;

fn main() {
    if let Err(e) = main_inner().map(print_output) {
        eprintln!("Error: {e}");
        std::process::exit(1);
    }
}

fn main_inner() -> Result<(Outcome, Duration), String> {
    let constraint_txt = read_problem()?;
    let parsed = Problem::parse(&mut constraint_txt.as_str()).map_err(|e| e.to_string())?;

    // Ensure problem can be solved
    let now = std::time::Instant::now();
    let solved = parsed.solve().map_err(|e| e.to_string())?;

    // It succeeded. Benchmark its perf
    for _ in 0..NUM_ITERS_BENCHMARK {
        let _ = black_box(parsed.solve());
    }
    let elapsed = now.elapsed();
    let duration_per_iter = elapsed / NUM_ITERS_BENCHMARK;
    Ok((solved, duration_per_iter))
}

/// Prints the output nicely to stdout.
fn print_output((outcome, duration): (Outcome, Duration)) {
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
    println!("Problem size: {num_eqs} rows, {num_vars} vars");
    println!("Iterations needed: {iterations}");
    println!(
        "Solved in {}us (mean over {NUM_ITERS_BENCHMARK} iterations)",
        duration.as_micros()
    );
    println!("Points:");
    for point in points {
        println!("\t{}: ({}, {})", point.0, point.1.x, point.1.y);
    }
}

/// Read the EZPZ problem text from a file or stdin, depending on user args.
/// They pass a filename, or '-' for stdin, as the first CLI arg.
fn read_problem() -> Result<String, String> {
    let mut args = std::env::args();
    let _this_program = args.next().unwrap();
    let dst = args
        .next()
        .ok_or("usage: first arg must be a path to an EZPZ problem text file, or '-' for stdin.")?;
    if dst == "-" {
        let mut constraint_txt = String::with_capacity(100);
        let mut stdin = io::stdin();
        stdin
            .read_to_string(&mut constraint_txt)
            .map_err(|e| e.to_string())?;
        Ok(constraint_txt)
    } else {
        std::fs::read_to_string(dst).map_err(|e| e.to_string())
    }
}
