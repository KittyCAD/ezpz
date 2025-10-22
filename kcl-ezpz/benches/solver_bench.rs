use std::{hint::black_box, str::FromStr};

use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use kcl_ezpz::{
    Config, Constraint, IdGenerator,
    datatypes::{DatumPoint, LineSegment},
    solve,
    textual::Problem,
};
use newton_faer::init_global_parallelism;

/// General benchmark template.
/// Opens a given test case from the test_cases/ dir,
/// benchmarks solving it.
fn bench_case(c: &mut Criterion, test_case: &'static str) {
    let txt = std::fs::read_to_string(format!("test_cases/{test_case}/problem.md")).unwrap();
    c.bench_function(&format!("solve_{test_case}"), |b| {
        let problem = Problem::from_str(txt.as_str()).unwrap();
        let constraints = problem.to_constraint_system().unwrap();
        b.iter(|| {
            let _actual = black_box(constraints.solve_no_metadata(Config::default()).unwrap());
        });
    });
}

fn solve_tiny(c: &mut Criterion) {
    bench_case(c, "tiny")
}

fn solve_inconsistent(c: &mut Criterion) {
    bench_case(c, "inconsistent")
}

fn solve_two_rectangles(c: &mut Criterion) {
    bench_case(c, "two_rectangles")
}

fn solve_angle_parallel(c: &mut Criterion) {
    bench_case(c, "angle_parallel")
}

fn solve_angle_parallel_manual(c: &mut Criterion) {
    bench_case(c, "angle_parallel_manual")
}

fn solve_nonsquare(c: &mut Criterion) {
    bench_case(c, "nonsquare")
}

fn solve_perpendicular(c: &mut Criterion) {
    bench_case(c, "perpendicular")
}

/// Just like `solve_two_rectangles`, except that the rectangles
/// depend on each other.
fn solve_two_rectangles_dependent(c: &mut Criterion) {
    let mut id_generator = IdGenerator::default();
    init_global_parallelism(1);
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
    c.bench_function("solve two rectangles dependent", |b| {
        b.iter(|| {
            let _actual = black_box(
                solve(
                    &constraints.clone(),
                    initial_guesses.clone(),
                    Config::default(),
                )
                .unwrap(),
            );
        })
    });
}

fn solve_massive(c: &mut Criterion) {
    run_massive(c, false)
}

fn solve_massive_overconstrained(c: &mut Criterion) {
    run_massive(c, true)
}

fn run_massive(c: &mut Criterion, overconstrained: bool) {
    let mut group = c.benchmark_group(format!(
        "massively_parallel{}",
        if overconstrained {
            "_overconstrained"
        } else {
            ""
        }
    ));
    for num_lines in [50, 150].iter() {
        // Each line has 2 points, each point has two variables (x and y)
        // So each line is 4 variables, and that is the relevant throughput metric.
        let size = num_lines * 4;
        std::process::Command::new("just")
            .args([
                if overconstrained {
                    "regen-massive-test-overconstrained"
                } else {
                    "regen-massive-test"
                },
                &size.to_string(),
            ])
            .spawn()
            .unwrap()
            .wait()
            .unwrap();
        group.throughput(Throughput::Elements(size));
        group.bench_with_input(BenchmarkId::from_parameter(size), &size, |b, _size| {
            let txt =
                std::fs::read_to_string("test_cases/massive_parallel_system/problem.md").unwrap();
            let t = txt.as_str();
            let problem = Problem::from_str(t).unwrap();
            let constraints = problem.to_constraint_system().unwrap();
            b.iter(|| {
                let _actual = black_box(constraints.solve_no_metadata(Config::default()).unwrap());
            });
        });
    }
    group.finish();
}

criterion_group!(
    benches,
    solve_tiny,
    solve_inconsistent,
    solve_nonsquare,
    solve_two_rectangles,
    solve_two_rectangles_dependent,
    solve_angle_parallel,
    solve_angle_parallel_manual,
    solve_perpendicular,
    solve_massive,
    solve_massive_overconstrained,
);
criterion_main!(benches);
