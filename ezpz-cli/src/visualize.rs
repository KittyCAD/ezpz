use std::f64::consts::PI;

use kcl_ezpz::textual::{Arc, Circle, Outcome, Point};
use plotters::{coord::types::RangedCoordf64, prelude::*};

const POINT_COLOR: RGBColor = RGBColor(0x58, 0x50, 0x8d);
const LINE_COLOR: RGBColor = RGBColor(0xff, 0xa6, 0x00);
const ARC_COLOR: RGBColor = RGBColor(0xff, 0x63, 0x61);
const CIRCLE_COLOR: RGBColor = RGBColor(0xbc, 0x50, 0x90);
// const LINE_COLOR: RGBColor = RGBColor(0xff, 0x63, 0x61);
// Others:
// #ffa600

use crate::Cli;

const LABEL_STYLE: (&str, i32) = ("sans-serif", 30);

pub fn save_png(cli: &Cli, soln: &Outcome, output_path: String) -> anyhow::Result<()> {
    let chart_name = cli.chart_name();
    let points = points_from_soln(soln);
    let circles = circles_from_soln(soln);
    let arcs = arcs_from_soln(soln);
    let lines = lines_from_soln(soln);
    let bounds = Bounds::new(&points, &circles, &arcs);

    let width = 800;
    let height = 800;
    let dpi_scale = 2;
    let root = BitMapBackend::new(&output_path, (width * dpi_scale, height * dpi_scale))
        .into_drawing_area();
    root.fill(&WHITE)?;

    // Build a Cartesian 2D chart from -10..10 on both axes
    let mut chart = ChartBuilder::on(&root)
        .margin(20)
        .x_label_area_size(40)
        .y_label_area_size(40)
        .caption(chart_name, ("sans-serif", 50))
        .build_cartesian_2d(bounds.min..bounds.max, bounds.min..bounds.max)?;

    draw_axes(&mut chart)?;

    // Draw the single points.
    for pt in points {
        draw_point(&mut chart, pt)?;
    }

    // Draw the circles
    for (Circle { radius, center }, label) in circles {
        draw_circle(&mut chart, center, radius, label)?;
    }

    // Draw the arcs
    for (Arc { a, b, center }, _label) in arcs {
        draw_arc(&mut chart, a, b, center, center.euclidean_distance(a))?;
    }

    // Draw the lines
    for line in lines {
        draw_line(&mut chart, line.0, line.1)?;
    }

    // Finished.
    root.present()?;
    println!("Plot saved to {output_path}");
    Ok(())
}

struct PointToDraw {
    point: Point,
    label: String,
    color: RGBColor,
}

fn points_from_soln(soln: &Outcome) -> Vec<PointToDraw> {
    let mut points: Vec<_> = soln
        .points
        .iter()
        .map(|(label, pt)| PointToDraw {
            point: *pt,
            label: label.clone(),
            color: POINT_COLOR,
        })
        .collect();
    points.extend(soln.circles.iter().map(|(label, circle)| PointToDraw {
        point: circle.center,
        label: format!("{}.center", label),
        color: CIRCLE_COLOR,
    }));
    points.extend(soln.arcs.iter().map(|(label, arc)| PointToDraw {
        point: arc.a,
        label: format!("{}.a", label),
        color: ARC_COLOR,
    }));
    points.extend(soln.arcs.iter().map(|(label, arc)| PointToDraw {
        point: arc.b,
        label: format!("{}.b", label),
        color: ARC_COLOR,
    }));
    points.extend(soln.arcs.iter().map(|(label, arc)| PointToDraw {
        point: arc.center,
        label: format!("{}.center", label),
        color: ARC_COLOR,
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

fn lines_from_soln(soln: &Outcome) -> Vec<(Point, Point)> {
    let mut out = Vec::new();
    for line in &soln.lines {
        let p0 = soln.points.get(&String::from(line.0.clone())).unwrap();
        let p1 = soln.points.get(&String::from(line.1.clone())).unwrap();
        out.push((*p0, *p1));
    }
    out
}

/// Span of the chart area
struct Bounds {
    min: f64,
    max: f64,
}

impl Bounds {
    pub fn new(
        points: &[PointToDraw],
        circles: &[(Circle, String)],
        arcs: &[(Arc, String)],
    ) -> Self {
        // Get the furthest X and Y component in each direction,
        // so we can establish the span of the graph.
        let (mut xs, mut ys): (Vec<_>, Vec<_>) =
            points.iter().map(|pt| (pt.point.x, pt.point.y)).unzip();
        for circle in circles {
            xs.push(circle.0.center.x + circle.0.radius);
            ys.push(circle.0.center.y + circle.0.radius);
            xs.push(circle.0.center.x - circle.0.radius);
            ys.push(circle.0.center.y - circle.0.radius);
        }
        for arc in arcs {
            xs.push(arc.0.center.x);
            ys.push(arc.0.center.y);
            xs.push(arc.0.a.x);
            ys.push(arc.0.a.y);
            xs.push(arc.0.b.x);
            ys.push(arc.0.b.y);
        }
        let padding = 1.0;
        let min_x = xs.iter().copied().reduce(f64::min).unwrap_or(0.0) - padding;
        let max_x = xs.iter().copied().reduce(f64::max).unwrap_or(0.0) + padding;
        let min_y = ys.iter().copied().reduce(f64::min).unwrap_or(0.0) - padding;
        let max_y = ys.iter().copied().reduce(f64::max).unwrap_or(0.0) + padding;
        let min = min_x.min(min_y);
        let max = max_x.max(max_y);
        Self { min, max }
    }
}

fn draw_circle<DB: DrawingBackend>(
    chart: &mut ChartContext<DB, Cartesian2d<RangedCoordf64, RangedCoordf64>>,
    center: Point,
    radius: f64,
    label: String,
) -> anyhow::Result<()>
where
    <DB as plotters::prelude::DrawingBackend>::ErrorType: 'static,
{
    // Map center to pixel coordinates
    let center_pixel = chart.plotting_area().map_coordinate(&(center.x, center.y));
    // Map a point at distance radius along x-axis to pixel coordinates
    let radius_pixel = chart
        .plotting_area()
        .map_coordinate(&(center.x + radius, center.y));
    let pixel_radius = (radius_pixel.0 - center_pixel.0).abs();
    chart.draw_series(std::iter::once(plotters::prelude::Circle::new(
        (center.x, center.y),
        pixel_radius,
        CIRCLE_COLOR.mix(0.3).filled(),
    )))?;
    chart.draw_series([Text::new(
        label,
        (center.x, center.y - radius / 2.0),
        LABEL_STYLE.into_font(),
    )])?;
    Ok(())
}

fn draw_axes<DB: DrawingBackend>(
    chart: &mut ChartContext<DB, Cartesian2d<RangedCoordf64, RangedCoordf64>>,
) -> anyhow::Result<()>
where
    <DB as plotters::prelude::DrawingBackend>::ErrorType: 'static,
{
    chart
        .configure_mesh()
        .label_style(LABEL_STYLE) // axis labels
        .axis_desc_style(LABEL_STYLE) // x/y axis captions
        .draw()?;

    // Overlay bold black axes at x=0 and y=0
    let x_range = chart.as_coord_spec().x_spec().to_owned();
    let y_range = chart.as_coord_spec().y_spec().to_owned();

    chart.draw_series(std::iter::once(PathElement::new(
        vec![(0.0, y_range.range().start), (0.0, y_range.range().end)],
        BLACK.stroke_width(3),
    )))?;

    chart.draw_series(std::iter::once(PathElement::new(
        vec![(x_range.range().start, 0.0), (x_range.range().end, 0.0)],
        BLACK.stroke_width(3),
    )))?;
    Ok(())
}

fn draw_point<DB: DrawingBackend>(
    chart: &mut ChartContext<DB, Cartesian2d<RangedCoordf64, RangedCoordf64>>,
    point: PointToDraw,
) -> anyhow::Result<()>
where
    <DB as plotters::prelude::DrawingBackend>::ErrorType: 'static,
{
    // println!(
    //     "Drawing point {} with color {:?}, near {:.2},{:.2}",
    //     point.label, point.color, point.point.x, point.point.y
    // );
    chart.draw_series(PointSeries::of_element(
        vec![(point.point.x, point.point.y)],
        5,
        &point.color,
        &|coord, size, style| {
            EmptyElement::at(coord)
                + plotters::prelude::Circle::new((0, 0), size, style.filled())
                + Text::new(point.label.clone(), (10, -10), LABEL_STYLE.into_font())
        },
    ))?;
    Ok(())
}

/// Draws a straight line between p0 and p1.
fn draw_line<DB: DrawingBackend>(
    chart: &mut ChartContext<DB, Cartesian2d<RangedCoordf64, RangedCoordf64>>,
    p0: Point,
    p1: Point,
) -> anyhow::Result<()>
where
    <DB as plotters::prelude::DrawingBackend>::ErrorType: 'static,
{
    let color = LINE_COLOR;
    chart.draw_series([PathElement::new(
        vec![(p0.x, p0.y), (p1.x, p1.y)],
        color.stroke_width(3),
    )])?;
    Ok(())
}

/// Draws a circular arc between p0 and p1. The circle's radius and center are given as params.
fn draw_arc<DB: DrawingBackend>(
    chart: &mut ChartContext<DB, Cartesian2d<RangedCoordf64, RangedCoordf64>>,
    p0: Point,
    p1: Point,
    center: Point,
    radius: f64,
) -> anyhow::Result<()>
where
    <DB as plotters::prelude::DrawingBackend>::ErrorType: 'static,
{
    let color = ARC_COLOR;
    // Bail out if radius is effectively zero; nothing sensible to render.
    if radius.abs() < f64::EPSILON {
        return Ok(());
    }

    let start_angle = libm::atan2(p0.y - center.y, p0.x - center.x);
    let potential_end = libm::atan2(p1.y - center.y, p1.x - center.x);
    let mut delta = potential_end - start_angle;

    // Normalize to the shortest signed delta in (-PI, PI].
    // Note this always draws the minor arc. Currently the arcs in EZPZ don't
    // track whether they're major or minor
    // (or equivalently, which point A or B is the start or end)
    while delta <= -PI {
        delta += 2.0 * PI;
    }
    while delta > PI {
        delta -= 2.0 * PI;
    }

    // Sample several straight lines along the arc.
    let interval_degrees = 2.0;
    let steps = (delta.abs() / (PI / (180.0 / interval_degrees))).ceil();
    let steps = (steps as usize).max(1);

    let points: Vec<_> = (0..=steps)
        .map(|step| {
            let t = step as f64 / steps as f64;
            let angle = start_angle + delta * t;
            let x = center.x + radius * libm::cos(angle);
            let y = center.y + radius * libm::sin(angle);
            (x, y)
        })
        .collect();

    chart.draw_series([PathElement::new(points, color.stroke_width(3))])?;
    Ok(())
}
