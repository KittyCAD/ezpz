//! Residual field visualization for constraints.
//!
//! Renders the residual as a 2D scalar field (e.g. over x,y) and saves as an image,
//! useful as a sanity check when changing residual math: the image should change.

use crate::constraints::Constraint;
use crate::datatypes::inputs::{DatumLineSegment, DatumPoint};
use crate::solver::{Config, Layout};
use std::io;
use std::path::Path;

/// Residual magnitude below this is drawn as turquoise (zero/satisfied).
const ZERO_RESIDUAL_THRESHOLD: f64 = 0.08;

/// Turquoise color for the zero-residual locus (R, G, B).
const TURQUOISE: [u8; 3] = [64, 224, 208];

/// Example point (world coords) for PointsCoincident: red = current, green = solved-to (the fixed point).
const EXAMPLE_POINT_X: f64 = 3.0;
const EXAMPLE_POINT_Y: f64 = 2.0;

/// Example point for Distance viz; further out so red and green don't sit on top of each other.
const DISTANCE_EXAMPLE_POINT_X: f64 = 4.5;
const DISTANCE_EXAMPLE_POINT_Y: f64 = 3.0;

/// Example point for PointLineDistance (perpendicular distance) viz; far enough from solution so red/green aren’t cramped.
const PERP_DISTANCE_EXAMPLE_POINT_X: f64 = -2.0;
const PERP_DISTANCE_EXAMPLE_POINT_Y: f64 = 5.0;

/// Example point for Vertical/Horizontal viz.
const VERTICAL_HORIZONTAL_EXAMPLE_POINT_X: f64 = 3.0;
const VERTICAL_HORIZONTAL_EXAMPLE_POINT_Y: f64 = 2.0;

const RING_SCALE: f64 = 1.0;

/// World-space viewport and pixel dimensions for consistent coordinate conversion.
struct Viewport {
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    width: u32,
    height: u32,
}

impl Viewport {
    fn new(x_min: f64, x_max: f64, y_min: f64, y_max: f64, width: u32, height: u32) -> Self {
        Self {
            x_min,
            x_max,
            y_min,
            y_max,
            width,
            height,
        }
    }

    fn pixel_center_to_world(&self, px: u32, py: u32) -> (f64, f64) {
        let x = self.x_min + (self.x_max - self.x_min) * (px as f64 + 0.5) / (self.width as f64);
        let y = self.y_min + (self.y_max - self.y_min) * (py as f64 + 0.5) / (self.height as f64);
        (x, y)
    }

    /// World (x, y) to pixel (px, py). Must match pixel_center_to_world: top of image = y_min, bottom = y_max.
    fn world_to_pixel(&self, x: f64, y: f64) -> (i32, i32) {
        let px = (x - self.x_min) / (self.x_max - self.x_min) * (self.width as f64);
        let py = (y - self.y_min) / (self.y_max - self.y_min) * (self.height as f64);
        (px.round() as i32, py.round() as i32)
    }
}

fn mag_to_pixel(mag: f64) -> image::Rgb<u8> {
    if mag < ZERO_RESIDUAL_THRESHOLD {
        image::Rgb(TURQUOISE)
    } else {
        let value = mag * RING_SCALE;
        let fractional = value - value.trunc();
        let intensity = (255.0 - fractional * 255.0).round() as u8;
        image::Rgb([intensity, intensity, intensity])
    }
}

fn draw_filled_circle(buf: &mut image::RgbImage, cx: i32, cy: i32, radius_px: i32, color: [u8; 3]) {
    let w = buf.width() as i32;
    let h = buf.height() as i32;
    for dy in -radius_px..=radius_px {
        for dx in -radius_px..=radius_px {
            if dx * dx + dy * dy <= radius_px * radius_px {
                let px = cx + dx;
                let py = cy + dy;
                if px >= 0 && px < w && py >= 0 && py < h {
                    buf.put_pixel(px as u32, py as u32, image::Rgb(color));
                }
            }
        }
    }
}

fn draw_line_segment(
    buf: &mut image::RgbImage,
    x0: i32,
    y0: i32,
    x1: i32,
    y1: i32,
    color: [u8; 3],
) {
    let w = buf.width() as i32;
    let h = buf.height() as i32;
    let dx = (x1 - x0).abs();
    let dy = (y1 - y0).abs();
    let steps = (dx.max(dy)).max(1);
    for i in 0..=steps {
        let t = (i as f64) / (steps as f64);
        let px = (x0 as f64 + (x1 - x0) as f64 * t).round() as i32;
        let py = (y0 as f64 + (y1 - y0) as f64 * t).round() as i32;
        if px >= 0 && px < w && py >= 0 && py < h {
            buf.put_pixel(px as u32, py as u32, image::Rgb(color));
        }
    }
}

/// Draws an arrow from (from_px, from_py) toward (to_px, to_py), but only length_fraction of the
/// full distance (e.g. 0.5 = half length) so the arrow doesn't sit under the green dot.
fn draw_arrow(
    buf: &mut image::RgbImage,
    from_px: i32,
    from_py: i32,
    to_px: i32,
    to_py: i32,
    color: [u8; 3],
    head_size_px: i32,
    length_fraction: f64,
) {
    let w = buf.width() as i32;
    let h = buf.height() as i32;
    let dx = to_px - from_px;
    let dy = to_py - from_py;
    let len = libm::hypot(dx as f64, dy as f64);
    if len < 1.0 {
        return;
    }
    let ux = dx as f64 / len;
    let uy = dy as f64 / len;
    let actual_len = len * length_fraction;
    let tip_px = from_px + (ux * actual_len).round() as i32;
    let tip_py = from_py + (uy * actual_len).round() as i32;
    let steps = (actual_len as i32).max(2);
    for i in 0..=steps {
        let t = (i as f64) / (steps as f64);
        let px = from_px + (ux * actual_len * t).round() as i32;
        let py = from_py + (uy * actual_len * t).round() as i32;
        if px >= 0 && px < w && py >= 0 && py < h {
            buf.put_pixel(px as u32, py as u32, image::Rgb(color));
        }
    }
    let back_px = tip_px - (ux * (head_size_px as f64)).round() as i32;
    let back_py = tip_py - (uy * (head_size_px as f64)).round() as i32;
    let perp_x = (-uy * (head_size_px as f64 * 0.6)).round() as i32;
    let perp_y = (ux * (head_size_px as f64 * 0.6)).round() as i32;
    let c1x = back_px + perp_x;
    let c1y = back_py + perp_y;
    let c2x = back_px - perp_x;
    let c2y = back_py - perp_y;
    draw_line_segment(buf, tip_px, tip_py, c1x, c1y, color);
    draw_line_segment(buf, tip_px, tip_py, c2x, c2y, color);
    draw_line_segment(buf, c1x, c1y, c2x, c2y, color);
}

/// Renders a 2D residual field by sampling magnitude at each pixel; turquoise where near zero,
/// otherwise ring-style grayscale. Caller provides a closure that returns residual magnitude at (x, y).
fn render_residual_field<F>(viewport: &Viewport, mut sample: F) -> image::RgbImage
where
    F: FnMut(f64, f64) -> f64,
{
    let mut buf = image::RgbImage::new(viewport.width, viewport.height);
    for py in 0..viewport.height {
        for px in 0..viewport.width {
            let (x, y) = viewport.pixel_center_to_world(px, py);
            let mag = sample(x, y);
            buf.put_pixel(px, py, mag_to_pixel(mag));
        }
    }
    buf
}

/// Draws red example point, green solution point, and arrow (half length) between them.
fn draw_solver_overlay(
    buf: &mut image::RgbImage,
    viewport: &Viewport,
    example_x: f64,
    example_y: f64,
    solution_x: f64,
    solution_y: f64,
) {
    let (ex_px, ex_py) = viewport.world_to_pixel(example_x, example_y);
    let (sol_px, sol_py) = viewport.world_to_pixel(solution_x, solution_y);
    draw_arrow(buf, ex_px, ex_py, sol_px, sol_py, [200, 0, 0], 6, 0.5);
    draw_filled_circle(buf, ex_px, ex_py, 5, [255, 0, 0]);
    draw_filled_circle(buf, sol_px, sol_py, 5, [0, 180, 0]);
}

/// Renders the residual field for a "point coincident with fixed point" constraint
/// into an image buffer. One point is fixed at `(fixed_x, fixed_y)`; the other is
/// varied over the grid. Residual is (dx, dy); we plot magnitude (concentric rings).
/// Near-zero residual is drawn in turquoise.
pub fn render_points_coincident_residual_to_image(
    fixed_x: f64,
    fixed_y: f64,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    width: u32,
    height: u32,
) -> image::RgbImage {
    let viewport = Viewport::new(x_min, x_max, y_min, y_max, width, height);
    let p0 = DatumPoint::new_xy(0, 1);
    let p1 = DatumPoint::new_xy(2, 3);
    let constraint = Constraint::PointsCoincident(p0, p1);
    let layout = Layout::new(&[0, 1, 2, 3], &[&constraint], Config::default());
    let mut assignments = [0.0_f64; 4];
    assignments[2] = fixed_x;
    assignments[3] = fixed_y;

    let mut buf = render_residual_field(&viewport, |x, y| {
        assignments[0] = x;
        assignments[1] = y;
        let mut r0 = 0.0_f64;
        let mut r1 = 0.0_f64;
        let mut r2 = 0.0_f64;
        let mut degenerate = false;
        constraint.residual(
            &layout,
            &assignments,
            &mut r0,
            &mut r1,
            &mut r2,
            &mut degenerate,
        );
        (r0 * r0 + r1 * r1).sqrt()
    });
    // Green = constraint solution (PointsCoincident ⇒ must coincide with fixed point).
    draw_solver_overlay(
        &mut buf,
        &viewport,
        EXAMPLE_POINT_X,
        EXAMPLE_POINT_Y,
        fixed_x,
        fixed_y,
    );
    buf
}

/// Renders the residual field for a "distance between two points" constraint.
/// One point is fixed at `(fixed_x, fixed_y)`; the other is varied over the grid.
/// Target distance is `target_distance`. Residual = actual distance − target (one scalar);
/// we plot the fractional part of scaled magnitude to get concentric rings (zero on the circle).
/// Near-zero residual is drawn in turquoise.
pub fn render_distance_residual_to_image(
    fixed_x: f64,
    fixed_y: f64,
    target_distance: f64,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    width: u32,
    height: u32,
) -> image::RgbImage {
    let viewport = Viewport::new(x_min, x_max, y_min, y_max, width, height);
    let p0 = DatumPoint::new_xy(0, 1);
    let p1 = DatumPoint::new_xy(2, 3);
    let constraint = Constraint::Distance(p0, p1, target_distance);
    let layout = Layout::new(&[0, 1, 2, 3], &[&constraint], Config::default());
    let mut assignments = [0.0_f64; 4];
    assignments[2] = fixed_x;
    assignments[3] = fixed_y;

    let mut buf = render_residual_field(&viewport, |x, y| {
        assignments[0] = x;
        assignments[1] = y;
        let mut r0 = 0.0_f64;
        let mut r1 = 0.0_f64;
        let mut r2 = 0.0_f64;
        let mut degenerate = false;
        constraint.residual(
            &layout,
            &assignments,
            &mut r0,
            &mut r1,
            &mut r2,
            &mut degenerate,
        );
        r0.abs()
    });

    let ex_x = DISTANCE_EXAMPLE_POINT_X;
    let ex_y = DISTANCE_EXAMPLE_POINT_Y;
    let dx = ex_x - fixed_x;
    let dy = ex_y - fixed_y;
    let dist_to_ex = libm::hypot(dx, dy);
    // Green = constraint solution: the unique point on the circle (radius target_distance
    // around fixed) in the same radial direction as the example (where the solver would land).
    let (sol_x, sol_y) = if dist_to_ex > 1e-10 {
        let ux = dx / dist_to_ex;
        let uy = dy / dist_to_ex;
        (
            fixed_x + ux * target_distance,
            fixed_y + uy * target_distance,
        )
    } else {
        (fixed_x + target_distance, fixed_y)
    };
    draw_solver_overlay(&mut buf, &viewport, ex_x, ex_y, sol_x, sol_y);
    buf
}

/// Line equation Ax + By + C = 0 from two points (px, py) and (qx, qy). Returns (a, b, c).
fn line_equation(px: f64, py: f64, qx: f64, qy: f64) -> (f64, f64, f64) {
    let a = py - qy;
    let b = qx - px;
    let c = px * qy - qx * py;
    (a, b, c)
}

/// Renders the residual field for "perpendicular distance from point to line" (PointLineDistance).
/// The line is fixed; the point is varied over the grid. Residual = signed perpendicular distance − target.
/// Near-zero residual is turquoise (two lines parallel to the fixed line). Green = where the point would solve to.
pub fn render_point_line_distance_residual_to_image(
    line_p0_x: f64,
    line_p0_y: f64,
    line_p1_x: f64,
    line_p1_y: f64,
    target_distance: f64,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    width: u32,
    height: u32,
) -> image::RgbImage {
    let viewport = Viewport::new(x_min, x_max, y_min, y_max, width, height);
    let point = DatumPoint::new_xy(0, 1);
    let line = DatumLineSegment::new(DatumPoint::new_xy(2, 3), DatumPoint::new_xy(4, 5));
    let constraint = Constraint::PointLineDistance(point, line, target_distance);
    let layout = Layout::new(&[0, 1, 2, 3, 4, 5], &[&constraint], Config::default());
    let mut assignments = [0.0_f64; 6];
    assignments[2] = line_p0_x;
    assignments[3] = line_p0_y;
    assignments[4] = line_p1_x;
    assignments[5] = line_p1_y;

    let (a, b, c) = line_equation(line_p0_x, line_p0_y, line_p1_x, line_p1_y);
    let denom = libm::hypot(a, b);
    let denom = if denom > 1e-10 { denom } else { 1.0 };

    let mut buf = render_residual_field(&viewport, |x, y| {
        assignments[0] = x;
        assignments[1] = y;
        let mut r0 = 0.0_f64;
        let mut r1 = 0.0_f64;
        let mut r2 = 0.0_f64;
        let mut degenerate = false;
        constraint.residual(
            &layout,
            &assignments,
            &mut r0,
            &mut r1,
            &mut r2,
            &mut degenerate,
        );
        r0.abs()
    });

    let ex_x = PERP_DISTANCE_EXAMPLE_POINT_X;
    let ex_y = PERP_DISTANCE_EXAMPLE_POINT_Y;
    let actual = (a * ex_x + b * ex_y + c) / denom;
    // Green = constraint solution: point on the line at target_distance in the same direction from the line as the example.
    let sol_x = ex_x + a / denom * (target_distance - actual);
    let sol_y = ex_y + b / denom * (target_distance - actual);
    draw_solver_overlay(&mut buf, &viewport, ex_x, ex_y, sol_x, sol_y);
    buf
}

/// Renders the residual field for "vertical" constraint (two points same x).
/// p1 is fixed at (fixed_x, fixed_y); p0 is varied. Residual = p0.x − p1.x (zero on vertical line).
/// Green = where the point would solve to (same x as fixed, same y as example).
pub fn render_vertical_residual_to_image(
    fixed_x: f64,
    fixed_y: f64,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    width: u32,
    height: u32,
) -> image::RgbImage {
    let viewport = Viewport::new(x_min, x_max, y_min, y_max, width, height);
    let line = DatumLineSegment::new(DatumPoint::new_xy(0, 1), DatumPoint::new_xy(2, 3));
    let constraint = Constraint::Vertical(line);
    let layout = Layout::new(&[0, 1, 2, 3], &[&constraint], Config::default());
    let mut assignments = [0.0_f64; 4];
    assignments[2] = fixed_x;
    assignments[3] = fixed_y;

    let mut buf = render_residual_field(&viewport, |x, y| {
        assignments[0] = x;
        assignments[1] = y;
        let mut r0 = 0.0_f64;
        let mut r1 = 0.0_f64;
        let mut r2 = 0.0_f64;
        let mut degenerate = false;
        constraint.residual(
            &layout,
            &assignments,
            &mut r0,
            &mut r1,
            &mut r2,
            &mut degenerate,
        );
        r0.abs()
    });

    let ex_x = VERTICAL_HORIZONTAL_EXAMPLE_POINT_X;
    let ex_y = VERTICAL_HORIZONTAL_EXAMPLE_POINT_Y;
    let sol_x = fixed_x;
    let sol_y = ex_y;
    draw_solver_overlay(&mut buf, &viewport, ex_x, ex_y, sol_x, sol_y);
    buf
}

/// Renders the residual field for "horizontal" constraint (two points same y).
/// p1 is fixed at (fixed_x, fixed_y); p0 is varied. Residual = p0.y − p1.y (zero on horizontal line).
/// Green = where the point would solve to (same x as example, same y as fixed).
pub fn render_horizontal_residual_to_image(
    fixed_x: f64,
    fixed_y: f64,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    width: u32,
    height: u32,
) -> image::RgbImage {
    let viewport = Viewport::new(x_min, x_max, y_min, y_max, width, height);
    let line = DatumLineSegment::new(DatumPoint::new_xy(0, 1), DatumPoint::new_xy(2, 3));
    let constraint = Constraint::Horizontal(line);
    let layout = Layout::new(&[0, 1, 2, 3], &[&constraint], Config::default());
    let mut assignments = [0.0_f64; 4];
    assignments[2] = fixed_x;
    assignments[3] = fixed_y;

    let mut buf = render_residual_field(&viewport, |x, y| {
        assignments[0] = x;
        assignments[1] = y;
        let mut r0 = 0.0_f64;
        let mut r1 = 0.0_f64;
        let mut r2 = 0.0_f64;
        let mut degenerate = false;
        constraint.residual(
            &layout,
            &assignments,
            &mut r0,
            &mut r1,
            &mut r2,
            &mut degenerate,
        );
        r0.abs()
    });

    let ex_x = VERTICAL_HORIZONTAL_EXAMPLE_POINT_X;
    let ex_y = VERTICAL_HORIZONTAL_EXAMPLE_POINT_Y;
    let sol_x = ex_x;
    let sol_y = fixed_y;
    draw_solver_overlay(&mut buf, &viewport, ex_x, ex_y, sol_x, sol_y);
    buf
}

/// Renders the residual field for a "point coincident with fixed point" constraint.
/// One point is fixed at `(fixed_x, fixed_y)`; the other is varied over the grid.
/// Residual is (dx, dy); we plot magnitude so you get concentric rings (distance field).
///
/// Returns an error if the image could not be written.
pub fn render_points_coincident_residual(
    path: &Path,
    fixed_x: f64,
    fixed_y: f64,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    width: u32,
    height: u32,
) -> Result<(), io::Error> {
    let buf = render_points_coincident_residual_to_image(
        fixed_x, fixed_y, x_min, x_max, y_min, y_max, width, height,
    );
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
    }
    buf.save(path)
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e))
}

#[cfg(all(test, feature = "residual-viz"))]
mod tests {
    use super::*;
    use std::path::PathBuf;

    /// Baseline path for visual regression (committed in repo). Update with
    /// `TWENTY_TWENTY=overwrite cargo test -p kcl-ezpz --features residual-viz residual_viz`.
    const POINTS_COINCIDENT_BASELINE: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/residual_viz_baselines/points_coincident.png"
    );
    const DISTANCE_BASELINE: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/residual_viz_baselines/distance.png"
    );
    const POINT_LINE_DISTANCE_BASELINE: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/residual_viz_baselines/point_line_distance.png"
    );
    const VERTICAL_BASELINE: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/residual_viz_baselines/vertical.png"
    );
    const HORIZONTAL_BASELINE: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/residual_viz_baselines/horizontal.png"
    );

    #[test]
    fn points_coincident_residual_matches_baseline() {
        let actual =
            render_points_coincident_residual_to_image(0.0, 0.0, -5.0, 5.0, -5.0, 5.0, 256, 256);
        let dynamic = image::DynamicImage::ImageRgb8(actual);
        twenty_twenty::assert_image(POINTS_COINCIDENT_BASELINE, &dynamic, 0.99);
    }

    #[test]
    fn points_coincident_residual_renders_to_file() {
        let out_dir: PathBuf = std::env::var("CARGO_TARGET_DIR")
            .unwrap_or_else(|_| "target".into())
            .into();
        let path = out_dir.join("residual_viz_points_coincident.png");
        let result =
            render_points_coincident_residual(&path, 0.0, 0.0, -5.0, 5.0, -5.0, 5.0, 256, 256);
        result.expect("residual viz should write PNG");
    }

    #[test]
    fn distance_residual_matches_baseline() {
        let actual =
            render_distance_residual_to_image(0.0, 0.0, 3.0, -5.0, 5.0, -5.0, 5.0, 256, 256);
        let dynamic = image::DynamicImage::ImageRgb8(actual);
        twenty_twenty::assert_image(DISTANCE_BASELINE, &dynamic, 0.99);
    }

    #[test]
    fn point_line_distance_residual_matches_baseline() {
        // Line from (-4, -2) to (4, 2) — angled so it’s clearly distinct from horizontal/vertical.
        let actual = render_point_line_distance_residual_to_image(
            -4.0, -2.0, 4.0, 2.0, 2.0, -5.0, 5.0, -5.0, 5.0, 256, 256,
        );
        let dynamic = image::DynamicImage::ImageRgb8(actual);
        twenty_twenty::assert_image(POINT_LINE_DISTANCE_BASELINE, &dynamic, 0.99);
    }

    #[test]
    fn vertical_residual_matches_baseline() {
        let actual = render_vertical_residual_to_image(0.0, 0.0, -5.0, 5.0, -5.0, 5.0, 256, 256);
        let dynamic = image::DynamicImage::ImageRgb8(actual);
        twenty_twenty::assert_image(VERTICAL_BASELINE, &dynamic, 0.99);
    }

    #[test]
    fn horizontal_residual_matches_baseline() {
        let actual = render_horizontal_residual_to_image(0.0, 0.0, -5.0, 5.0, -5.0, 5.0, 256, 256);
        let dynamic = image::DynamicImage::ImageRgb8(actual);
        twenty_twenty::assert_image(HORIZONTAL_BASELINE, &dynamic, 0.99);
    }
}
