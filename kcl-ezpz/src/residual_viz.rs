//! Residual field visualization for constraints.
//!
//! Renders the residual as a 2D scalar field (e.g. over x,y) and saves as an image,
//! useful as a sanity check when changing residual math: the image should change.

use crate::constraints::Constraint;
use crate::datatypes::inputs::DatumPoint;
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

fn world_to_pixel(
    x: f64,
    y: f64,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    width: u32,
    height: u32,
) -> (i32, i32) {
    let px = (x - x_min) / (x_max - x_min) * (width as f64);
    let py = (y_max - y) / (y_max - y_min) * (height as f64);
    (px.round() as i32, py.round() as i32)
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
    let p0 = DatumPoint::new_xy(0, 1);
    let p1 = DatumPoint::new_xy(2, 3);
    let constraint = Constraint::PointsCoincident(p0, p1);
    let constraints: &[&Constraint] = &[&constraint];
    let all_variables: Vec<crate::Id> = vec![0, 1, 2, 3];
    let config = Config::default();
    let layout = Layout::new(&all_variables, constraints, config);

    let mut assignments = [0.0_f64; 4];
    assignments[2] = fixed_x;
    assignments[3] = fixed_y;

    let ring_scale = 1.0_f64;
    let mut buf = image::RgbImage::new(width, height);
    for py in 0..height {
        let y = y_min + (y_max - y_min) * (py as f64 + 0.5) / (height as f64);
        for px in 0..width {
            let x = x_min + (x_max - x_min) * (px as f64 + 0.5) / (width as f64);
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
            let mag = (r0 * r0 + r1 * r1).sqrt();
            let pixel = if mag < ZERO_RESIDUAL_THRESHOLD {
                image::Rgb(TURQUOISE)
            } else {
                let value = mag * ring_scale;
                let fractional = value - value.trunc();
                let intensity = (255.0 - fractional * 255.0).round() as u8;
                image::Rgb([intensity, intensity, intensity])
            };
            buf.put_pixel(px, py, pixel);
        }
    }
    let (ex_px, ex_py) = world_to_pixel(
        EXAMPLE_POINT_X,
        EXAMPLE_POINT_Y,
        x_min,
        x_max,
        y_min,
        y_max,
        width,
        height,
    );
    let (sol_px, sol_py) =
        world_to_pixel(fixed_x, fixed_y, x_min, x_max, y_min, y_max, width, height);
    // Green = constraint solution (PointsCoincident ⇒ must coincide with fixed point).
    draw_arrow(&mut buf, ex_px, ex_py, sol_px, sol_py, [200, 0, 0], 6, 0.5);
    draw_filled_circle(&mut buf, ex_px, ex_py, 5, [255, 0, 0]);
    draw_filled_circle(&mut buf, sol_px, sol_py, 5, [0, 180, 0]);
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
    let p0 = DatumPoint::new_xy(0, 1);
    let p1 = DatumPoint::new_xy(2, 3);
    let constraint = Constraint::Distance(p0, p1, target_distance);
    let constraints: &[&Constraint] = &[&constraint];
    let all_variables: Vec<crate::Id> = vec![0, 1, 2, 3];
    let config = Config::default();
    let layout = Layout::new(&all_variables, constraints, config);

    let mut assignments = [0.0_f64; 4];
    assignments[2] = fixed_x;
    assignments[3] = fixed_y;

    let ring_scale = 1.0_f64;
    let mut buf = image::RgbImage::new(width, height);
    for py in 0..height {
        let y = y_min + (y_max - y_min) * (py as f64 + 0.5) / (height as f64);
        for px in 0..width {
            let x = x_min + (x_max - x_min) * (px as f64 + 0.5) / (width as f64);
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
            let mag = r0.abs();
            let pixel = if mag < ZERO_RESIDUAL_THRESHOLD {
                image::Rgb(TURQUOISE)
            } else {
                let value = mag * ring_scale;
                let fractional = value - value.trunc();
                let intensity = (255.0 - fractional * 255.0).round() as u8;
                image::Rgb([intensity, intensity, intensity])
            };
            buf.put_pixel(px, py, pixel);
        }
    }
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
    let (ex_px, ex_py) = world_to_pixel(ex_x, ex_y, x_min, x_max, y_min, y_max, width, height);
    let (sol_px, sol_py) = world_to_pixel(sol_x, sol_y, x_min, x_max, y_min, y_max, width, height);
    draw_arrow(&mut buf, ex_px, ex_py, sol_px, sol_py, [200, 0, 0], 6, 0.5);
    draw_filled_circle(&mut buf, ex_px, ex_py, 5, [255, 0, 0]);
    draw_filled_circle(&mut buf, sol_px, sol_py, 5, [0, 180, 0]);
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
}
