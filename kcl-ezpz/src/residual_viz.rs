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
