use std::str::FromStr;

use ezpz::{
    CircleSide, Config, Constraint, ConstraintRequest, FailureOutcome, FreedomAnalysis, Id,
    LineSide, NonLinearSystemError, SolveOutcome, SolveOutcomeFreedomAnalysis, Warning,
    WarningContent,
    datatypes::{Angle, AngleKind, inputs::*, outputs::*},
    textual,
};
use serde::{Deserialize, Serialize, de::DeserializeOwned};
use wasm_bindgen::prelude::*;

type JsResult<T> = Result<T, JsValue>;

#[derive(Clone, Copy, Default, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
struct WasmConfig {
    max_iterations: Option<usize>,
    convergence_tolerance: Option<f64>,
    step_tolerance: Option<f64>,
}

#[derive(Clone, Copy, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
enum WasmAngleUnit {
    Degrees,
    Radians,
}

#[derive(Clone, Copy, Serialize, Deserialize)]
struct WasmAngle {
    value: f64,
    unit: WasmAngleUnit,
}

#[derive(Clone, Copy, Serialize, Deserialize)]
struct WasmDatumDistance {
    id: Id,
}

#[derive(Clone, Copy, Serialize, Deserialize)]
struct WasmDatumPoint {
    x_id: Id,
    y_id: Id,
}

#[derive(Clone, Copy, Serialize, Deserialize)]
struct WasmDatumLineSegment {
    p0: WasmDatumPoint,
    p1: WasmDatumPoint,
}

#[derive(Clone, Copy, Serialize, Deserialize)]
struct WasmDatumCircle {
    center: WasmDatumPoint,
    radius: WasmDatumDistance,
}

#[derive(Clone, Copy, Serialize, Deserialize)]
struct WasmDatumCircularArc {
    center: WasmDatumPoint,
    start: WasmDatumPoint,
    end: WasmDatumPoint,
}

#[derive(Clone, Copy, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
enum WasmAngleKind {
    Parallel,
    Perpendicular,
    Other(WasmAngle),
}

#[derive(Clone, Copy, Serialize, Deserialize)]
enum WasmLineSide {
    Undefined,
    Left,
    Right,
}

#[derive(Clone, Copy, Serialize, Deserialize)]
enum WasmCircleSide {
    Undefined,
    Exterior,
    Interior,
}

#[derive(Clone, Serialize, Deserialize)]
#[serde(tag = "kind", content = "data")]
enum WasmConstraint {
    LineTangentToCircle(WasmDatumLineSegment, WasmDatumCircle, WasmLineSide),
    CircleTangentToCircle(WasmDatumCircle, WasmDatumCircle, WasmCircleSide),
    Distance(WasmDatumPoint, WasmDatumPoint, f64),
    DistanceVar(WasmDatumPoint, WasmDatumPoint, WasmDatumDistance),
    VerticalDistance(WasmDatumPoint, WasmDatumPoint, f64),
    HorizontalDistance(WasmDatumPoint, WasmDatumPoint, f64),
    Vertical(WasmDatumLineSegment),
    Horizontal(WasmDatumLineSegment),
    LinesAtAngle(WasmDatumLineSegment, WasmDatumLineSegment, WasmAngleKind),
    Fixed(Id, f64),
    ScalarEqual(Id, Id),
    PointsCoincident(WasmDatumPoint, WasmDatumPoint),
    CircleRadius(WasmDatumCircle, f64),
    LinesEqualLength(WasmDatumLineSegment, WasmDatumLineSegment),
    ArcRadius(WasmDatumCircularArc, f64),
    Arc(WasmDatumCircularArc),
    Midpoint(WasmDatumLineSegment, WasmDatumPoint),
    PointLineDistance(WasmDatumPoint, WasmDatumLineSegment, f64),
    VerticalPointLineDistance(WasmDatumPoint, WasmDatumLineSegment, f64),
    HorizontalPointLineDistance(WasmDatumPoint, WasmDatumLineSegment, f64),
    Symmetric(WasmDatumLineSegment, WasmDatumPoint, WasmDatumPoint),
    PointArcCoincident(WasmDatumCircularArc, WasmDatumPoint),
    ArcLength(WasmDatumCircularArc, f64),
    ArcAngle(WasmDatumCircularArc, WasmAngle),
    PointsAtAngle(
        WasmDatumPoint,
        WasmDatumPoint,
        WasmDatumPoint,
        WasmAngleKind,
    ),
}

#[derive(Clone, Serialize, Deserialize)]
struct WasmConstraintRequest {
    constraint: WasmConstraint,
    priority: u32,
    weight: f64,
}

#[derive(Clone, Copy, Serialize, Deserialize)]
struct WasmPoint {
    x: f64,
    y: f64,
}

#[derive(Clone, Copy, Serialize, Deserialize)]
struct WasmCircle {
    radius: f64,
    center: WasmPoint,
}

#[derive(Clone, Copy, Serialize, Deserialize)]
struct WasmArc {
    a: WasmPoint,
    b: WasmPoint,
    center: WasmPoint,
}

#[derive(Clone, Serialize, Deserialize)]
#[serde(tag = "kind", content = "data")]
enum WasmWarningContent {
    Degenerate,
    ShouldBeParallel(WasmAngle),
    ShouldBePerpendicular(WasmAngle),
}

#[derive(Clone, Serialize, Deserialize)]
struct WasmWarning {
    about_constraint: Option<usize>,
    content: WasmWarningContent,
    message: String,
}

#[derive(Clone, Serialize, Deserialize)]
struct WasmSolveOutcome {
    unsatisfied: Vec<usize>,
    final_values: Vec<f64>,
    iterations: usize,
    warnings: Vec<WasmWarning>,
    priority_solved: u32,
    is_satisfied: bool,
    is_unsatisfied: bool,
}

#[derive(Clone, Serialize, Deserialize)]
struct WasmFreedomAnalysis {
    underconstrained: Vec<Id>,
    is_underconstrained: bool,
}

#[derive(Clone, Serialize, Deserialize)]
struct WasmSolveOutcomeFreedomAnalysis {
    analysis: WasmFreedomAnalysis,
    outcome: WasmSolveOutcome,
}

#[derive(Clone, Serialize, Deserialize)]
#[serde(tag = "kind", content = "data")]
enum WasmNonLinearSystemError {
    NotFound(Id),
    WrongNumberGuesses { labels: usize, guesses: usize },
    MissingGuess { constraint_id: usize, variable: Id },
    FaerMatrix { message: String },
    Faer { message: String },
    FaerSolve { message: String },
    FaerSvd { message: String },
    DidNotConverge,
    EmptySystemNotAllowed,
}

#[derive(Clone, Serialize, Deserialize)]
struct WasmFailureOutcome {
    error: WasmNonLinearSystemError,
    message: String,
    warnings: Vec<WasmWarning>,
    num_vars: usize,
    num_eqs: usize,
}

#[derive(Clone, Serialize, Deserialize)]
#[serde(tag = "kind", content = "data")]
enum WasmTextualError {
    MissingGuess { label: String },
    UnusedGuesses { labels: Vec<String> },
    UndefinedPoint { label: String },
    Parse { message: String },
}

#[derive(Clone, Serialize, Deserialize)]
struct WasmNamedPoint {
    label: String,
    point: WasmPoint,
}

#[derive(Clone, Serialize, Deserialize)]
struct WasmNamedCircle {
    label: String,
    circle: WasmCircle,
}

#[derive(Clone, Serialize, Deserialize)]
struct WasmNamedArc {
    label: String,
    arc: WasmArc,
}

#[derive(Clone, Serialize, Deserialize)]
struct WasmNamedLine {
    p0: String,
    p1: String,
}

#[derive(Clone, Serialize, Deserialize)]
struct WasmTextualOutcome {
    unsatisfied: Vec<usize>,
    iterations: usize,
    warnings: Vec<WasmWarning>,
    points: Vec<WasmNamedPoint>,
    circles: Vec<WasmNamedCircle>,
    arcs: Vec<WasmNamedArc>,
    lines: Vec<WasmNamedLine>,
    num_vars: usize,
    num_eqs: usize,
    priority_solved: u32,
}

#[derive(Clone, Serialize, Deserialize)]
struct WasmTextualOutcomeAnalysis {
    analysis: WasmFreedomAnalysis,
    outcome: WasmTextualOutcome,
}

fn serialize<T: Serialize>(value: &T) -> JsResult<JsValue> {
    serde_wasm_bindgen::to_value(value).map_err(js_value_from_display)
}

fn deserialize<T: DeserializeOwned>(value: JsValue) -> JsResult<T> {
    serde_wasm_bindgen::from_value(value).map_err(js_value_from_display)
}

fn js_value_from_display(error: impl std::fmt::Display) -> JsValue {
    JsValue::from_str(&error.to_string())
}

impl From<WasmConfig> for Config {
    fn from(value: WasmConfig) -> Self {
        let mut out = Config::default();
        if let Some(max_iterations) = value.max_iterations {
            out = out.with_max_iterations(max_iterations);
        }
        if let Some(convergence_tolerance) = value.convergence_tolerance {
            out = out.with_convergence_tolerance(convergence_tolerance);
        }
        if let Some(step_tolerance) = value.step_tolerance {
            out = out.with_step_tolerance(step_tolerance);
        }
        out
    }
}

impl From<WasmAngle> for Angle {
    fn from(value: WasmAngle) -> Self {
        match value.unit {
            WasmAngleUnit::Degrees => Self::from_degrees(value.value),
            WasmAngleUnit::Radians => Self::from_radians(value.value),
        }
    }
}

impl From<Angle> for WasmAngle {
    fn from(value: Angle) -> Self {
        Self {
            value: value.to_radians(),
            unit: WasmAngleUnit::Radians,
        }
    }
}

impl From<WasmAngleKind> for AngleKind {
    fn from(value: WasmAngleKind) -> Self {
        match value {
            WasmAngleKind::Parallel => Self::Parallel,
            WasmAngleKind::Perpendicular => Self::Perpendicular,
            WasmAngleKind::Other(angle) => Self::Other(angle.into()),
        }
    }
}

impl From<WarningContent> for WasmWarningContent {
    fn from(value: WarningContent) -> Self {
        match value {
            WarningContent::Degenerate => Self::Degenerate,
            WarningContent::ShouldBeParallel(angle) => Self::ShouldBeParallel(angle.into()),
            WarningContent::ShouldBePerpendicular(angle) => {
                Self::ShouldBePerpendicular(angle.into())
            }
            _ => unreachable!("unsupported future WarningContent variant"),
        }
    }
}

impl From<Warning> for WasmWarning {
    fn from(value: Warning) -> Self {
        Self {
            about_constraint: value.about_constraint,
            content: value.content.into(),
            message: value.content.to_string(),
        }
    }
}

impl From<WasmDatumDistance> for DatumDistance {
    fn from(value: WasmDatumDistance) -> Self {
        Self::new(value.id)
    }
}

impl From<WasmDatumPoint> for DatumPoint {
    fn from(value: WasmDatumPoint) -> Self {
        Self::new_xy(value.x_id, value.y_id)
    }
}

impl From<WasmDatumLineSegment> for DatumLineSegment {
    fn from(value: WasmDatumLineSegment) -> Self {
        Self::new(value.p0.into(), value.p1.into())
    }
}

impl From<WasmDatumCircle> for DatumCircle {
    fn from(value: WasmDatumCircle) -> Self {
        Self {
            center: value.center.into(),
            radius: value.radius.into(),
        }
    }
}

impl From<WasmDatumCircularArc> for DatumCircularArc {
    fn from(value: WasmDatumCircularArc) -> Self {
        Self {
            center: value.center.into(),
            start: value.start.into(),
            end: value.end.into(),
        }
    }
}

impl From<WasmLineSide> for LineSide {
    fn from(value: WasmLineSide) -> Self {
        match value {
            WasmLineSide::Undefined => Self::Undefined,
            WasmLineSide::Left => Self::Left,
            WasmLineSide::Right => Self::Right,
        }
    }
}

impl From<WasmCircleSide> for CircleSide {
    fn from(value: WasmCircleSide) -> Self {
        match value {
            WasmCircleSide::Undefined => Self::Undefined,
            WasmCircleSide::Exterior => Self::Exterior,
            WasmCircleSide::Interior => Self::Interior,
        }
    }
}

impl From<WasmConstraint> for Constraint {
    fn from(value: WasmConstraint) -> Self {
        match value {
            WasmConstraint::LineTangentToCircle(line, circle, side) => {
                Self::LineTangentToCircle(line.into(), circle.into(), side.into())
            }
            WasmConstraint::CircleTangentToCircle(circle0, circle1, side) => {
                Self::CircleTangentToCircle(circle0.into(), circle1.into(), side.into())
            }
            WasmConstraint::Distance(p0, p1, distance) => {
                Self::Distance(p0.into(), p1.into(), distance)
            }
            WasmConstraint::DistanceVar(p0, p1, distance) => {
                Self::DistanceVar(p0.into(), p1.into(), distance.into())
            }
            WasmConstraint::VerticalDistance(p0, p1, distance) => {
                Self::VerticalDistance(p0.into(), p1.into(), distance)
            }
            WasmConstraint::HorizontalDistance(p0, p1, distance) => {
                Self::HorizontalDistance(p0.into(), p1.into(), distance)
            }
            WasmConstraint::Vertical(line) => Self::Vertical(line.into()),
            WasmConstraint::Horizontal(line) => Self::Horizontal(line.into()),
            WasmConstraint::LinesAtAngle(line0, line1, angle) => {
                Self::LinesAtAngle(line0.into(), line1.into(), angle.into())
            }
            WasmConstraint::Fixed(id, value) => Self::Fixed(id, value),
            WasmConstraint::ScalarEqual(left, right) => Self::ScalarEqual(left, right),
            WasmConstraint::PointsCoincident(p0, p1) => {
                Self::PointsCoincident(p0.into(), p1.into())
            }
            WasmConstraint::CircleRadius(circle, radius) => {
                Self::CircleRadius(circle.into(), radius)
            }
            WasmConstraint::LinesEqualLength(line0, line1) => {
                Self::LinesEqualLength(line0.into(), line1.into())
            }
            WasmConstraint::ArcRadius(arc, radius) => Self::ArcRadius(arc.into(), radius),
            WasmConstraint::Arc(arc) => Self::Arc(arc.into()),
            WasmConstraint::Midpoint(line, point) => Self::Midpoint(line.into(), point.into()),
            WasmConstraint::PointLineDistance(point, line, distance) => {
                Self::PointLineDistance(point.into(), line.into(), distance)
            }
            WasmConstraint::VerticalPointLineDistance(point, line, distance) => {
                Self::VerticalPointLineDistance(point.into(), line.into(), distance)
            }
            WasmConstraint::HorizontalPointLineDistance(point, line, distance) => {
                Self::HorizontalPointLineDistance(point.into(), line.into(), distance)
            }
            WasmConstraint::Symmetric(line, p0, p1) => {
                Self::Symmetric(line.into(), p0.into(), p1.into())
            }
            WasmConstraint::PointArcCoincident(arc, point) => {
                Self::PointArcCoincident(arc.into(), point.into())
            }
            WasmConstraint::ArcLength(arc, length) => Self::ArcLength(arc.into(), length),
            WasmConstraint::ArcAngle(arc, angle) => Self::ArcAngle(arc.into(), angle.into()),
            WasmConstraint::PointsAtAngle(p0, p1, p2, angle) => {
                Self::PointsAtAngle(p0.into(), p1.into(), p2.into(), angle.into())
            }
        }
    }
}

impl From<WasmConstraintRequest> for ConstraintRequest {
    fn from(value: WasmConstraintRequest) -> Self {
        ConstraintRequest::new(value.constraint.into(), value.priority).with_weight(value.weight)
    }
}

impl From<Point> for WasmPoint {
    fn from(value: Point) -> Self {
        Self {
            x: value.x,
            y: value.y,
        }
    }
}

impl From<Circle> for WasmCircle {
    fn from(value: Circle) -> Self {
        Self {
            radius: value.radius,
            center: value.center.into(),
        }
    }
}

impl From<Arc> for WasmArc {
    fn from(value: Arc) -> Self {
        Self {
            a: value.a.into(),
            b: value.b.into(),
            center: value.center.into(),
        }
    }
}

impl From<&FreedomAnalysis> for WasmFreedomAnalysis {
    fn from(value: &FreedomAnalysis) -> Self {
        Self {
            underconstrained: value.underconstrained().to_vec(),
            is_underconstrained: value.is_underconstrained(),
        }
    }
}

impl From<&SolveOutcome> for WasmSolveOutcome {
    fn from(value: &SolveOutcome) -> Self {
        Self {
            unsatisfied: value.unsatisfied().to_vec(),
            final_values: value.final_values().to_vec(),
            iterations: value.iterations(),
            warnings: value.warnings().iter().copied().map(Into::into).collect(),
            priority_solved: value.priority_solved(),
            is_satisfied: value.is_satisfied(),
            is_unsatisfied: value.is_unsatisfied(),
        }
    }
}

impl From<&SolveOutcomeFreedomAnalysis> for WasmSolveOutcomeFreedomAnalysis {
    fn from(value: &SolveOutcomeFreedomAnalysis) -> Self {
        Self {
            analysis: (&value.analysis).into(),
            outcome: (&value.outcome).into(),
        }
    }
}

impl From<&NonLinearSystemError> for WasmNonLinearSystemError {
    fn from(value: &NonLinearSystemError) -> Self {
        match value {
            NonLinearSystemError::NotFound(id) => Self::NotFound(*id),
            NonLinearSystemError::WrongNumberGuesses { labels, guesses } => {
                Self::WrongNumberGuesses {
                    labels: *labels,
                    guesses: *guesses,
                }
            }
            NonLinearSystemError::MissingGuess {
                constraint_id,
                variable,
            } => Self::MissingGuess {
                constraint_id: *constraint_id,
                variable: *variable,
            },
            NonLinearSystemError::FaerMatrix { error } => Self::FaerMatrix {
                message: error.to_string(),
            },
            NonLinearSystemError::Faer { error } => Self::Faer {
                message: error.to_string(),
            },
            NonLinearSystemError::FaerSolve { error } => Self::FaerSolve {
                message: error.to_string(),
            },
            NonLinearSystemError::FaerSvd(error) => Self::FaerSvd {
                message: format!("{error:?}"),
            },
            NonLinearSystemError::DidNotConverge => Self::DidNotConverge,
            NonLinearSystemError::EmptySystemNotAllowed => Self::EmptySystemNotAllowed,
            _ => Self::Faer {
                message: value.to_string(),
            },
        }
    }
}

impl From<&FailureOutcome> for WasmFailureOutcome {
    fn from(value: &FailureOutcome) -> Self {
        Self {
            error: value.error().into(),
            message: value.error().to_string(),
            warnings: value.warnings().iter().copied().map(Into::into).collect(),
            num_vars: value.num_vars(),
            num_eqs: value.num_eqs(),
        }
    }
}

impl From<ezpz::TextualError> for WasmTextualError {
    fn from(value: ezpz::TextualError) -> Self {
        match value {
            ezpz::TextualError::MissingGuess { label } => Self::MissingGuess { label },
            ezpz::TextualError::UnusedGuesses { labels } => Self::UnusedGuesses { labels },
            ezpz::TextualError::UndefinedPoint { label } => Self::UndefinedPoint { label },
            _ => Self::Parse {
                message: value.to_string(),
            },
        }
    }
}

impl From<&textual::Outcome> for WasmTextualOutcome {
    fn from(value: &textual::Outcome) -> Self {
        Self {
            unsatisfied: value.unsatisfied.clone(),
            iterations: value.iterations,
            warnings: value.warnings.iter().copied().map(Into::into).collect(),
            points: value
                .points
                .iter()
                .map(|(label, point)| WasmNamedPoint {
                    label: label.clone(),
                    point: (*point).into(),
                })
                .collect(),
            circles: value
                .circles
                .iter()
                .map(|(label, circle)| WasmNamedCircle {
                    label: label.clone(),
                    circle: (*circle).into(),
                })
                .collect(),
            arcs: value
                .arcs
                .iter()
                .map(|(label, arc)| WasmNamedArc {
                    label: label.clone(),
                    arc: (*arc).into(),
                })
                .collect(),
            lines: value
                .lines
                .iter()
                .map(|(p0, p1)| WasmNamedLine {
                    p0: String::from(p0.clone()),
                    p1: String::from(p1.clone()),
                })
                .collect(),
            num_vars: value.num_vars,
            num_eqs: value.num_eqs,
            priority_solved: value.priority_solved,
        }
    }
}

impl From<&textual::OutcomeAnalysis> for WasmTextualOutcomeAnalysis {
    fn from(value: &textual::OutcomeAnalysis) -> Self {
        Self {
            analysis: (&value.analysis).into(),
            outcome: (&value.outcome).into(),
        }
    }
}

fn failure_to_js(error: &FailureOutcome) -> JsValue {
    serialize(&WasmFailureOutcome::from(error))
        .unwrap_or_else(|_| JsValue::from_str(&error.error().to_string()))
}

fn textual_error_to_js(error: impl Into<WasmTextualError>) -> JsValue {
    serialize(&error.into()).unwrap_or_else(|_| JsValue::from_str("textual error"))
}

fn parse_config(config: Option<JsValue>) -> JsResult<Config> {
    match config {
        Some(config) if !config.is_undefined() && !config.is_null() => {
            Ok(Config::from(deserialize::<WasmConfig>(config)?))
        }
        _ => Ok(Config::default()),
    }
}

fn parse_requests(value: JsValue) -> JsResult<Vec<ConstraintRequest>> {
    let requests: Vec<WasmConstraintRequest> = deserialize(value)?;
    Ok(requests.into_iter().map(Into::into).collect())
}

fn parse_initial_guesses(value: JsValue) -> JsResult<Vec<(Id, f64)>> {
    deserialize(value)
}

#[wasm_bindgen]
pub fn solve(
    requests: JsValue,
    initial_guesses: JsValue,
    config: Option<JsValue>,
) -> JsResult<JsValue> {
    let requests = parse_requests(requests)?;
    let initial_guesses = parse_initial_guesses(initial_guesses)?;
    let config = parse_config(config)?;
    match ezpz::solve(&requests, initial_guesses, config) {
        Ok(outcome) => serialize(&WasmSolveOutcome::from(&outcome)),
        Err(error) => Err(failure_to_js(&error)),
    }
}

#[wasm_bindgen(js_name = solveAnalysis)]
pub fn solve_analysis(
    requests: JsValue,
    initial_guesses: JsValue,
    config: Option<JsValue>,
) -> JsResult<JsValue> {
    let requests = parse_requests(requests)?;
    let initial_guesses = parse_initial_guesses(initial_guesses)?;
    let config = parse_config(config)?;
    match ezpz::solve_analysis(&requests, initial_guesses, config) {
        Ok(outcome) => serialize(&WasmSolveOutcomeFreedomAnalysis::from(&outcome)),
        Err(error) => Err(failure_to_js(&error)),
    }
}

#[wasm_bindgen]
pub fn solve_text(source: &str, config: Option<JsValue>) -> JsResult<JsValue> {
    let problem = textual::Problem::from_str(source)
        .map_err(|message| textual_error_to_js(WasmTextualError::Parse { message }))?;
    let system = problem
        .to_constraint_system()
        .map_err(textual_error_to_js)?;
    let config = parse_config(config)?;
    match system.solve_with_config(config) {
        Ok(outcome) => serialize(&WasmTextualOutcome::from(&outcome)),
        Err(error) => Err(failure_to_js(&error)),
    }
}

#[wasm_bindgen(js_name = solveTextAnalysis)]
pub fn solve_text_analysis(source: &str, config: Option<JsValue>) -> JsResult<JsValue> {
    let problem = textual::Problem::from_str(source)
        .map_err(|message| textual_error_to_js(WasmTextualError::Parse { message }))?;
    let system = problem
        .to_constraint_system()
        .map_err(textual_error_to_js)?;
    let config = parse_config(config)?;
    match system.solve_with_config_analysis(config) {
        Ok(outcome) => serialize(&WasmTextualOutcomeAnalysis::from(&outcome)),
        Err(error) => Err(failure_to_js(&error)),
    }
}
