use std::str::FromStr;

use ezpz::{Config, ConstraintRequest, FailureOutcome, NonLinearSystemError, textual};
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

#[derive(Clone, Serialize, Deserialize)]
#[serde(tag = "kind", content = "data")]
enum WasmNonLinearSystemError {
    NotFound(ezpz::Id),
    WrongNumberGuesses {
        labels: usize,
        guesses: usize,
    },
    MissingGuess {
        constraint_id: usize,
        variable: ezpz::Id,
    },
    FaerMatrix {
        message: String,
    },
    Faer {
        message: String,
    },
    FaerSolve {
        message: String,
    },
    FaerSvd {
        message: String,
    },
    DidNotConverge,
    EmptySystemNotAllowed,
}

#[derive(Clone, Serialize, Deserialize)]
struct WasmFailureOutcome {
    error: WasmNonLinearSystemError,
    message: String,
    warnings: Vec<ezpz::Warning>,
    num_vars: usize,
    num_eqs: usize,
}

#[derive(Clone, Serialize, Deserialize)]
#[serde(tag = "kind", content = "data")]
enum WasmStringError {
    Parse { message: String },
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
            warnings: value.warnings().to_vec(),
            num_vars: value.num_vars(),
            num_eqs: value.num_eqs(),
        }
    }
}

fn failure_to_js(error: &FailureOutcome) -> JsValue {
    serialize(&WasmFailureOutcome::from(error))
        .unwrap_or_else(|_| JsValue::from_str(&error.error().to_string()))
}

fn parse_string_error_to_js(message: String) -> JsValue {
    serialize(&WasmStringError::Parse { message })
        .unwrap_or_else(|_| JsValue::from_str("parse error"))
}

fn parse_config(config: Option<JsValue>) -> JsResult<Config> {
    match config {
        Some(config) if !config.is_undefined() && !config.is_null() => {
            Ok(Config::from(deserialize::<WasmConfig>(config)?))
        }
        _ => Ok(Config::default()),
    }
}

#[wasm_bindgen]
pub fn solve(
    requests: JsValue,
    initial_guesses: JsValue,
    config: Option<JsValue>,
) -> JsResult<JsValue> {
    let requests: Vec<ConstraintRequest> = deserialize(requests)?;
    let initial_guesses: Vec<(ezpz::Id, f64)> = deserialize(initial_guesses)?;
    let config = parse_config(config)?;
    match ezpz::solve(&requests, initial_guesses, config) {
        Ok(outcome) => serialize(&outcome),
        Err(error) => Err(failure_to_js(&error)),
    }
}

#[wasm_bindgen(js_name = solveAnalysis)]
pub fn solve_analysis(
    requests: JsValue,
    initial_guesses: JsValue,
    config: Option<JsValue>,
) -> JsResult<JsValue> {
    let requests: Vec<ConstraintRequest> = deserialize(requests)?;
    let initial_guesses: Vec<(ezpz::Id, f64)> = deserialize(initial_guesses)?;
    let config = parse_config(config)?;
    match ezpz::solve_analysis(&requests, initial_guesses, config) {
        Ok(outcome) => serialize(&outcome),
        Err(error) => Err(failure_to_js(&error)),
    }
}

#[wasm_bindgen(js_name = solveText)]
pub fn solve_text(source: &str, config: Option<JsValue>) -> JsResult<JsValue> {
    let problem = textual::Problem::from_str(source).map_err(parse_string_error_to_js)?;
    let system = problem.to_constraint_system().map_err(|error| {
        serialize(&error).unwrap_or_else(|_| JsValue::from_str("textual error"))
    })?;
    let config = parse_config(config)?;
    match system.solve_with_config(config) {
        Ok(outcome) => serialize(&outcome),
        Err(error) => Err(failure_to_js(&error)),
    }
}

#[wasm_bindgen(js_name = solveTextAnalysis)]
pub fn solve_text_analysis(source: &str, config: Option<JsValue>) -> JsResult<JsValue> {
    let problem = textual::Problem::from_str(source).map_err(parse_string_error_to_js)?;
    let system = problem.to_constraint_system().map_err(|error| {
        serialize(&error).unwrap_or_else(|_| JsValue::from_str("textual error"))
    })?;
    let config = parse_config(config)?;
    match system.solve_with_config_analysis(config) {
        Ok(outcome) => serialize(&outcome),
        Err(error) => Err(failure_to_js(&error)),
    }
}
