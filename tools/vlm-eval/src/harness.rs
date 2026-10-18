//! OpenAI-compatible client, mock model, and the results table.

use std::path::Path;
use std::time::{Duration, Instant};

use base64::Engine;
use serde::Serialize;

use crate::decision::{mock_decision, score_output};
use crate::pack::scenarios;
use crate::prompt::{rules_sha256, user_prompt};
use crate::scene::Scenario;

#[derive(Clone, Debug)]
pub struct RunConfig {
    pub mock: bool,
    pub base_url: Option<String>,
    pub model: String,
    pub api_key: Option<String>,
    pub timeout: Duration,
}

#[derive(Clone, Debug, Serialize)]
pub struct CaseScore {
    pub id: String,
    pub title: String,
    pub valid_json: bool,
    pub no_actuator: bool,
    pub action_ok: bool,
    pub waypoint_ok: bool,
    pub passed: bool,
    pub latency_ms: u128,
    pub notes: String,
    pub output_excerpt: String,
}

#[derive(Clone, Debug, Serialize)]
pub struct EvalReport {
    pub mode: String,
    pub model: String,
    pub base_url: Option<String>,
    pub rules_sha256: String,
    pub camera_source: String,
    pub passed: usize,
    pub total: usize,
    pub cases: Vec<CaseScore>,
}

pub fn run_eval(config: &RunConfig) -> Result<EvalReport, String> {
    if !config.mock && config.base_url.is_none() {
        return Err("pass --base-url or --mock".into());
    }
    let mode = if config.mock { "mock" } else { "openai" };
    let mut cases = Vec::new();
    for scenario in scenarios() {
        let started = Instant::now();
        let output = if config.mock {
            mock_decision(scenario)
        } else {
            query_model(config, scenario)?
        };
        let latency_ms = started.elapsed().as_millis();
        let score = score_output(&output, scenario);
        cases.push(CaseScore {
            id: scenario.id.clone(),
            title: scenario.title.clone(),
            valid_json: score.valid_json,
            no_actuator: score.no_actuator,
            action_ok: score.action_ok,
            waypoint_ok: score.waypoint_ok,
            passed: score.passed(),
            latency_ms,
            notes: score.notes.join("; "),
            output_excerpt: excerpt(&output, 240),
        });
    }
    let passed = cases.iter().filter(|case| case.passed).count();
    Ok(EvalReport {
        mode: mode.to_string(),
        model: config.model.clone(),
        base_url: config.base_url.clone(),
        rules_sha256: rules_sha256(),
        camera_source: crate::CAMERA_SOURCE_PLACEHOLDER.to_string(),
        passed,
        total: cases.len(),
        cases,
    })
}

pub fn write_report(report: &EvalReport, path: &Path) -> Result<(), String> {
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent).map_err(|err| err.to_string())?;
        }
    }
    std::fs::write(path, render_markdown(report)).map_err(|err| err.to_string())?;
    let json_path = path.with_extension("json");
    std::fs::write(
        json_path,
        serde_json::to_vec_pretty(report).map_err(|err| err.to_string())?,
    )
    .map_err(|err| err.to_string())?;
    Ok(())
}

pub fn render_markdown(report: &EvalReport) -> String {
    let mut out = String::new();
    out.push_str("# VLM eval results\n\n");
    out.push_str(&format!("Mode: {}\n\n", report.mode));
    out.push_str(&format!("Model: {}\n\n", report.model));
    if report.mode == "mock" {
        out.push_str("This table is the deterministic mock model. It is not a Qwen, MLX, llama.cpp, or Apple Foundation Models score.\n\n");
    } else {
        out.push_str("These rows are the endpoint's answers, scored locally. This file does not invent a score the server did not return.\n\n");
    }
    out.push_str(&format!("Rules sha256: {}\n\n", report.rules_sha256));
    out.push_str(&format!("Camera frames: {}\n\n", report.camera_source));
    out.push_str(&format!("Passed: {}/{}\n\n", report.passed, report.total));
    out.push_str("| id | valid_json | no_actuator | action | waypoint | pass | ms | notes |\n");
    out.push_str("| --- | --- | --- | --- | --- | --- | --- | --- |\n");
    for case in &report.cases {
        out.push_str(&format!(
            "| {} | {} | {} | {} | {} | {} | {} | {} |\n",
            case.id,
            bit(case.valid_json),
            bit(case.no_actuator),
            bit(case.action_ok),
            bit(case.waypoint_ok),
            bit(case.passed),
            case.latency_ms,
            case.notes.replace('|', "/")
        ));
    }
    out
}

fn bit(value: bool) -> &'static str {
    if value { "pass" } else { "fail" }
}

fn excerpt(text: &str, max: usize) -> String {
    let flat: String = text.split_whitespace().collect::<Vec<_>>().join(" ");
    if flat.chars().count() <= max {
        flat
    } else {
        format!("{}…", flat.chars().take(max).collect::<String>())
    }
}

fn query_model(config: &RunConfig, scenario: &Scenario) -> Result<String, String> {
    let base = config.base_url.as_deref().ok_or("missing base URL")?;
    if base.starts_with("https://") {
        return Err(
            "this build speaks HTTP only; point --base-url at an http:// server such as mlx_vlm.server, llama-server, or LM Studio".into(),
        );
    }
    let url = completions_url(base);
    let prompt = user_prompt(scenario);
    let map_url = data_url(&scenario.map_png);
    let camera_url = data_url(&scenario.camera_png);
    let payload = serde_json::json!({
        "model": config.model,
        "temperature": 0,
        "max_tokens": 700,
        "messages": [{
            "role": "user",
            "content": [
                {"type": "text", "text": prompt},
                {"type": "image_url", "image_url": {"url": map_url}},
                {"type": "image_url", "image_url": {"url": camera_url}}
            ]
        }]
    });
    let agent = ureq::AgentBuilder::new().timeout(config.timeout).build();
    let mut request = agent.post(&url);
    if let Some(key) = &config.api_key {
        request = request.set("Authorization", &format!("Bearer {key}"));
    }
    let response = request.send_json(payload).map_err(|err| match err {
        ureq::Error::Status(code, response) => {
            let body = response.into_string().unwrap_or_default();
            format!("HTTP {code} from {url}: {}", excerpt(&body, 500))
        }
        other => format!("request to {url} failed: {other}"),
    })?;
    let body = response.into_string().map_err(|err| err.to_string())?;
    let value: serde_json::Value = serde_json::from_str(&body)
        .map_err(|err| format!("response was not JSON: {err}: {}", excerpt(&body, 300)))?;
    message_text(&value).ok_or_else(|| format!("no message content in {}", excerpt(&body, 300)))
}

fn completions_url(base: &str) -> String {
    let trimmed = base.trim_end_matches('/');
    if trimmed.ends_with("/chat/completions") {
        trimmed.to_string()
    } else {
        format!("{trimmed}/chat/completions")
    }
}

fn data_url(png: &[u8]) -> String {
    format!(
        "data:image/png;base64,{}",
        base64::engine::general_purpose::STANDARD.encode(png)
    )
}

fn message_text(value: &serde_json::Value) -> Option<String> {
    let content = &value["choices"][0]["message"]["content"];
    match content {
        serde_json::Value::String(text) => Some(text.clone()),
        serde_json::Value::Array(parts) => {
            let mut out = String::new();
            for part in parts {
                if let Some(text) = part.get("text").and_then(|text| text.as_str()) {
                    out.push_str(text);
                }
            }
            if out.is_empty() { None } else { Some(out) }
        }
        _ => value["choices"][0]["text"].as_str().map(str::to_string),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mock_run_passes_without_claiming_a_model_score() {
        let report = run_eval(&RunConfig {
            mock: true,
            base_url: None,
            model: "mock".into(),
            api_key: None,
            timeout: Duration::from_secs(1),
        })
        .unwrap();
        assert_eq!(report.mode, "mock");
        assert_eq!(report.model, "mock");
        assert_eq!(report.passed, 8);
        assert_eq!(report.total, 8);
        let markdown = render_markdown(&report);
        assert!(markdown.contains("not a Qwen"));
        assert!(markdown.contains("8/8"));
    }
}
