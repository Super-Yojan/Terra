//! Offline metrics computed from recorded experiment events.

use serde::{Deserialize, Serialize};
use serde_json::Value;
/// Clearance episode detector with 0.3 m entry and 0.4 m exit hysteresis.
#[derive(Default)]
pub struct NearMiss {
    active: bool,
}
impl NearMiss {
    /// Emit a near-miss start below 0.3 m or end above 0.4 m, once per transition.
    ///
    /// Missing, negative, and nonfinite clearances return `None` and preserve the episode state.
    pub fn update(&mut self, clearance: Option<f64>) -> Option<&'static str> {
        let value = clearance.filter(|v| v.is_finite() && *v >= 0.)?;
        if !self.active && value < 0.3 {
            self.active = true;
            Some("near_miss_start")
        } else if self.active && value > 0.4 {
            self.active = false;
            Some("near_miss_end")
        } else {
            None
        }
    }
}
/// Offline run outcomes; durations use seconds and rates use interventions per minute.
#[derive(Debug, Default, Serialize, Deserialize)]
pub struct RunSummary {
    /// Whether the recording or mission completed successfully, according to this record.
    pub complete: bool,
    /// Maximum recorded control-tick time, in seconds from run start.
    pub duration: f64,
    /// Accepted authority changes whose reason is `operator_selection`.
    pub mode_changes: u64,
    /// Number of operator takeover events.
    pub takeovers: u64,
    /// Takeovers, approvals, rejections, and redirects divided by run minutes.
    pub interventions_per_minute: f64,
    /// Earliest recorded survivor observation time, in seconds from run start.
    pub time_to_first_discovery: Option<f64>,
    /// First control-tick time reporting mission completion, in seconds.
    pub mission_completion_time: Option<f64>,
    /// Number of accepted supervised proposal approvals.
    pub approvals: u64,
    /// Number of accepted supervised proposal rejections.
    pub rejections: u64,
    /// Number of accepted goal events.
    pub redirects: u64,
    /// Number of near-miss episode starts.
    pub near_misses: u64,
    /// Number of contact episode starts.
    pub contacts: u64,
    /// Per-rover operator exposure in seconds; each adjacent interval is capped at 0.5 s.
    pub operator_control_seconds: f64,
    /// Whether any recorded control tick reported mission completion.
    pub mission_complete: bool,
    /// Distinct completed search sessions, keyed by rover, run, and search.
    #[serde(default)]
    pub search_confirmations: u64,
    /// Distinct exhausted search sessions, separate from confirmations.
    #[serde(default)]
    pub search_exhaustions: u64,
    /// Number of recorded route-failure events.
    #[serde(default)]
    pub search_route_failures: u64,
    /// First completed-search tick time, in seconds from run start.
    #[serde(default)]
    pub time_to_target_confirmation: Option<f64>,
}
/// Summarize JSON records in input order, tolerating missing fields with defaults.
///
/// Times are seconds from run start. Search terminal outcomes are deduplicated by rover/run/search;
/// recording completeness comes from `run_end`, independently of mission completion.
pub fn summarize(records: &[Value]) -> RunSummary {
    let mut out = RunSummary::default();
    let mut previous = std::collections::BTreeMap::<u64, (f64, String)>::new();
    let mut terminal_searches = std::collections::BTreeSet::new();
    for r in records {
        if r["kind"] == "run_end" {
            out.complete = r["complete"] == true;
        }
        if r["kind"] == "control_tick" {
            let time = r["time"].as_f64().unwrap_or(0.);
            out.duration = out.duration.max(time);
            let id = r["rover_id"].as_u64().unwrap_or(0);
            let source = r["status"]["active_source"].as_str().unwrap_or("none");
            if let Some((old, src)) = previous.get(&id)
                && (src == "operator" || src == "assisted_operator")
            {
                out.operator_control_seconds += (time - old).clamp(0., 0.5);
            }
            previous.insert(id, (time, source.into()));
            if let (Some(id), Some(phase)) = (
                r["search"]["search_id"].as_str(),
                r["search"]["phase"].as_str(),
            ) {
                if matches!(phase, "completed" | "exhausted" | "timed_out" | "cancelled")
                    && terminal_searches.insert((
                        r["rover_id"].as_u64().unwrap_or(0),
                        r["run_id"].as_str().unwrap_or("").to_string(),
                        id.to_string(),
                    ))
                {
                    if phase == "completed" {
                        out.search_confirmations += 1;
                        out.time_to_target_confirmation.get_or_insert(time);
                    } else if phase == "exhausted" {
                        out.search_exhaustions += 1;
                    }
                }
            }
            if r["mission"]["complete"] == true {
                out.mission_complete = true;
                out.mission_completion_time.get_or_insert(time);
            }
            if let Some(observations) = r["mission"]["observations"].as_array() {
                for observation in observations {
                    if let Some(t) = observation["observed_at"].as_f64() {
                        out.time_to_first_discovery =
                            Some(out.time_to_first_discovery.map_or(t, |old| old.min(t)));
                    }
                }
            }
            if let Some(events) = r["events"].as_array() {
                for e in events {
                    match e["kind"].as_str() {
                        Some("request_accepted") => match e["reason"].as_str() {
                            Some("operator_selection") => out.mode_changes += 1,
                            Some("proposal_approve") => out.approvals += 1,
                            Some("proposal_reject") => out.rejections += 1,
                            _ => {}
                        },
                        Some("route_failed") => out.search_route_failures += 1,
                        Some("takeover") => out.takeovers += 1,
                        Some("goal_accepted") => out.redirects += 1,
                        Some("near_miss_start") => out.near_misses += 1,
                        Some("contact_start") => out.contacts += 1,
                        _ => {}
                    }
                }
            }
        }
    }
    if out.duration > 0. {
        out.interventions_per_minute =
            (out.takeovers + out.approvals + out.rejections + out.redirects) as f64 * 60.
                / out.duration;
    }
    out
}
/// Area of observed free cells. This measures map observation, not confirmed survivor search.
#[derive(Default)]
pub struct ObservedArea {
    cells: std::collections::BTreeSet<(i64, i64)>,
    resolution: Option<f64>,
}
impl ObservedArea {
    /// Union observed free grid cells across compatible snapshots and return accumulated area in square metres.
    ///
    /// Returns `None` for nonpositive/nonfinite resolution, invalid dimensions/storage, or a resolution change, preserving state.
    pub fn observe(&mut self, map: &terra_mapping::MapSnapshot) -> Option<f64> {
        if !map.resolution.is_finite()
            || map.resolution <= 0.
            || map.occupancy.len() != map.width as usize * map.height as usize
            || map.width == 0
            || self.resolution.is_some_and(|r| r != map.resolution)
        {
            return None;
        }
        self.resolution = Some(map.resolution);
        for (i, v) in map.occupancy.iter().enumerate() {
            if (0..65).contains(v) {
                let x = map.origin_x + (i % map.width as usize) as f64 * map.resolution;
                let y = map.origin_y + (i / map.width as usize) as f64 * map.resolution;
                self.cells.insert((
                    (x / map.resolution).round() as i64,
                    (y / map.resolution).round() as i64,
                ));
            }
        }
        Some(self.cells.len() as f64 * map.resolution.powi(2))
    }
}
