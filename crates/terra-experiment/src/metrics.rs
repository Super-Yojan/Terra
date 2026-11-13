//! Offline metrics computed from recorded experiment events.

use serde::{Deserialize, Serialize};
use serde_json::Value;
use terra_autonomy::{EventKind, SearchPhase, Source, accepted_reason};
/// Offline run outcomes; durations use seconds and rates use interventions per minute.
#[derive(Debug, Default, Serialize, Deserialize)]
pub struct RunSummary {
    /// Whether the recording or mission completed successfully, according to this record.
    pub complete: bool,
    /// Maximum recorded control-tick time, in seconds from run start.
    pub duration: f64,
    /// Accepted authority changes whose reason is [`accepted_reason::OPERATOR_SELECTION`].
    pub mode_changes: u64,
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
    let mut previous = std::collections::BTreeMap::<u64, (f64, Source)>::new();
    let mut terminal_searches = std::collections::BTreeSet::new();
    for r in records {
        if r["kind"] == "run_end" {
            out.complete = r["complete"] == true;
        }
        if r["kind"] == crate::CONTROL_TICK {
            let time = r["time"].as_f64().unwrap_or(0.);
            out.duration = out.duration.max(time);
            let id = r["rover_id"].as_u64().unwrap_or(0);
            let source = Source::deserialize(&r["status"]["active_source"]).unwrap_or_default();
            if let Some((old, src)) = previous.get(&id)
                && matches!(src, Source::Operator | Source::AssistedOperator)
            {
                out.operator_control_seconds += (time - old).clamp(0., 0.5);
            }
            previous.insert(id, (time, source));
            if let (Some(id), Ok(phase)) = (
                r["search"]["search_id"].as_str(),
                SearchPhase::deserialize(&r["search"]["phase"]),
            ) {
                if phase.terminal()
                    && terminal_searches.insert((
                        r["rover_id"].as_u64().unwrap_or(0),
                        r["run_id"].as_str().unwrap_or("").to_string(),
                        id.to_string(),
                    ))
                {
                    if phase == SearchPhase::Completed {
                        out.search_confirmations += 1;
                        out.time_to_target_confirmation.get_or_insert(time);
                    } else if phase == SearchPhase::Exhausted {
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
                    let kind = e["kind"].as_str().unwrap_or_default();
                    if kind == EventKind::RequestAccepted.as_str() {
                        match e["reason"].as_str().unwrap_or_default() {
                            accepted_reason::OPERATOR_SELECTION => out.mode_changes += 1,
                            accepted_reason::PROPOSAL_APPROVE => out.approvals += 1,
                            accepted_reason::PROPOSAL_REJECT => out.rejections += 1,
                            _ => {}
                        }
                    } else if kind == EventKind::RouteFailed.as_str() {
                        out.search_route_failures += 1;
                    } else if kind == EventKind::Takeover.as_str() {
                        out.takeovers += 1;
                    } else if kind == EventKind::GoalAccepted.as_str() {
                        out.redirects += 1;
                    } else if kind == EventKind::NearMissStart.as_str() {
                        out.near_misses += 1;
                    } else if kind == EventKind::ContactStart.as_str() {
                        out.contacts += 1;
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
    /// Returns `None` for an invalid snapshot (see `MapSnapshot::valid`) or a resolution change, preserving state.
    pub fn observe(&mut self, map: &terra_mapping::MapSnapshot) -> Option<f64> {
        if !map.valid() || self.resolution.is_some_and(|r| r != map.resolution) {
            return None;
        }
        self.resolution = Some(map.resolution);
        for (i, v) in map.occupancy.iter().enumerate() {
            if (0..terra_mapping::OCCUPIED).contains(v) {
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
