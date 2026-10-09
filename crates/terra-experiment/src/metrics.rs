use serde::{Deserialize, Serialize};
use serde_json::Value;
#[derive(Default)]
pub struct NearMiss {
    active: bool,
}
impl NearMiss {
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
#[derive(Debug, Default, Serialize, Deserialize)]
pub struct RunSummary {
    pub complete: bool,
    pub duration: f64,
    pub mode_changes: u64,
    pub takeovers: u64,
    pub interventions_per_minute: f64,
    pub time_to_first_discovery: Option<f64>,
    pub mission_completion_time: Option<f64>,
    pub approvals: u64,
    pub rejections: u64,
    pub redirects: u64,
    pub near_misses: u64,
    pub contacts: u64,
    pub operator_control_seconds: f64,
    pub mission_complete: bool,
    #[serde(default)] pub search_confirmations:u64,
    #[serde(default)] pub search_exhaustions:u64,
    #[serde(default)] pub search_route_failures:u64,
    #[serde(default)] pub time_to_target_confirmation:Option<f64>,
}
pub fn summarize(records: &[Value]) -> RunSummary {
    let mut out = RunSummary::default();
    let mut previous = std::collections::BTreeMap::<u64, (f64, String)>::new();
    let mut terminal_searches=std::collections::BTreeSet::new();
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
            if let (Some(id),Some(phase))=(r["search"]["search_id"].as_str(),r["search"]["phase"].as_str()) {
                if matches!(phase,"completed"|"exhausted"|"timed_out"|"cancelled") && terminal_searches.insert((r["rover_id"].as_u64().unwrap_or(0),r["run_id"].as_str().unwrap_or("").to_string(),id.to_string())) {
                    if phase=="completed" {out.search_confirmations+=1;out.time_to_target_confirmation.get_or_insert(time);}else if phase=="exhausted" {out.search_exhaustions+=1;}
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
