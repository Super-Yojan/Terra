//! The status each tick publishes: one full JSON record for dashboards and
//! the apps, and a typed brief of the few fields hosts act on every tick.
use super::*;
use terra_autonomy::{Level, Reason, Safety};
use terra_waypoint::WaypointStatus;

/// The fields of the latest tick a host reads at control rate, without the
/// full status (the L3 planner trace alone is about 30 KB).
#[derive(Clone, Debug, PartialEq)]
pub struct Brief {
    pub run_id: String,
    /// Authority revision the tick ran under.
    pub revision: u64,
    pub requested_level: Level,
    pub effective_level: Option<Level>,
    pub safety: Safety,
    pub reason: String,
    pub pose_degraded: bool,
    pub pose_age_s: Option<f64>,
    pub goal: WaypointStatus,
}

impl Runtime {
    /// Full status JSON from the latest tick; empty before the first tick.
    pub fn status_json(&self) -> &str {
        &self.status_json
    }
    /// The latest tick's brief; `None` before the first tick.
    pub fn brief(&self) -> Option<&Brief> {
        self.brief.as_ref()
    }
    /// [`Self::status_json`] cut to `status.{run_id, requested_level,
    /// effective_level, safety, reason}`, `pose_quality` and `goal.state`.
    pub fn brief_json(&self) -> &str {
        &self.brief_json
    }

    /// An outdoor mission replaces the goal with its own and, while it holds
    /// motion, names its hold as the reason. Returns the outdoor status.
    pub(crate) fn apply_outdoor_status(
        &self,
        output: &mut ArbiterOutput,
        outdoor_motion: bool,
        manual: bool,
        now: f64,
    ) -> Value {
        let mut status = self
            .outdoor
            .status(now, outdoor::evidence(self, now), &self.run_id);
        self.outdoor_admission.annotate(&mut status);
        if !self.outdoor.selected() {
            return status;
        }
        if let Some(goal) = self.outdoor.mission_status() {
            output.goal = goal;
        }
        let emergency = output.status.safety == Safety::EmergencyStop;
        let hold_reason = |status: &Value| {
            status["hold_reason"]
                .as_str()
                .unwrap_or(self.outdoor.phase().as_str())
                .to_string()
        };
        if !outdoor_motion && !manual && !emergency {
            output.status.reason = hold_reason(&status);
        }
        if self.outdoor.phase() == terra_autonomy::outdoor::OutdoorPhase::Planning
            && Reason::known(&output.status.reason).is_some_and(Reason::is_route_blocked)
        {
            status["hold_reason"] = json!("route_blocked");
        }
        // Explore and task keep their own hold reason: the outdoor mission does not gate them.
        if output.status.safety != Safety::Clear
            && !emergency
            && !self.autonomy.reads_outdoor_destination()
        {
            output.status.reason = hold_reason(&status);
        }
        status
    }

    /// Record this tick's status, brief and JSON. Proposal expiry is
    /// published relative to the run's first tick.
    pub(crate) fn publish_status(
        &mut self,
        output: &ArbiterOutput,
        outdoor: Value,
        runtime: Value,
        now: f64,
    ) -> Value {
        let quality = pose_quality::value(self, now);
        let start = *self.run_start.get_or_insert(now);
        let proposal = output.proposal.clone().map(|mut p| {
            p.expires_at -= start;
            p
        });
        let mut connectivity = self.connectivity.status();
        connectivity["accepted_onboard_mission"] = json!(self.accepted_onboard_mission());
        let status = json!({
            "status": output.status,
            "command_origin": output.command_origin,
            "runtime": runtime,
            "pose_quality": quality,
            "outdoor": outdoor,
            "navigation": self.navigation.status(),
            "connectivity": connectivity,
            "goal": output.goal,
            "proposal": proposal,
            "exploration": output.exploration,
            "search": output.search,
            "task": output.task,
            "pending_reports": output.pending_reports,
            "twist": output.twist,
        });
        self.brief_json = json!({
            "status": {
                "run_id": output.status.run_id,
                "requested_level": output.status.requested_level,
                "effective_level": output.status.effective_level,
                "safety": output.status.safety,
                "reason": output.status.reason,
            },
            "pose_quality": quality,
            "goal": {"state": output.goal.state},
        })
        .to_string();
        self.brief = Some(Brief {
            run_id: output.status.run_id.clone(),
            revision: output.status.revision,
            requested_level: output.status.requested_level,
            effective_level: output.status.effective_level,
            safety: output.status.safety,
            reason: output.status.reason.clone(),
            pose_degraded: quality["degraded"] != false,
            pose_age_s: quality["pose_age_s"].as_f64(),
            goal: output.goal.clone(),
        });
        self.status_json = status.to_string();
        status
    }
}
