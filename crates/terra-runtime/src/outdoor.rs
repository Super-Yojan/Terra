//! Source admission and integration with the existing L3 local planner.
use crate::*;
use terra_autonomy::outdoor::{Evidence, NavigationSource, OutdoorPhase};

/// Outdoor request admission: replay memory for source requests and
/// geographic goals, and the last source request's token and result.
#[derive(Default)]
pub(crate) struct Admission {
    requests: replay::ReplayGuard<128>,
    goals: replay::ReplayGuard<128>,
    token: Option<String>,
    result: Option<String>,
}
impl Admission {
    /// Add the request bookkeeping, and the settings revision requests are
    /// reviewed against, to the controller's `outdoor` status.
    pub(crate) fn annotate(&self, status: &mut serde_json::Value, revision: u64) {
        status["navigation_revision"] = revision.into();
        status["token"] = self.token.clone().into();
        status["result"] = self.result.clone().into();
    }
}

pub(crate) fn evidence(b: &Runtime, now: f64) -> Evidence {
    Evidence {
        pose: b.pose.unwrap_or_default().0,
        pose_ready: pose_quality::ready(b, now),
        // L2 direct drives without the occupancy map, so a stale map never holds it.
        map_ready: !b.autonomy.needs_map()
            || b.autonomy.map_fresh(now, b.map.as_ref().map(|m| m.1)),
        hardware_ready: b.hardware.is_some_and(|h| h.fresh_and_ready(now)),
        authority: b.autonomy.outdoor_authority(),
    }
}
/// A `navigation/source` or `navigation/resume` request, already admitted
/// for size, age and authority.
pub(crate) fn request(
    b: &mut Runtime,
    kind: &str,
    bytes: &[u8],
    now: f64,
) -> Result<(), RuntimeError> {
    let v: serde_json::Value =
        serde_json::from_slice(bytes).map_err(|_| invalid("invalid request"))?;
    let token = v["token"]
        .as_str()
        .filter(|t| terra_autonomy::valid_token(t))
        .ok_or_else(|| invalid("invalid token"))?;
    if !b.outdoor_admission.requests.admit(token) {
        return Err(invalid("replayed request"));
    }
    b.outdoor_admission.token = Some(token.into());
    let result = (|| {
        if v["schema_version"] != 1
            || v["run_id"] != b.run_id
            || v["navigation_revision"] != b.navigation.revision
        {
            return Err(invalid("run or navigation revision changed"));
        }
        if !b.outdoor.supported() {
            return Err(invalid("outdoor GPS unsupported"));
        }
        let e = evidence(b, now);
        if kind == requests::NAVIGATION_RESUME {
            return b.outdoor.resume(now, e).map_err(invalid);
        }
        let source = v["source"]
            .as_str()
            .and_then(NavigationSource::parse)
            .ok_or_else(|| invalid("invalid source"))?;
        let output_idle = b.hardware.is_some_and(|h| h.fresh_and_idle(now));
        if !output_idle || b.outdoor.active() || b.autonomy.has_active_goal(e.pose) {
            return Err(invalid(
                "stop output and cancel mission before changing source",
            ));
        }
        let radius = v
            .get("arrival_radius_m")
            .and_then(|v| v.as_f64())
            .unwrap_or(b.outdoor.arrival_radius());
        if b.navigation.revision == u64::MAX {
            return Err(invalid("revision exhausted"));
        }
        b.outdoor.select_source(source, radius).map_err(invalid)?;
        b.settings_changed();
        b.autonomy.accept_goal(&terra_waypoint::GoalCommand::Cancel);
        b.controller.reset();
        Ok(())
    })();
    b.outdoor_admission.result = Some(match &result {
        Ok(()) => "accepted".into(),
        Err(e) => format!("rejected: {e}"),
    });
    result
}
fn goal(b: &mut Runtime, g: &terra_waypoint::GoalCommand, now: f64, radius: f64) -> bool {
    use terra_waypoint::GoalCommand;
    if matches!(g, GoalCommand::Cancel) {
        b.outdoor.cancel();
        return b.autonomy.accept_goal(g);
    }
    if !b.outdoor.selected() {
        return b.autonomy.accept_goal(g);
    }
    let GoalCommand::Wgs84 {
        latitude,
        longitude,
        token: Some(token),
        yaw: None,
    } = g
    else {
        b.autonomy
            .reject_goal(g, "outdoor_requires_geographic_goal");
        return false;
    };
    // Replay never restarts a paused/completed mission or replaces its reviewed radius.
    if b.outdoor_admission.goals.seen(token) {
        b.autonomy.reject_goal(g, "replayed_goal");
        return false;
    }
    let e = evidence(b, now);

    if let Err(reason) = b
        .outdoor
        .start(*latitude, *longitude, token.clone(), radius, now, e)
    {
        b.autonomy.reject_goal(g, reason);
        return false;
    }
    b.outdoor_admission.goals.record(token);
    if b.autonomy.reads_outdoor_destination() {
        // L4 explore and L5 task keep their own follower and read the destination each tick.
        b.autonomy
            .set_outdoor_destination(b.outdoor.destination_local(e.pose));
        b.autonomy.replan_task_for_destination();
        return true;
    }
    let target = b.outdoor.tick(now, e).unwrap_or((e.pose.x, e.pose.y));
    let local = GoalCommand::Local {
        x: target.0,
        y: target.1,
        yaw: None,
        token: Some(token.clone()),
    };
    if !b.autonomy.accept_goal(&local) {
        b.outdoor.cancel();
        return false;
    }
    true
}
pub(crate) fn update(b: &mut Runtime, now: f64) -> bool {
    if !b.outdoor.selected() {
        b.autonomy.set_outdoor_destination(None);
        return true;
    }
    let e = evidence(b, now);
    let guidance = b.outdoor.tick(now, e);
    if b.autonomy.reads_outdoor_destination() {
        // Explore and task own the follower in the local frame and plan toward the
        // destination on the occupancy map. A paused mission only withdraws the
        // destination: the level keeps working locally until an explicit resume.
        b.autonomy
            .set_outdoor_destination(b.outdoor.destination_local(e.pose));
        if b.outdoor.phase() == OutdoorPhase::Arrived {
            b.autonomy.finish_explore_at_destination();
        }
        return true;
    }
    b.autonomy.set_outdoor_destination(None);
    if let Some((x, y)) = guidance {
        b.autonomy.update_guidance(x, y);
    }
    if b.outdoor.phase() != OutdoorPhase::Planning {
        // Never retain a moving local target through a pause or arrival confirmation.
        // With recovered sensors, an idle follower permits an explicit mission resume.
        b.autonomy.clear_guidance();
    }
    !b.outdoor.active()
        || b.outdoor.phase() == OutdoorPhase::Planning
        || b.outdoor.recovery_ready(now, e)
}

/// Decode a goal request, already admitted for size, age and authority. An
/// outdoor goal carries its review (run, navigation revision and arrival
/// radius); it is checked here and removed, with `run_id`, before the
/// waypoint decoder.
pub(crate) fn dispatch_goal(b: &mut Runtime, bytes: &[u8], now: f64) -> bool {
    let Ok(mut value) = serde_json::from_slice::<serde_json::Value>(bytes) else {
        return false;
    };
    let outdoor = b.outdoor.selected() && value["cancel"] != true;
    let radius = value["arrival_radius_m"].as_f64();
    let bound =
        value["run_id"] == b.run_id && value["navigation_revision"] == b.navigation.revision;
    if let Some(o) = value.as_object_mut() {
        o.remove("run_id");
        if outdoor {
            o.remove("navigation_revision");
            o.remove("arrival_radius_m");
        }
    }
    let Some(g) = terra_waypoint::decode_goal(&serde_json::to_vec(&value).unwrap()) else {
        return false;
    };
    if !matches!(g, terra_waypoint::GoalCommand::Cancel) && !pose_quality::ready(b, now) {
        b.autonomy.reject_goal(&g, "pose_degraded");
        return false;
    }
    if outdoor && (!bound || radius.is_none()) {
        b.autonomy.reject_goal(&g, "outdoor_review_expired");
        return false;
    }
    goal(b, &g, now, radius.unwrap_or(b.outdoor.arrival_radius()))
}
