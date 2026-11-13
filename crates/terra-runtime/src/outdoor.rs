//! Source admission and integration with the existing L3 local planner.
use crate::*;
use terra_autonomy::outdoor::Evidence;
use terra_waypoint::MAX_LOCAL_GOAL_M;

/// Replay memory for outdoor source requests and geographic goals.
#[derive(Default)]
pub(crate) struct Admission {
    requests: replay::ReplayGuard<128>,
    goals: replay::ReplayGuard<128>,
}

pub fn evidence(b: &Runtime, now: f64) -> Evidence {
    Evidence {
        pose: b.pose.unwrap_or_default().0,
        pose_ready: pose_quality::ready(b, now),
        // L2 direct drives without the occupancy map, so a stale map never holds it.
        map_ready: !b.autonomy.needs_map()
            || b.map.as_ref().is_some_and(|(_, t, _)| {
                now.is_finite() && (0.0..=b.autonomy.map_freshness_timeout()).contains(&(now - t))
            }),
        hardware_ready: b.hardware.is_some_and(|h| h.fresh_and_ready(now)),
        authority: b.autonomy.outdoor_authority(),
    }
}
pub fn request(
    b: &mut Runtime,
    kind: &str,
    bytes: &[u8],
    now: f64,
    age: f64,
) -> Result<(), RuntimeError> {
    if bytes.len() > requests::MAX_REQUEST_BYTES
        || !age.is_finite()
        || !(0.0..CONTROL_REQUEST_FRESHNESS_SECONDS).contains(&age)
    {
        return Err(invalid("request expired"));
    }
    let v: serde_json::Value =
        serde_json::from_slice(bytes).map_err(|_| invalid("invalid request"))?;
    let token = v["token"]
        .as_str()
        .filter(|t| terra_autonomy::valid_token(t))
        .ok_or_else(|| invalid("invalid token"))?;
    if !b.outdoor_admission.requests.admit(token) {
        return Err(invalid("replayed request"));
    }
    b.outdoor.token = Some(token.into());
    let result = (|| {
        if v["schema_version"] != 1
            || v["run_id"] != b.run_id
            || v["navigation_revision"] != b.outdoor.revision
        {
            return Err(invalid("run or navigation revision changed"));
        }
        if !b.outdoor.supported {
            return Err(invalid("outdoor GPS unsupported"));
        }
        let e = evidence(b, now);
        if kind == requests::NAVIGATION_RESUME {
            return b.outdoor.resume(now, e).map_err(invalid);
        }
        let source = v["source"]
            .as_str()
            .ok_or_else(|| invalid("invalid source"))?;
        if !matches!(source, "local" | "outdoor_gps") {
            return Err(invalid("invalid source"));
        }
        let output_idle = b.hardware.is_some_and(|h| h.fresh_and_idle(now));
        if !output_idle || b.outdoor.active() || b.autonomy.has_active_goal(e.pose) {
            return Err(invalid(
                "stop output and cancel mission before changing source",
            ));
        }
        let radius = v
            .get("arrival_radius_m")
            .and_then(|v| v.as_f64())
            .unwrap_or(b.outdoor.radius);
        if !radius.is_finite() || !(2.0..=20.).contains(&radius) {
            return Err(invalid("invalid arrival radius"));
        }
        if b.outdoor.revision == u64::MAX {
            return Err(invalid("revision exhausted"));
        }
        b.bump_navigation_revision();
        b.outdoor.cancel();
        b.outdoor.selected = source == "outdoor_gps";
        b.outdoor.radius = radius;
        b.autonomy
            .accept_goal(&terra_waypoint::GoalCommand::Cancel, MAX_LOCAL_GOAL_M);
        b.controller.reset();
        Ok(())
    })();
    b.outdoor.result = Some(match &result {
        Ok(()) => "accepted".into(),
        Err(e) => format!("rejected: {e}"),
    });
    result
}
pub fn goal(b: &mut Runtime, g: &terra_waypoint::GoalCommand, now: f64, radius: f64) -> bool {
    use terra_waypoint::GoalCommand;
    if matches!(g, GoalCommand::Cancel) {
        b.outdoor.cancel();
        return b.autonomy.accept_goal(g, MAX_LOCAL_GOAL_M);
    }
    if !b.outdoor.selected {
        return b.autonomy.accept_goal(g, MAX_LOCAL_GOAL_M);
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
    if !b.autonomy.accept_goal(&local, MAX_LOCAL_GOAL_M) {
        b.outdoor.cancel();
        return false;
    }
    true
}
pub fn update(b: &mut Runtime, now: f64) -> bool {
    if !b.outdoor.selected {
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
        if b.outdoor.phase() == "arrived" {
            b.autonomy.finish_explore_at_destination();
        }
        return true;
    }
    b.autonomy.set_outdoor_destination(None);
    if let Some((x, y)) = guidance {
        b.autonomy.update_guidance(x, y);
    }
    if b.outdoor.phase() != "planning" {
        // Never retain a moving local target through a pause or arrival confirmation.
        // With recovered sensors, an idle follower permits an explicit mission resume.
        b.autonomy.clear_guidance();
    }
    !b.outdoor.active() || b.outdoor.phase() == "planning" || b.outdoor.recovery_ready(now, e)
}

/// Decode the outdoor envelope before stripping its reviewed metadata for the legacy local decoder.
pub fn dispatch_goal(b: &mut Runtime, bytes: &[u8], now: f64, age: f64) -> bool {
    if bytes.len() > terra_types::requests::MAX_REQUEST_BYTES {
        return false;
    }
    let Ok(mut value) = serde_json::from_slice::<serde_json::Value>(bytes) else {
        return false;
    };
    let outdoor = b.outdoor.selected && value["cancel"] != true;
    let radius = value["arrival_radius_m"].as_f64();
    let bound = value["run_id"] == b.run_id && value["navigation_revision"] == b.outdoor.revision;
    if outdoor {
        if let Some(o) = value.as_object_mut() {
            o.remove("run_id");
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
    if outdoor
        && (!bound
            || radius.is_none()
            || !age.is_finite()
            || !(0.0..CONTROL_REQUEST_FRESHNESS_SECONDS).contains(&age))
    {
        b.autonomy.reject_goal(&g, "outdoor_review_expired");
        return false;
    }
    goal(b, &g, now, radius.unwrap_or(b.outdoor.radius))
}
