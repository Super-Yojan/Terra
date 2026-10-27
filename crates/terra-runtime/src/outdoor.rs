//! Source admission and integration with the existing L3 local planner.
use crate::*;
use terra_autonomy::outdoor::Evidence;
fn invalid(s: &str) -> RuntimeError {
    RuntimeError::InvalidInput { message: s.into() }
}
pub fn evidence(b: &Runtime, now: f64) -> Evidence {
    let h = b
        .hardware_observation
        .as_ref()
        .filter(|(_, t)| (0.0..=timing::HARDWARE_FRESHNESS_SECONDS).contains(&(now - *t)))
        .map(|(h, _)| h);
    Evidence {
        pose: b.pose.unwrap_or_default().0,
        pose_ready: pose_quality::ready(b, now),
        map_ready: b.map.as_ref().is_some_and(|(_, t, _)| {
            now.is_finite() && (0.0..=b.autonomy.map_freshness_timeout()).contains(&(now - t))
        }),
        hardware_ready: h.is_some_and(|h| h["ready"] == true && h["fault"].is_null()),
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
    if bytes.len() > 2048
        || !age.is_finite()
        || !(0.0..timing::CONTROL_REQUEST_FRESHNESS_SECONDS).contains(&age)
    {
        return Err(invalid("request expired"));
    }
    let v: serde_json::Value =
        serde_json::from_slice(bytes).map_err(|_| invalid("invalid request"))?;
    let token = v["token"]
        .as_str()
        .filter(|t| terra_autonomy::valid_token(t))
        .ok_or_else(|| invalid("invalid token"))?;
    if b.outdoor_tokens.iter().any(|t| t == token) {
        return Err(invalid("replayed request"));
    }
    if b.outdoor_tokens.len() >= 128 {
        b.outdoor_tokens.pop_front();
    }
    b.outdoor_tokens.push_back(token.into());
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
        if kind == "navigation/resume" {
            return b.outdoor.resume(now, e).map_err(invalid);
        }
        let source = v["source"]
            .as_str()
            .ok_or_else(|| invalid("invalid source"))?;
        if !matches!(source, "local" | "outdoor_gps") {
            return Err(invalid("invalid source"));
        }
        let output_idle = b.hardware_observation.as_ref().is_some_and(|(h, t)| {
            (0.0..=timing::HARDWARE_FRESHNESS_SECONDS).contains(&(now - *t))
                && h["ready"] == true
                && h["output_active"] == false
        });
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
        b.outdoor.revision = b
            .outdoor
            .revision
            .checked_add(1)
            .ok_or_else(|| invalid("revision exhausted"))?;
        b.outdoor.cancel();
        b.outdoor.selected = source == "outdoor_gps";
        b.outdoor.radius = radius;
        b.autonomy
            .accept_goal(&terra_waypoint::GoalCommand::Cancel, 20_000.);
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
        return b.autonomy.accept_goal(g, 20_000.);
    }
    if !b.outdoor.selected {
        return b.autonomy.accept_goal(g, 20_000.);
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
    if b.outdoor_goal_tokens.contains(token) {
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
    if b.outdoor_goal_tokens.len() >= 128 {
        b.outdoor_goal_tokens.pop_front();
    }
    b.outdoor_goal_tokens.push_back(token.clone());
    let target = b.outdoor.tick(now, e).unwrap_or((e.pose.x, e.pose.y));
    let local = GoalCommand::Local {
        x: target.0,
        y: target.1,
        yaw: None,
        token: Some(token.clone()),
    };
    if !b.autonomy.accept_goal(&local, 20_000.) {
        b.outdoor.cancel();
        return false;
    }
    true
}
pub fn update(b: &mut Runtime, now: f64) -> bool {
    if !b.outdoor.selected {
        return true;
    }
    let e = evidence(b, now);
    if let Some((x, y)) = b.outdoor.tick(now, e) {
        b.autonomy.update_guidance(x, y);
    }
    // Explore owns the follower in the local frame. The tick above has already
    // paused any outdoor mission for lost authority; it waits for an explicit resume.
    if b.autonomy.is_exploring() {
        return true;
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
    if bytes.len() > 2048 {
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
            || !(0.0..timing::CONTROL_REQUEST_FRESHNESS_SECONDS).contains(&age))
    {
        b.autonomy.reject_goal(&g, "outdoor_review_expired");
        return false;
    }
    goal(b, &g, now, radius.unwrap_or(b.outdoor.radius))
}
