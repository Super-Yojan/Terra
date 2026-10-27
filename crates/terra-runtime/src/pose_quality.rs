//! Explicit source health, including when pose publication is suppressed.
use super::*;
pub const FRESHNESS_SECONDS: f64 = timing::VIO_FRESHNESS_SECONDS;
fn age(now: f64, time: Option<f64>) -> Option<f64> {
    time.filter(|t| now.is_finite() && t.is_finite() && *t <= now)
        .map(|t| now - t)
}
pub fn ready(b: &Runtime, now: f64) -> bool {
    b.estimator.estimate(now).health == Health::Ready
        && b.pose.is_some_and(|(p, t)| {
            p.finite() && age(now, Some(t)).is_some_and(|a| a <= FRESHNESS_SECONDS)
        })
}
fn reason(b: &Runtime, now: f64) -> &'static str {
    match b.estimator.estimate(now).health {
        Health::Ready => {
            if ready(b, now) {
                "ready"
            } else {
                "pose_stale"
            }
        }
        Health::MissingImu => "missing_imu",
        Health::MissingVio => "missing_vio",
        Health::TrackingLost => "tracking_lost",
        Health::StaleImu => "stale_imu",
        Health::StaleVio => "stale_vio",
        Health::InvalidTime => "invalid_time",
    }
}
/// The input that made this tick `sensor_unhealthy`, in the order the
/// runtime and the arbiter test them, with the sensor ages at that moment.
pub fn hold(b: &Runtime, now: f64, outdoor_motion: bool) -> terra_autonomy::SensorHold {
    let pose_age = age(now, b.pose.map(|p| p.1));
    let estimate = b.estimator.estimate(now);
    let cause = if !b.recording_healthy {
        "recording_failed"
    } else if !outdoor_motion {
        "outdoor_recovery"
    } else {
        match reason(b, now) {
            "ready" if !b.pose.is_some_and(|(p, _)| p.finite()) => "pose_invalid",
            "ready" if pose_age.is_none_or(|a| a >= timing::POSE_FRESHNESS_SECONDS) => "pose_stale",
            "ready" if !estimate.forward.is_finite() || !estimate.yaw_rate.is_finite() => {
                "velocity_invalid"
            }
            "ready" => "unknown",
            other => other,
        }
    };
    terra_autonomy::SensorHold {
        cause: cause.into(),
        pose_age_s: pose_age,
        imu_age_s: age(now, b.imu_time),
        age_s: 0.,
        count: 0,
    }
}
pub fn value(b: &Runtime, now: f64) -> serde_json::Value {
    let reason = reason(b, now);
    serde_json::json!({"schema_version":1,"rover_id":b.rover_id,"run_id":b.run_id,"sequence":b.sequence,"degraded":!ready(b,now),"reason":reason,"pose_age_s":age(now,b.pose.map(|p|p.1)),"imu_age_s":age(now,b.imu_time)})
}
