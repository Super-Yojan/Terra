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
pub fn value(b: &Runtime, now: f64) -> serde_json::Value {
    let health = b.estimator.estimate(now).health;
    let reason = match health {
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
    };
    serde_json::json!({"schema_version":1,"rover_id":b.rover_id,"run_id":b.run_id,"sequence":b.sequence,"degraded":!ready(b,now),"reason":reason,"pose_age_s":age(now,b.pose.map(|p|p.1)),"imu_age_s":age(now,b.imu_time)})
}
