//! `sensor_unhealthy` holds: naming their cause in the status, the settle wait
//! after a long hold, and dropping a stale goal when a hold lasts too long.
use super::Runtime;
use serde_json::json;
use terra_autonomy::{ArbiterOutput, SensorHold};

/// How long a `sensor_unhealthy` hold stays in the status after it clears.
const SENSOR_HOLD_DISPLAY_SECONDS: f64 = 10.;
/// A hold at least this long must see fresh data for
/// [`SENSOR_SETTLE_SECONDS`] before motion resumes. Shorter holds resume on
/// the next fresh tick, as before.
const SENSOR_SETTLE_AFTER_SECONDS: f64 = 1.;
/// How long data must stay fresh after a long hold before the rover drives.
const SENSOR_SETTLE_SECONDS: f64 = 0.5;
/// An unbroken hold this long drops the active L2/L3 goal, so the rover does
/// not lurch toward an old goal minutes later. The operator resends it.
pub const SENSOR_HOLD_GOAL_TIMEOUT_SECONDS: f64 = 30.;

/// Hold bookkeeping for one run; a new run starts from the default.
#[derive(Default)]
pub(crate) struct SensorHolds {
    /// Latest hold and when it happened.
    last: Option<(f64, SensorHold)>,
    /// Holds started this run.
    count: u64,
    /// Whether the previous tick was held.
    held: bool,
    /// Start of the current unbroken hold.
    since: Option<f64>,
    /// When data came back fresh after a long hold; motion waits out
    /// [`SENSOR_SETTLE_SECONDS`] from here.
    settle_since: Option<f64>,
    /// The current hold already dropped its goal.
    goal_dropped: bool,
}

impl Runtime {
    /// Whether fresh data must wait out the settle period after a long hold.
    /// Called once per tick with the health the sensors report on their own.
    pub(crate) fn settling(&mut self, healthy: bool, now: f64) -> bool {
        let h = &mut self.holds;
        let Some(since) = h.since.filter(|_| healthy) else {
            h.settle_since = None;
            return false;
        };
        let back = *h.settle_since.get_or_insert(now);
        back - since >= SENSOR_SETTLE_AFTER_SECONDS && now - back < SENSOR_SETTLE_SECONDS
    }

    /// Name the cause of a `sensor_unhealthy` hold in the status, and log
    /// each new hold, so holds of a single tick are not silent. A hold that
    /// outlasts [`SENSOR_HOLD_GOAL_TIMEOUT_SECONDS`] drops the L2/L3 goal.
    pub(crate) fn note_sensor_hold(
        &mut self,
        output: &mut ArbiterOutput,
        now: f64,
        outdoor_motion: bool,
        settling: bool,
    ) {
        if output.status.reason == "sensor_unhealthy" {
            let since = *self.holds.since.get_or_insert(now);
            if settling && let Some((at, _)) = self.holds.last.as_mut() {
                // Keep the cause that started the hold while data settles.
                *at = now;
            } else {
                let mut hold = super::pose_quality::hold(self, now, outdoor_motion);
                hold.goal_dropped = self.holds.goal_dropped;
                if !self.holds.held {
                    self.holds.count += 1;
                    self.events.push(json!({"kind":"autonomy.sensor_hold","cause":hold.cause,"pose_age_s":hold.pose_age_s,"imu_age_s":hold.imu_age_s,"count":self.holds.count}));
                }
                self.holds.last = Some((now, hold));
            }
            self.holds.held = true;
            if !self.holds.goal_dropped
                && !self.outdoor.selected
                && now - since >= SENSOR_HOLD_GOAL_TIMEOUT_SECONDS
                && self
                    .autonomy
                    .drop_goal_for_sensor_hold(self.pose.unwrap_or_default().0)
            {
                self.holds.goal_dropped = true;
                if let Some((_, hold)) = self.holds.last.as_mut() {
                    hold.goal_dropped = true;
                }
                self.events.push(json!({"kind":"autonomy.goal_dropped","reason":"sensor_hold_timeout","held_s":now - since}));
            }
        } else {
            if self.holds.held
                && let Some(since) = self.holds.since
            {
                self.events
                    .push(json!({"kind":"autonomy.sensor_hold_cleared","held_s":now - since}));
            }
            self.holds.held = false;
            self.holds.since = None;
            self.holds.settle_since = None;
        }
        let h = &mut self.holds;
        let held_s = h.since.map_or(0., |since| now - since);
        if !h.held {
            h.goal_dropped = false;
        }
        output.status.sensor_hold = h
            .last
            .as_ref()
            .filter(|(at, _)| now - at <= SENSOR_HOLD_DISPLAY_SECONDS)
            .map(|(at, hold)| SensorHold {
                age_s: now - at,
                count: h.count,
                held_s,
                settling: settling && h.held,
                ..hold.clone()
            });
    }
}
