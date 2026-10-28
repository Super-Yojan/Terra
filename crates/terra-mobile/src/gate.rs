//! Motor gate and tick schedule shared by both phone apps.
//!
//! Each control tick the app reports what it knows about its Bluetooth link and
//! camera tracking; the gate reads the controller's own autonomy brief and
//! output and answers with one [`MotorCommand`]: drive this actuator frame,
//! stop output, or send nothing. This is the logic iOS keeps in
//! `PhoneController.update()` / `routeHardware`, so both apps route the same
//! Rust frame through the same checks. The e-stop latch survives
//! [`MotorGate::reset`]; only [`MotorGate::clear_emergency_stop`] clears it.
use crate::{ControlOutput, ControllerError, MobileController, SafetyState};
use std::sync::{Arc, Mutex};

/// Levels that drive on the controller's own targets instead of operator effort.
const AUTONOMOUS_LEVELS: [&str; 6] = [
    "waypoint",
    "waypoint_direct",
    "supervised",
    "explore",
    "target_search",
    "task",
];
/// Levels whose motion estimate is phone VIO (L3 route, and L5 on that route).
const VIO_LEVELS: [&str; 2] = ["waypoint", "task"];
/// Pose older than this is not fresh enough for closed-loop motion.
const MAX_POSE_AGE: f64 = 0.35;

/// The fields of [`MobileController::autonomy_brief`] the phone acts on,
/// parsed once in Rust instead of in Swift or Kotlin every tick.
#[derive(Clone, Debug, Default, PartialEq, uniffi::Record)]
pub struct AutonomyBrief {
    /// Operator-selected level, e.g. `teleop`, `waypoint`; empty before the first tick.
    pub requested_level: String,
    /// Level currently allowed to act; `None` during a hold.
    pub effective_level: Option<String>,
    /// Arbiter safety state.
    pub safety: String,
    /// Arbiter reason for the current decision.
    pub reason: String,
    /// Whether the pose estimate is marked degraded.
    pub pose_degraded: bool,
    /// Age of the latest pose, seconds.
    pub pose_age_s: Option<f64>,
    /// `goal.state`, e.g. `active`; empty when none.
    pub goal_state: String,
    /// L0: see [`Self::fully_manual`].
    pub manual: bool,
    /// See [`Self::autonomous`].
    pub autonomous_level: bool,
    /// See [`Self::requires_vio`].
    pub vio_level: bool,
    /// See [`Self::vio_ready`].
    pub pose_fresh: bool,
    /// See [`Self::emergency_stopped`].
    pub router_emergency_stop: bool,
}

impl AutonomyBrief {
    /// Parse the JSON of [`MobileController::autonomy_brief`]; missing fields stay default.
    pub(crate) fn parse(json: &str) -> Self {
        let root: serde_json::Value = serde_json::from_str(json).unwrap_or_default();
        let status = &root["status"];
        let quality = &root["pose_quality"];
        let text = |v: &serde_json::Value| v.as_str().unwrap_or_default().to_owned();
        let mut brief = Self {
            requested_level: text(&status["requested_level"]),
            effective_level: status["effective_level"].as_str().map(str::to_owned),
            safety: text(&status["safety"]),
            reason: text(&status["reason"]),
            pose_degraded: quality["degraded"].as_bool() != Some(false),
            pose_age_s: quality["pose_age_s"].as_f64(),
            goal_state: text(&root["goal"]["state"]),
            ..Self::default()
        };
        brief.manual = brief.fully_manual();
        brief.autonomous_level = brief.autonomous();
        brief.vio_level = brief.requires_vio();
        brief.pose_fresh = brief.vio_ready();
        brief.router_emergency_stop = brief.emergency_stopped();
        brief
    }

    /// L0: the requested level (or, without one, the effective level) is teleop.
    pub fn fully_manual(&self) -> bool {
        let level = if self.requested_level.is_empty() {
            self.effective_level.as_deref().unwrap_or_default()
        } else {
            self.requested_level.as_str()
        };
        level == "teleop"
    }

    /// The requested level drives on autonomous targets.
    pub fn autonomous(&self) -> bool {
        AUTONOMOUS_LEVELS.contains(&self.requested_level.as_str())
    }

    /// The requested or effective level uses phone VIO motion estimates.
    pub fn requires_vio(&self) -> bool {
        VIO_LEVELS.contains(&self.requested_level.as_str())
            || self
                .effective_level
                .as_deref()
                .is_some_and(|level| VIO_LEVELS.contains(&level))
    }

    /// The router latched an emergency stop (what sensor-free manual drive must obey).
    pub fn emergency_stopped(&self) -> bool {
        self.safety == "emergency_stop"
    }

    /// Pose is not degraded and at most 0.35 s old.
    pub fn vio_ready(&self) -> bool {
        !self.pose_degraded
            && self
                .pose_age_s
                .is_some_and(|age| age.is_finite() && (0.0..=MAX_POSE_AGE).contains(&age))
    }
}

#[uniffi::export]
impl MobileController {
    /// [`Self::autonomy_brief`] as a typed record.
    pub fn autonomy_brief_typed(&self) -> AutonomyBrief {
        AutonomyBrief::parse(&self.autonomy_brief())
    }

    /// Whether the current actuator layout has differential drive routes (L3 needs them).
    pub fn layout_supports_vio_control(&self) -> bool {
        self.brain
            .lock()
            .map(|b| {
                b.actuators
                    .as_ref()
                    .is_some_and(|l| l.supports_vio_control())
            })
            .unwrap_or(false)
    }
}

/// What the phone is driving from.
#[derive(Clone, Copy, Debug, PartialEq, Eq, uniffi::Enum)]
pub enum DriveSource {
    /// Phone sensors (ARKit / ARCore + IMU).
    Phone,
    /// The local simulated plant.
    Simulation,
    /// Bluetooth effort with no phone sensors.
    BluetoothManual,
}

/// What the app knows this tick. Everything else comes from the controller.
#[derive(Clone, Copy, Debug, PartialEq, Eq, uniffi::Record)]
pub struct LinkFacts {
    /// A rover Bluetooth link exists.
    pub ble_active: bool,
    /// The link is armed and ready for drive frames.
    pub ble_ready: bool,
    /// Velocity-feedback actuation is selected (otherwise direct effort).
    pub feedback: bool,
    /// The rover reports a layout usable for feedback actuation.
    pub feedback_compatible: bool,
    /// The router (ARGOS) link is connected.
    pub dashboard_connected: bool,
    /// AR tracking is normal on the latest frame.
    pub phone_tracked: bool,
}

/// One tick's answer for the motor link.
#[derive(Clone, Debug, PartialEq, uniffi::Enum)]
pub enum MotorCommand {
    /// Send nothing this tick; the rover's own lease keeps it safe.
    Idle,
    /// Write these actuator values (`[{"id","value"}]` JSON).
    Drive { values_json: String },
    /// Stop output now. `reason` is recorded as the first stop cause.
    Stop { reason: String },
}

/// Which periodic jobs run on a 100 Hz tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq, uniffi::Record)]
pub struct TickPlan {
    /// Route a motor frame (20 Hz).
    pub route: bool,
    /// Publish UI status, read autonomy, service the router link (10 Hz).
    pub publish: bool,
    /// Drain FPV signaling (20 Hz).
    pub fpv: bool,
    /// Service the L5 task model (10 Hz).
    pub task_model: bool,
}

#[uniffi::export]
/// The schedule iOS uses today: 20 Hz motor frames and FPV, 10 Hz status and task model.
pub fn tick_plan(tick: u64) -> TickPlan {
    TickPlan {
        route: tick.is_multiple_of(5),
        publish: tick.is_multiple_of(10),
        fpv: tick.is_multiple_of(5),
        task_model: tick.is_multiple_of(10),
    }
}

/// Shown while motors are held for an unconfirmed mount.
pub const MOUNT_HOLD_REASON: &str =
    "Phone mount not confirmed · confirm the phone is mounted before motors move";

#[derive(Default)]
struct Gate {
    estop: bool,
    mount_confirmed: bool,
    mount_hold_reported: bool,
    first_stop: Option<(String, f64)>,
}

/// Motor gate for one app session. Calls are serialized by a mutex.
#[derive(uniffi::Object)]
pub struct MotorGate {
    state: Mutex<Gate>,
}

#[uniffi::export]
impl MotorGate {
    #[uniffi::constructor]
    /// Mount unconfirmed, no e-stop, no stop cause.
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            state: Mutex::new(Gate::default()),
        })
    }

    /// The person confirmed (or withdrew) that the phone sits in its mount.
    pub fn set_mount_confirmed(&self, confirmed: bool) -> Result<(), ControllerError> {
        let mut g = self.lock()?;
        g.mount_confirmed = confirmed;
        if confirmed {
            g.mount_hold_reported = false;
        }
        Ok(())
    }

    /// Latch an emergency stop (router or local). Motors stay stopped until
    /// [`Self::clear_emergency_stop`].
    pub fn emergency_stop(&self, now: f64) -> Result<(), ControllerError> {
        let mut g = self.lock()?;
        g.estop = true;
        record(&mut g, "Emergency stop", now);
        Ok(())
    }

    /// Clear the latch; only an explicit operator reset should call this.
    pub fn clear_emergency_stop(&self) -> Result<(), ControllerError> {
        self.lock()?.estop = false;
        Ok(())
    }

    /// Whether the e-stop latch is set.
    pub fn emergency_stopped(&self) -> Result<bool, ControllerError> {
        Ok(self.lock()?.estop)
    }

    /// First stop cause since the last [`Self::clear_stop_reason`], with its time.
    pub fn stop_reason(&self) -> Result<Option<String>, ControllerError> {
        Ok(self.lock()?.first_stop.as_ref().map(|(r, _)| r.clone()))
    }

    /// Forget the recorded stop cause.
    pub fn clear_stop_reason(&self) -> Result<(), ControllerError> {
        self.lock()?.first_stop = None;
        Ok(())
    }

    /// Controller stopped: forget the mount-hold report. The e-stop latch and
    /// mount confirmation stay.
    pub fn reset(&self) -> Result<(), ControllerError> {
        self.lock()?.mount_hold_reported = false;
        Ok(())
    }

    /// Decide the motor command for `tick` from the link facts, the
    /// controller's brief and this tick's output. Call once per control tick
    /// after `step`.
    #[allow(clippy::too_many_arguments)]
    pub fn decide(
        &self,
        tick: u64,
        now: f64,
        source: DriveSource,
        facts: LinkFacts,
        brief: AutonomyBrief,
        output: ControlOutput,
        layout_supports_vio: bool,
    ) -> Result<MotorCommand, ControllerError> {
        let mut g = self.lock()?;
        Ok(decide(
            &mut g,
            tick,
            now,
            source,
            facts,
            &brief,
            &output,
            layout_supports_vio,
        ))
    }
}

impl MotorGate {
    fn lock(&self) -> Result<std::sync::MutexGuard<'_, Gate>, ControllerError> {
        self.state.lock().map_err(|_| ControllerError::Internal)
    }
}

fn record(g: &mut Gate, reason: &str, now: f64) {
    if g.first_stop.is_none() && !reason.is_empty() {
        g.first_stop = Some((reason.to_owned(), now));
    }
}

fn stop(g: &mut Gate, reason: String, now: f64) -> MotorCommand {
    record(g, &reason, now);
    MotorCommand::Stop { reason }
}

#[allow(clippy::too_many_arguments)]
fn decide(
    g: &mut Gate,
    tick: u64,
    now: f64,
    source: DriveSource,
    facts: LinkFacts,
    brief: &AutonomyBrief,
    output: &ControlOutput,
    layout_supports_vio: bool,
) -> MotorCommand {
    if !facts.ble_active {
        return MotorCommand::Idle;
    }
    let route = tick_plan(tick).route;
    if source == DriveSource::BluetoothManual {
        // The phone's debug joystick needs no mount; a fleet session does.
        let local_debug = !facts.dashboard_connected;
        return if route {
            frame(g, now, facts, output, local_debug)
        } else {
            MotorCommand::Idle
        };
    }
    let manual = brief.fully_manual();
    let autonomous = brief.autonomous();
    if source == DriveSource::Phone
        && !facts.phone_tracked
        && (facts.feedback || brief.requires_vio())
        && !manual
    {
        return stop(
            g,
            "Camera tracking lost in tracking-dependent mode".into(),
            now,
        );
    }
    if autonomous && !layout_supports_vio {
        return stop(g, "L3 requires differential motor routes".into(), now);
    }
    if (facts.feedback || autonomous) && !manual {
        let healthy = source == DriveSource::Phone && brief.vio_ready();
        if !healthy || output.safety != SafetyState::Active {
            return stop(
                g,
                format!(
                    "Autonomy: {}; feedback controller safety={:?}",
                    if brief.reason.is_empty() {
                        "unavailable"
                    } else {
                        &brief.reason
                    },
                    output.safety
                ),
                now,
            );
        }
        return if route {
            frame(g, now, facts, output, false)
        } else {
            MotorCommand::Idle
        };
    }
    if manual {
        return if route {
            frame(g, now, facts, output, false)
        } else {
            MotorCommand::Idle
        };
    }
    // Direct effort from the arbiter at a non-manual level: only with a fleet attached.
    if route && facts.dashboard_connected {
        return frame(g, now, facts, output, false);
    }
    MotorCommand::Idle
}

fn frame(
    g: &mut Gate,
    now: f64,
    facts: LinkFacts,
    output: &ControlOutput,
    local_debug: bool,
) -> MotorCommand {
    if !facts.ble_ready || g.estop || (facts.feedback && !facts.feedback_compatible) {
        return MotorCommand::Idle;
    }
    if !g.mount_confirmed && !local_debug {
        // Report once per hold; output stays stopped until the mount is confirmed.
        if g.mount_hold_reported {
            return MotorCommand::Idle;
        }
        g.mount_hold_reported = true;
        return stop(g, MOUNT_HOLD_REASON.into(), now);
    }
    match &output.actuator_frame {
        Some(values) => MotorCommand::Drive {
            values_json: values.clone(),
        },
        None => stop(
            g,
            format!(
                "Actuator routing failed: no frame for left={} right={}",
                output.left_effort, output.right_effort
            ),
            now,
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn brief(level: &str, age: f64) -> AutonomyBrief {
        AutonomyBrief::parse(&format!(
            r#"{{"status":{{"requested_level":"{level}","effective_level":"{level}","safety":"ok","reason":"r"}},
                "pose_quality":{{"degraded":false,"pose_age_s":{age}}},"goal":{{"state":"active"}}}}"#
        ))
    }
    fn output(safety: SafetyState, frame: Option<&str>) -> ControlOutput {
        ControlOutput {
            left_effort: 0.2,
            right_effort: 0.3,
            estimated_forward: 0.0,
            estimated_yaw_rate: 0.0,
            target_forward: 0.0,
            target_yaw_rate: 0.0,
            safety,
            actuator_frame: frame.map(str::to_owned),
            gate_servo: None,
        }
    }
    fn facts() -> LinkFacts {
        LinkFacts {
            ble_active: true,
            ble_ready: true,
            feedback: false,
            feedback_compatible: true,
            dashboard_connected: true,
            phone_tracked: true,
        }
    }
    const FRAME: &str = r#"[{"id":1,"value":0.2}]"#;
    fn drive() -> MotorCommand {
        MotorCommand::Drive {
            values_json: FRAME.into(),
        }
    }
    fn confirmed() -> Arc<MotorGate> {
        let gate = MotorGate::new();
        gate.set_mount_confirmed(true).unwrap();
        gate
    }

    #[test]
    /// The brief parses levels, pose freshness and goal, and classifies levels as Swift did.
    fn brief_parsing_and_levels() {
        let b = brief("waypoint", 0.1);
        assert_eq!(b.goal_state, "active");
        assert!(b.vio_ready() && b.autonomous() && b.requires_vio() && !b.fully_manual());
        assert!(!brief("waypoint", 0.5).vio_ready());
        assert!(brief("teleop", 0.0).fully_manual());
        assert!(!brief("explore", 0.0).requires_vio());
        let held =
            AutonomyBrief::parse(r#"{"status":{"requested_level":"task","effective_level":null}}"#);
        assert!(held.requires_vio() && held.pose_degraded && !held.vio_ready());
        assert!(
            AutonomyBrief::parse(r#"{"status":{"safety":"emergency_stop"}}"#).emergency_stopped()
        );
        assert!(!AutonomyBrief::parse(r#"{"status":{"safety":"hold"}}"#).emergency_stopped());
        assert_eq!(
            AutonomyBrief::parse(""),
            AutonomyBrief {
                pose_degraded: true,
                ..Default::default()
            }
        );
    }

    #[test]
    /// 20 Hz frames and FPV, 10 Hz status and task model.
    fn schedule() {
        let routes = (0..100).filter(|t| tick_plan(*t).route).count();
        let publishes = (0..100).filter(|t| tick_plan(*t).publish).count();
        assert_eq!((routes, publishes), (20, 10));
        assert_eq!(
            tick_plan(10),
            TickPlan {
                route: true,
                publish: true,
                fpv: true,
                task_model: true
            }
        );
    }

    #[test]
    /// L0 sends the Rust frame on route ticks only; no link means nothing.
    fn teleop_routes_the_rust_frame() {
        let gate = confirmed();
        let b = brief("teleop", 0.0);
        let out = output(SafetyState::StaleTarget, Some(FRAME));
        let d = |tick, f| {
            gate.decide(
                tick,
                0.0,
                DriveSource::Phone,
                f,
                b.clone(),
                out.clone(),
                false,
            )
            .unwrap()
        };
        assert_eq!(d(5, facts()), drive());
        assert_eq!(d(6, facts()), MotorCommand::Idle);
        assert_eq!(
            d(
                5,
                LinkFacts {
                    ble_active: false,
                    ..facts()
                }
            ),
            MotorCommand::Idle
        );
        assert_eq!(
            d(
                5,
                LinkFacts {
                    ble_ready: false,
                    ..facts()
                }
            ),
            MotorCommand::Idle
        );
        // L0 keeps driving through tracking loss.
        assert_eq!(
            d(
                5,
                LinkFacts {
                    phone_tracked: false,
                    ..facts()
                }
            ),
            drive()
        );
    }

    #[test]
    /// The mount hold stops once, then stays idle until confirmed; local debug skips it.
    fn mount_hold() {
        let gate = MotorGate::new();
        let b = brief("teleop", 0.0);
        let out = output(SafetyState::Active, Some(FRAME));
        let d = |src, f| {
            gate.decide(0, 1.0, src, f, b.clone(), out.clone(), false)
                .unwrap()
        };
        assert_eq!(
            d(DriveSource::Phone, facts()),
            MotorCommand::Stop {
                reason: MOUNT_HOLD_REASON.into()
            }
        );
        assert_eq!(d(DriveSource::Phone, facts()), MotorCommand::Idle);
        let local = LinkFacts {
            dashboard_connected: false,
            ..facts()
        };
        assert_eq!(d(DriveSource::BluetoothManual, local), drive());
        assert_eq!(d(DriveSource::BluetoothManual, facts()), MotorCommand::Idle);
        gate.set_mount_confirmed(true).unwrap();
        assert_eq!(d(DriveSource::Phone, facts()), drive());
        assert_eq!(
            gate.stop_reason().unwrap().as_deref(),
            Some(MOUNT_HOLD_REASON)
        );
    }

    #[test]
    /// The e-stop latch blocks frames through reset until it is cleared.
    fn estop_latch_survives_reset() {
        let gate = confirmed();
        let b = brief("teleop", 0.0);
        let out = output(SafetyState::Active, Some(FRAME));
        gate.emergency_stop(2.0).unwrap();
        gate.reset().unwrap();
        let d = || {
            gate.decide(
                0,
                3.0,
                DriveSource::Phone,
                facts(),
                b.clone(),
                out.clone(),
                true,
            )
            .unwrap()
        };
        assert_eq!(d(), MotorCommand::Idle);
        assert!(gate.emergency_stopped().unwrap());
        gate.clear_emergency_stop().unwrap();
        assert_eq!(d(), drive());
        assert_eq!(
            gate.stop_reason().unwrap().as_deref(),
            Some("Emergency stop")
        );
        gate.clear_stop_reason().unwrap();
        assert_eq!(gate.stop_reason().unwrap(), None);
    }

    #[test]
    /// Closed loop needs phone tracking, fresh VIO, an active loop and differential routes.
    fn closed_loop_gates() {
        let gate = confirmed();
        let active = output(SafetyState::Active, Some(FRAME));
        let d = |src, f, b: AutonomyBrief, o: &ControlOutput, vio| {
            gate.decide(0, 0.0, src, f, b, o.clone(), vio).unwrap()
        };
        assert_eq!(
            d(
                DriveSource::Phone,
                facts(),
                brief("waypoint", 0.1),
                &active,
                true
            ),
            drive()
        );
        let lost = LinkFacts {
            phone_tracked: false,
            ..facts()
        };
        assert!(
            matches!(d(DriveSource::Phone, lost, brief("waypoint", 0.1), &active, true), MotorCommand::Stop { reason } if reason.contains("tracking lost"))
        );
        assert!(
            matches!(d(DriveSource::Phone, facts(), brief("waypoint", 0.1), &active, false), MotorCommand::Stop { reason } if reason.contains("differential"))
        );
        assert!(matches!(
            d(
                DriveSource::Phone,
                facts(),
                brief("waypoint", 0.9),
                &active,
                true
            ),
            MotorCommand::Stop { .. }
        ));
        let stale = output(SafetyState::StaleSensors, Some(FRAME));
        assert!(
            matches!(d(DriveSource::Phone, facts(), brief("explore", 0.1), &stale, true), MotorCommand::Stop { reason } if reason.contains("StaleSensors"))
        );
        // The simulated plant never drives a real rover closed loop.
        assert!(matches!(
            d(
                DriveSource::Simulation,
                facts(),
                brief("waypoint", 0.1),
                &active,
                true
            ),
            MotorCommand::Stop { .. }
        ));
        // Feedback actuation on an incompatible layout sends nothing.
        let fb = LinkFacts {
            feedback: true,
            feedback_compatible: false,
            ..facts()
        };
        assert_eq!(
            d(DriveSource::Phone, fb, brief("teleop", 0.1), &active, true),
            MotorCommand::Idle
        );
        assert_eq!(
            d(
                DriveSource::Phone,
                fb,
                brief("waypoint", 0.1),
                &active,
                true
            ),
            MotorCommand::Idle
        );
    }

    #[test]
    /// A missing Rust frame stops with a routing reason instead of sending stale values.
    fn missing_frame_stops() {
        let gate = confirmed();
        let out = output(SafetyState::Active, None);
        let cmd = gate
            .decide(
                0,
                0.0,
                DriveSource::Phone,
                facts(),
                brief("teleop", 0.0),
                out,
                false,
            )
            .unwrap();
        assert!(
            matches!(cmd, MotorCommand::Stop { reason } if reason.starts_with("Actuator routing failed"))
        );
    }

    #[test]
    /// The typed brief comes from a real controller tick.
    fn controller_brief_is_typed() {
        let controller = MobileController::new(crate::default_control_settings()).unwrap();
        controller.step(0.01).unwrap();
        let b = controller.autonomy_brief_typed();
        assert!(!b.requested_level.is_empty());
        assert!(!controller.layout_supports_vio_control());
    }
}
