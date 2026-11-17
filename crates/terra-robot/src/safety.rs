//! The last word before the motors: one ordered list of checks between a
//! level's proposal and the actuator frame. These used to live in Swift
//! (`routeHardware`); the app now gets the answer from [`crate::Robot::update`].
use crate::sensors::{DriveSource, SensorData};
use crate::{ActuationMode, Plan};
use terra_autonomy::Safety;
use terra_types::{MotorOutput, StopReason};

/// Shown while motors are held for an unconfirmed mount.
pub const MOUNT_HOLD_REASON: &str =
    "Phone mount not confirmed · confirm the phone is mounted before motors move";

/// What the motor link does this tick.
#[derive(Clone, Debug, PartialEq)]
pub enum Verdict {
    /// Send nothing; the rover's own 250 ms lease keeps it safe.
    Idle,
    /// The wheels may take these efforts.
    Drive,
    /// Stop output now, for this reason.
    Stop(String),
}

/// Checks that need memory across ticks: the mount hold is reported once
/// per hold, and the first stop cause is kept for the phone UI.
#[derive(Default)]
pub struct SafetyGate {
    mount_hold_reported: bool,
    first_stop: Option<String>,
}

/// Facts about this tick the gate reads besides the sensors and the plan.
pub struct GateInput<'a> {
    pub sensors: &'a SensorData,
    pub plan: &'a Plan,
    /// Wheel efforts the proposal produced.
    pub motor: &'a MotorOutput,
    /// Age of the input behind the proposal, seconds.
    pub input_age: f64,
    /// Fresh tracked pose and a ready velocity estimate.
    pub pose_ready: bool,
    /// The actuator layout has differential drive routes.
    pub layout_supports_vio: bool,
    /// An ARGOS link is attached and working.
    pub fleet_connected: bool,
}

/// Longest an operator input may drive; the same limit the arbiter applies.
pub const LEASE: f64 = terra_types::timing::OPERATOR_COMMAND_FRESHNESS_SECONDS;

impl SafetyGate {
    /// First match wins: no link, e-stop or tilt, stale input, lost
    /// tracking, unusable layout, unhealthy feedback, then the link checks.
    pub fn check(&mut self, g: &GateInput) -> Verdict {
        let link = g.sensors.motors;
        if !link.connected {
            return Verdict::Idle;
        }
        let status = &g.plan.output.status;
        let level = status.requested_level;
        let manual = level.is_manual();
        let feedback = g.plan.mode == ActuationMode::VelocityFeedback;
        // The phone's debug joystick needs no mount; a fleet session does.
        let local_debug = g.sensors.source == DriveSource::BluetoothManual && !g.fleet_connected;
        if status.safety == Safety::EmergencyStop {
            return self.stop(format!("Emergency stop: {}", status.reason));
        }
        if !(0.0..=LEASE).contains(&g.input_age) {
            return self.stop("Operator input expired".into());
        }
        if g.sensors.source != DriveSource::BluetoothManual {
            if g.sensors.source == DriveSource::Phone
                && !link.tracked
                && (feedback || level.uses_vio())
                && !manual
            {
                return self.stop("Camera tracking lost in tracking-dependent mode".into());
            }
            if !level.operator_driven() && !g.layout_supports_vio {
                return self.stop("L3 requires differential motor routes".into());
            }
            if (feedback || !level.operator_driven()) && !manual {
                let healthy = g.sensors.source == DriveSource::Phone && g.pose_ready;
                if !healthy || g.motor.stop_reason != StopReason::None {
                    let reason = if status.reason.is_empty() {
                        "unavailable"
                    } else {
                        status.reason.as_str()
                    };
                    return self.stop(format!(
                        "Autonomy: {reason}; feedback controller safety={:?}",
                        g.motor.stop_reason
                    ));
                }
            } else if !manual && !g.fleet_connected {
                // Direct effort at a non-manual level only with a fleet attached.
                return Verdict::Idle;
            }
        }
        if !link.ready || (feedback && !link.feedback_compatible) {
            return Verdict::Idle;
        }
        if !link.mount_confirmed && !local_debug {
            if self.mount_hold_reported {
                return Verdict::Idle;
            }
            self.mount_hold_reported = true;
            return self.stop(MOUNT_HOLD_REASON.into());
        }
        self.mount_hold_reported = false;
        Verdict::Drive
    }

    pub fn stop(&mut self, reason: String) -> Verdict {
        if self.first_stop.is_none() && !reason.is_empty() {
            self.first_stop = Some(reason.clone());
        }
        Verdict::Stop(reason)
    }

    /// First stop cause since [`Self::clear_stop_reason`].
    pub fn stop_reason(&self) -> Option<&str> {
        self.first_stop.as_deref()
    }

    pub fn clear_stop_reason(&mut self) {
        self.first_stop = None;
    }

    /// Controller restarted: report the next mount hold again.
    pub fn reset(&mut self) {
        self.mount_hold_reported = false;
    }
}
