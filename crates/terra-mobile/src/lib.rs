//! UniFFI bindings for the phone apps. All control lives in `terra-robot`:
//! the apps fill a [`PhoneSensors`] each control tick, call
//! [`MobileController::update`] and send the [`MotorTick`] it returns to the
//! motor board. This crate only converts records and forwards calls.

use std::sync::{Arc, Mutex};
use terra_control::ControllerConfig;
use terra_robot::Robot;
use terra_state::EstimatorConfig;
use terra_types::*;
uniffi::setup_scaffolding!();
mod actuators;
pub use actuators::*;
mod autonomy;
mod diagnostics;
pub use autonomy::{FpvEffect, WebrtcSignal};
pub use diagnostics::ObservationAttachment;
pub mod outdoor;
mod outdoor_bridge;
pub use outdoor::{CourseFix, GpsFix, HeadingFix};
mod search;
pub use search::{
    MobileObjectDetection, MobileSearchRequest, MobileSearchSnapshot, MobileTargetObservation,
};
mod mcp;
pub use mcp::{
    MobileAgentPrompt, agent_parse_reply, agent_system_prompt_text, agent_tool_specs_json,
};
mod task;
pub use task::MobileTaskQuery;
mod geo;
mod mapping;
mod simulation;
pub use geo::*;
pub use mapping::*;
pub use simulation::*;

/// Shared presentation and router-loss defaults. Local sensor limits are independent.
#[uniffi::export]
pub fn default_connectivity_policy_settings() -> String {
    serde_json::to_string(&terra_robot::connectivity::ConnectivityPolicySettings::default())
        .expect("finite policy defaults")
}

/// Built-in rover profiles as a JSON array; each entry is accepted by
/// [`MobileController::set_rover_profile`]. The first entry is Terra.
#[uniffi::export]
pub fn rover_profile_presets() -> String {
    serde_json::to_string(&terra_robot::rover_profile::RoverProfile::presets())
        .expect("finite rover profiles")
}

#[derive(Clone, Debug, uniffi::Record)]
/// Sensor freshness, PI gains, and wheel effort limits used at construction.
/// All values must be finite. Gains lie in 0..=100; timeouts and limits must
/// be positive. See [`MobileController::new`] for configuration validation.
pub struct ControlSettings {
    /// Forward proportional gain, effort per m/s of error.
    pub linear_kp: f64,
    /// Forward integral gain, effort per metre of accumulated error.
    pub linear_ki: f64,
    /// Yaw proportional gain, effort per rad/s of error.
    pub yaw_kp: f64,
    /// Yaw integral gain, effort per radian of accumulated error.
    pub yaw_ki: f64,
    /// Forward feedforward gain, effort per m/s.
    pub linear_feedforward: f64,
    /// Yaw feedforward gain, effort per rad/s.
    pub yaw_feedforward: f64,
    /// Maximum commanded forward speed magnitude, m/s (at most 20).
    pub max_forward: f64,
    /// Maximum commanded yaw-rate magnitude, rad/s (at most 20).
    pub max_yaw_rate: f64,
    /// Maximum absolute normalized wheel effort, in (0, 1].
    pub max_effort: f64,
    /// Maximum IMU age, seconds.
    pub imu_timeout: f64,
    /// Maximum VIO age, seconds.
    pub vio_timeout: f64,
    /// Maximum target age, seconds.
    pub target_timeout: f64,
    /// Maximum allowed control tick interval, seconds (at most 1).
    pub max_step: f64,
    /// Back-calculation gain for effort saturation, per second.
    pub anti_windup: f64,
    /// Open-loop wheel effort for a managed in-place spin, in [0, 1]; 0 keeps feedback.
    pub spin_effort: f64,
}
impl ControlSettings {
    fn controller(&self) -> ControllerConfig {
        // Until a rover profile is selected (before the phone picks one),
        // back up with Terra's reverse compensation.
        terra_robot::rover_profile::RoverProfile::terra().reverse_compensation(ControllerConfig {
            linear_kp: self.linear_kp,
            linear_ki: self.linear_ki,
            yaw_kp: self.yaw_kp,
            yaw_ki: self.yaw_ki,
            linear_feedforward: self.linear_feedforward,
            yaw_feedforward: self.yaw_feedforward,
            max_forward: self.max_forward,
            max_yaw_rate: self.max_yaw_rate,
            max_effort: self.max_effort,
            target_timeout: self.target_timeout,
            max_step: self.max_step,
            anti_windup: self.anti_windup,
            spin_effort: self.spin_effort,
            ..ControllerConfig::default()
        })
    }
    fn estimator(&self) -> EstimatorConfig {
        EstimatorConfig {
            imu_timeout: self.imu_timeout,
            vio_timeout: self.vio_timeout,
        }
    }
}
#[uniffi::export]
/// Return the portable controller and estimator defaults as a UniFFI record.
///
/// ```
/// let controller = terra_mobile::MobileController::new(
///     terra_mobile::default_control_settings(),
/// )?;
/// # Ok::<(), terra_mobile::ControllerError>(())
/// ```
pub fn default_control_settings() -> ControlSettings {
    let c = ControllerConfig::default();
    let e = EstimatorConfig::default();
    ControlSettings {
        linear_kp: c.linear_kp,
        linear_ki: c.linear_ki,
        yaw_kp: c.yaw_kp,
        yaw_ki: c.yaw_ki,
        linear_feedforward: c.linear_feedforward,
        yaw_feedforward: c.yaw_feedforward,
        max_forward: c.max_forward,
        max_yaw_rate: c.max_yaw_rate,
        max_effort: c.max_effort,
        imu_timeout: e.imu_timeout,
        vio_timeout: e.vio_timeout,
        target_timeout: c.target_timeout,
        max_step: c.max_step,
        anti_windup: c.anti_windup,
        spin_effort: c.spin_effort,
    }
}
#[derive(Clone, Debug, uniffi::Record)]
/// Timestamped body-frame inertial sample: +X forward, +Y left, +Z up.
/// Use the same monotonic seconds clock as VIO, targets, and control ticks.
pub struct ImuReading {
    /// Sample or intent time in seconds on the shared monotonic clock.
    pub timestamp: f64,
    /// Body forward acceleration, m/s².
    pub acceleration_forward: f64,
    /// Body left acceleration, m/s².
    pub acceleration_left: f64,
    /// Body upward acceleration, m/s².
    pub acceleration_up: f64,
    /// Body roll angular velocity about +X, rad/s.
    pub gyro_roll: f64,
    /// Body pitch angular velocity about +Y, rad/s.
    pub gyro_pitch: f64,
    /// Body yaw angular velocity about +Z, rad/s.
    pub gyro_yaw: f64,
}
#[derive(Clone, Debug, uniffi::Record)]
/// Tracked body pose and world-frame velocity in a right-handed Z-up frame.
/// Quaternion components rotate body vectors into the world. Use the shared
/// monotonic seconds clock; report tracking loss with `tracked = false`.
pub struct VioReading {
    /// Sample or intent time in seconds on the shared monotonic clock.
    pub timestamp: f64,
    /// World +X position, metres.
    pub position_x: f64,
    /// World +Y position, metres.
    pub position_y: f64,
    /// World +Z position, metres.
    pub position_z: f64,
    /// Body-to-world quaternion X component, dimensionless.
    pub quaternion_x: f64,
    /// Body-to-world quaternion Y component, dimensionless.
    pub quaternion_y: f64,
    /// Body-to-world quaternion Z component, dimensionless.
    pub quaternion_z: f64,
    /// Body-to-world quaternion scalar component, dimensionless.
    pub quaternion_w: f64,
    /// World +X velocity, m/s.
    pub velocity_x: f64,
    /// World +Y velocity, m/s.
    pub velocity_y: f64,
    /// World +Z velocity, m/s.
    pub velocity_z: f64,
    /// Whether visual tracking is currently valid.
    pub tracked: bool,
}
impl From<ImuReading> for ImuSample {
    fn from(s: ImuReading) -> Self {
        ImuSample {
            timestamp: s.timestamp,
            acceleration: Vector3 {
                x: s.acceleration_forward,
                y: s.acceleration_left,
                z: s.acceleration_up,
            },
            angular_velocity: Vector3 {
                x: s.gyro_roll,
                y: s.gyro_pitch,
                z: s.gyro_yaw,
            },
        }
    }
}
impl From<VioReading> for VioSample {
    fn from(s: VioReading) -> Self {
        VioSample {
            timestamp: s.timestamp,
            position: Vector3 {
                x: s.position_x,
                y: s.position_y,
                z: s.position_z,
            },
            orientation: Quaternion {
                x: s.quaternion_x,
                y: s.quaternion_y,
                z: s.quaternion_z,
                w: s.quaternion_w,
            },
            velocity: Vector3 {
                x: s.velocity_x,
                y: s.velocity_y,
                z: s.velocity_z,
            },
            tracked: s.tracked,
        }
    }
}
#[derive(Clone, Copy, Debug, uniffi::Record)]
/// The phone's own joystick: `forward` and `turn` in `[-1, 1]`, positive turn
/// is left. Full forward asks for 0.5 m/s.
pub struct StickInput {
    pub forward: f64,
    pub turn: f64,
    pub timestamp: f64,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, uniffi::Record)]
/// The phone's link to the motor board, and the facts only the phone knows.
pub struct MotorLink {
    /// A rover Bluetooth link exists.
    pub connected: bool,
    /// The link is armed and accepts drive frames.
    pub ready: bool,
    /// The person confirmed the phone sits in its mount.
    pub mount_confirmed: bool,
    /// AR tracking is normal on the latest camera frame.
    pub tracked: bool,
    /// The rover reports a layout usable for velocity-feedback actuation.
    pub feedback_compatible: bool,
}
/// What the phone is driving from.
#[derive(Clone, Copy, Debug, PartialEq, Eq, uniffi::Enum)]
pub enum DriveSource {
    /// Phone sensors (ARKit / ARCore + IMU).
    Phone,
    /// The local simulated plant.
    Simulation,
    /// Bluetooth effort with no phone sensors: the debug joystick.
    BluetoothManual,
}
#[derive(Clone, Debug, uniffi::Record)]
/// Everything the phone knows this tick: the latest sample of each sensor,
/// `None` until the first one. A sample already seen is not applied twice.
pub struct PhoneSensors {
    /// Control-clock time of this tick, monotonic seconds.
    pub now: f64,
    pub imu: Option<ImuReading>,
    pub vio: Option<VioReading>,
    pub gps: Option<GpsFix>,
    pub heading: Option<HeadingFix>,
    pub course: Option<CourseFix>,
    pub stick: Option<StickInput>,
    pub motors: MotorLink,
    pub source: DriveSource,
}
impl From<PhoneSensors> for terra_robot::SensorData {
    fn from(s: PhoneSensors) -> Self {
        Self {
            now: s.now,
            imu: s.imu.map(Into::into),
            vio: s.vio.map(Into::into),
            gps: s.gps.map(Into::into),
            heading: s.heading.map(Into::into),
            course: s.course.map(Into::into),
            stick: s.stick.map(|x| terra_robot::Stick {
                forward: x.forward,
                turn: x.turn,
                t: x.timestamp,
            }),
            motors: terra_robot::MotorLink {
                connected: s.motors.connected,
                ready: s.motors.ready,
                mount_confirmed: s.motors.mount_confirmed,
                tracked: s.motors.tracked,
                feedback_compatible: s.motors.feedback_compatible,
            },
            source: match s.source {
                DriveSource::Phone => terra_robot::DriveSource::Phone,
                DriveSource::Simulation => terra_robot::DriveSource::Simulation,
                DriveSource::BluetoothManual => terra_robot::DriveSource::BluetoothManual,
            },
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, uniffi::Enum)]
/// Why the velocity loop returns stopped wheel efforts, if it does.
pub enum SafetyState {
    Active,
    SensorNotReady,
    TrackingLost,
    StaleSensors,
    StaleTarget,
    InvalidTime,
}
#[derive(Clone, Debug, PartialEq, uniffi::Enum)]
/// What to do with the motor link this tick.
pub enum MotorAction {
    /// Send nothing; the rover's own lease keeps it safe.
    Idle,
    /// Write this actuator frame (`[{"id","value"}]` JSON).
    Drive { frame_json: String },
    /// Stop output now. `reason` is for the phone UI.
    Stop { reason: String },
}
#[derive(Clone, Copy, Debug, PartialEq, uniffi::Record)]
/// One servo target from Rust control.
pub struct GateServo {
    /// Actuator id in the layout.
    pub id: u8,
    /// Normalized position, already clipped to the servo's limits.
    pub position: f64,
}
#[derive(Clone, Debug, uniffi::Record)]
/// One control tick's answer and the numbers behind it for the UI.
pub struct MotorTick {
    pub action: MotorAction,
    pub left_effort: f64,
    pub right_effort: f64,
    /// Estimated body forward speed, m/s.
    pub estimated_forward: f64,
    /// Estimated body yaw rate, rad/s.
    pub estimated_yaw_rate: f64,
    pub target_forward: f64,
    pub target_yaw_rate: f64,
    pub safety: SafetyState,
    /// Dump-gate servo position chosen by the bucket dock, already in the frame.
    pub gate_servo: Option<GateServo>,
}
impl From<terra_robot::MotorCommand> for MotorTick {
    fn from(c: terra_robot::MotorCommand) -> Self {
        let m = c.motor;
        Self {
            action: match c.action {
                terra_robot::MotorAction::Idle => MotorAction::Idle,
                terra_robot::MotorAction::Drive { frame_json } => MotorAction::Drive { frame_json },
                terra_robot::MotorAction::Stop { reason } => MotorAction::Stop { reason },
            },
            left_effort: m.left,
            right_effort: m.right,
            estimated_forward: m.estimated_forward,
            estimated_yaw_rate: m.estimated_yaw_rate,
            target_forward: m.target_forward,
            target_yaw_rate: m.target_yaw_rate,
            safety: match m.stop_reason {
                StopReason::None => SafetyState::Active,
                StopReason::SensorNotReady => SafetyState::SensorNotReady,
                StopReason::TrackingLost => SafetyState::TrackingLost,
                StopReason::StaleSensors => SafetyState::StaleSensors,
                StopReason::StaleTarget => SafetyState::StaleTarget,
                StopReason::InvalidTime => SafetyState::InvalidTime,
            },
            gate_servo: c.gate_servo.map(|(id, position)| GateServo { id, position }),
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, uniffi::Record)]
/// The dump gate as the person set it on the phone.
pub struct DumpGate {
    /// Actuator id of the positional servo in the layout.
    pub servo_id: u8,
    /// Normalized positions, -1..=1.
    pub open_position: f64,
    pub closed_position: f64,
}
#[derive(Debug, thiserror::Error, uniffi::Error)]
/// Failure reported through UniFFI by validation or shared-state access.
pub enum ControllerError {
    #[error("{message}")]
    /// Input or configuration failed validation; `message` describes the rejection.
    InvalidInput {
        /// Human-readable diagnostic from the rejected operation.
        message: String,
    },
    #[error("controller lock was poisoned")]
    /// Shared controller state could not be locked because its mutex was poisoned.
    Internal,
}
impl From<InputError> for ControllerError {
    fn from(error: InputError) -> Self {
        Self::InvalidInput {
            message: error.to_string(),
        }
    }
}
impl From<terra_robot::RobotError> for ControllerError {
    fn from(e: terra_robot::RobotError) -> Self {
        Self::InvalidInput {
            message: e.to_string(),
        }
    }
}
pub(crate) fn invalid(message: impl Into<String>) -> ControllerError {
    ControllerError::InvalidInput {
        message: message.into(),
    }
}
#[derive(uniffi::Object)]
/// The robot behind a mutex, plus the video-signaling and MCP state that
/// never touch motion.
pub struct MobileController {
    robot: Mutex<Robot>,
    diagnostics: Arc<terra_robot::diagnostics::Diagnostics>,
    /// Video signaling only. It cannot arm the rover or change the velocity target.
    fpv: Mutex<fpv_signaling::SignalingSession>,
    /// MCP sessions, the camera frame for `look`, and the on-phone agent's conversation.
    mcp: Mutex<mcp::McpState>,
}
impl MobileController {
    pub(crate) fn robot(&self) -> Result<std::sync::MutexGuard<'_, Robot>, ControllerError> {
        self.robot.lock().map_err(|_| ControllerError::Internal)
    }
}
#[uniffi::export]
impl MobileController {
    #[uniffi::constructor]
    /// An idle controller with validated estimator and control settings.
    /// Opens no network or hardware connection.
    pub fn new(settings: ControlSettings) -> Result<Arc<Self>, ControllerError> {
        let robot = Robot::new(
            terra_robot::CoreConfig {
                estimator: settings.estimator(),
                controller: settings.controller(),
                ..Default::default()
            },
            format!(
                "phone-{}",
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos()
            ),
        )?;
        Ok(Arc::new(Self {
            diagnostics: robot.diagnostics(),
            robot: Mutex::new(robot),
            fpv: Mutex::new(fpv_signaling::SignalingSession::phone()),
            mcp: Default::default(),
        }))
    }
    /// One control tick: the only call that moves the rover. Send the
    /// returned action to the motor board.
    pub fn update(&self, sensors: PhoneSensors) -> Result<MotorTick, ControllerError> {
        Ok(self.robot()?.update(&sensors.into())?.into())
    }
    /// Velocity feedback (true) or direct effort for operator levels.
    pub fn set_actuation_mode(&self, feedback: bool) -> Result<(), ControllerError> {
        Ok(self.robot()?.set_actuation_mode(feedback)?)
    }
    /// Replace the rover drive profiles that turn efforts into frames; an
    /// empty string clears them.
    pub fn set_actuator_layout(&self, profiles_json: String) -> Result<(), ControllerError> {
        Ok(self.robot()?.set_actuator_layout(&profiles_json)?)
    }
    /// Set a normalized servo target, clipped to that servo's limits.
    pub fn set_servo_target(&self, id: u8, value: f64) -> Result<(), ControllerError> {
        Ok(self.robot()?.set_servo_target(id, value)?)
    }
    /// Whether the current actuator layout has differential drive routes (L2–L5 need them).
    pub fn layout_supports_vio_control(&self) -> bool {
        self.robot()
            .is_ok_and(|r| r.layout_supports_vio_control())
    }
    /// First motor stop cause since [`Self::clear_stop_reason`].
    pub fn stop_reason(&self) -> Option<String> {
        self.robot().ok()?.stop_reason().map(str::to_owned)
    }
    pub fn clear_stop_reason(&self) {
        if let Ok(mut r) = self.robot() {
            r.clear_stop_reason();
        }
    }
    /// Dump-gate servo and its positions as the person set them; `None`
    /// returns to the rover profile's.
    pub fn set_dump_gate(&self, gate: Option<DumpGate>) -> Result<(), ControllerError> {
        Ok(self
            .robot()?
            .core_mut()
            .set_dock_gate(gate.map(|g| (g.servo_id, g.open_position, g.closed_position)))?)
    }
    /// Dump outlet ahead of the rover centre, metres; `None` returns to the profile's.
    pub fn set_dock_outlet_ahead(&self, metres: Option<f64>) -> Result<(), ControllerError> {
        Ok(self.robot()?.core_mut().set_dock_outlet(metres)?)
    }
    /// Hold the dump gate open or closed to try it, or end the test (`None`).
    pub fn test_dump_gate(&self, open: Option<bool>) -> Result<(), ControllerError> {
        Ok(self.robot()?.core_mut().test_dock_gate(open)?)
    }
    /// Drive and plan as the rover in `profile_json`, one entry of
    /// [`rover_profile_presets`] or an edited copy. Local and trusted.
    pub fn set_rover_profile(&self, profile_json: String) -> Result<(), ControllerError> {
        let profile = serde_json::from_str(&profile_json)
            .map_err(|e| invalid(format!("rover profile: {e}")))?;
        Ok(self.robot()?.core_mut().apply_rover_profile(profile)?)
    }
    /// Reset estimator, velocity loop and autonomy; detach the fleet link and
    /// close the recording.
    pub fn reset(&self) -> Result<(), ControllerError> {
        Ok(self.robot()?.reset()?)
    }
}

#[derive(Clone, Debug, uniffi::Record)]
/// Final velocities and absolute errors from the deterministic motor benchmark.
pub struct BenchmarkReport {
    /// Final simulated forward speed, m/s.
    pub final_forward: f64,
    /// Final simulated yaw rate, rad/s.
    pub final_yaw_rate: f64,
    /// Absolute final error relative to 0.8 m/s, in m/s.
    pub forward_error: f64,
    /// Absolute final error relative to 0.3 rad/s, in rad/s.
    pub yaw_error: f64,
    /// True when both final absolute errors are below 0.05 in their respective units.
    pub passed: bool,
}
/// Deterministic 10 s motor plant experiment using the same facade as iOS.
#[uniffi::export]
pub fn run_velocity_benchmark() -> Result<BenchmarkReport, ControllerError> {
    let mut robot = Robot::new(
        terra_robot::CoreConfig {
            estimator: default_control_settings().estimator(),
            controller: default_control_settings().controller(),
            ..Default::default()
        },
        "benchmark".into(),
    )?;
    let (mut v, mut w, mut yaw, mut ax) = (0.0, 0.0, 0.0, 0.0);
    let mut sensors = terra_robot::SensorData {
        source: terra_robot::DriveSource::Simulation,
        ..Default::default()
    };
    for tick in 0..1000 {
        let t = tick as f64 * 0.01;
        sensors.now = t;
        sensors.imu = Some(ImuSample {
            timestamp: t,
            acceleration: Vector3 {
                x: ax,
                y: v * w,
                z: 0.0,
            },
            angular_velocity: Vector3 { x: 0.0, y: 0.0, z: w },
        });
        if tick % 5 == 0 {
            sensors.vio = Some(VioSample {
                timestamp: t,
                position: Vector3::default(),
                orientation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: (yaw * 0.5_f64).sin(),
                    w: (yaw * 0.5_f64).cos(),
                },
                velocity: Vector3 {
                    x: v * yaw.cos(),
                    y: v * yaw.sin(),
                    z: 0.0,
                },
                tracked: true,
            });
        }
        robot.core_mut().ingest(terra_robot::Event::LocalVelocity {
            target: VelocityTarget {
                timestamp: t,
                forward: if tick < 300 { 2.0 } else { 0.8 },
                yaw_rate: 0.3,
            },
            received_at: t,
        })?;
        let output = robot.update(&sensors)?.motor;
        ax = 3.0 * (output.left + output.right) * 0.5 - v - 0.15;
        v += ax * 0.01;
        w += (5.0 * (output.right - output.left) * 0.5 - w) * 0.01;
        yaw += w * 0.01;
    }
    let forward_error = (v - 0.8).abs();
    let yaw_error = (w - 0.3).abs();
    Ok(BenchmarkReport {
        final_forward: v,
        final_yaw_rate: w,
        forward_error,
        yaw_error,
        passed: forward_error < 0.05 && yaw_error < 0.05,
    })
}
/// Deterministic contract and regression scenarios for this module.
#[cfg(test)]
mod tests {
    use super::*;
    pub(crate) fn sensors(now: f64) -> PhoneSensors {
        PhoneSensors {
            now,
            imu: None,
            vio: None,
            gps: None,
            heading: None,
            course: None,
            stick: None,
            motors: MotorLink::default(),
            source: DriveSource::Phone,
        }
    }
    #[test]
    /// Runs the deterministic ten-second phone sensor/control/motor loop and checks its final-error acceptance criterion.
    fn mobile_facade_runs_the_same_sensor_and_motor_loop() {
        let report = run_velocity_benchmark().unwrap();
        assert!(report.passed, "{report:?}");
    }
    #[test]
    /// Rejects a normalized effort limit above one when constructing the phone facade.
    fn validates_settings_before_crossing_the_ffi_boundary() {
        let mut settings = default_control_settings();
        settings.max_effort = 1.5;
        assert!(MobileController::new(settings).is_err());
    }
    /// The published status and brief keep their shape: the apps and
    /// dashboards read these fields (dashboard_reconnect checks the brief
    /// matches the status).
    #[test]
    fn status_and_brief_keep_their_published_fields() {
        let controller = MobileController::new(default_control_settings()).unwrap();
        assert_eq!(controller.autonomy_status(), "");
        assert_eq!(controller.autonomy_brief(), "");
        controller.update(sensors(0.0)).unwrap();
        controller.update(sensors(0.01)).unwrap();
        let status: serde_json::Value =
            serde_json::from_str(&controller.autonomy_status()).unwrap();
        let keys =
            |v: &serde_json::Value| v.as_object().unwrap().keys().cloned().collect::<Vec<_>>();
        assert_eq!(
            keys(&status),
            [
                "command_origin",
                "connectivity",
                "exploration",
                "goal",
                "navigation",
                "outdoor",
                "pending_reports",
                "pose_quality",
                "proposal",
                "runtime",
                "search",
                "status",
                "task",
                "twist"
            ]
        );
        assert_eq!(
            keys(&status["connectivity"]),
            ["accepted_onboard_mission", "policy", "revision"]
        );
        assert_eq!(
            keys(&status["runtime"]),
            [
                "configuration_identity",
                "configured_mode",
                "mode",
                "schema_version",
                "source_revision",
                "tilt"
            ]
        );
        assert_eq!(
            keys(&status["pose_quality"]),
            [
                "degraded",
                "imu_age_s",
                "pose_age_s",
                "reason",
                "rover_id",
                "run_id",
                "schema_version",
                "sequence"
            ]
        );
        assert_eq!(
            status["pose_quality"]["sequence"], 2,
            "one sequence step per tick"
        );
        let brief: serde_json::Value = serde_json::from_str(&controller.autonomy_brief()).unwrap();
        assert_eq!(keys(&brief), ["goal", "pose_quality", "status"]);
        assert_eq!(
            keys(&brief["status"]),
            [
                "effective_level",
                "reason",
                "requested_level",
                "run_id",
                "safety"
            ]
        );
    }
    #[test]
    /// A preset from `rover_profile_presets` sets the planner footprint and is
    /// reported in navigation status; a malformed profile changes nothing.
    fn rover_profile_sets_planner_footprint() {
        let controller = MobileController::new(default_control_settings()).unwrap();
        let presets: Vec<serde_json::Value> =
            serde_json::from_str(&rover_profile_presets()).unwrap();
        let mini = presets.iter().find(|p| p["name"] == "terra_mini").unwrap();
        assert!(controller.set_rover_profile("{}".into()).is_err());
        controller.set_rover_profile(mini.to_string()).unwrap();
        controller.update(sensors(0.0)).unwrap();
        let status: serde_json::Value =
            serde_json::from_str(&controller.autonomy_status()).unwrap();
        assert_eq!(status["navigation"]["profile"], "terra_mini");
        let radius = status["navigation"]["footprint_radius"].as_f64().unwrap();
        assert!(radius < 0.3, "radius={radius}");
        let mut bad = mini.clone();
        bad["wheel_deadband"] = serde_json::json!(2.0);
        assert!(controller.set_rover_profile(bad.to_string()).is_err());
    }
}

