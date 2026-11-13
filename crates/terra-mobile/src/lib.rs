//! Thread-safe phone bindings around the portable robotics crates.
//!
//! [`MobileController`] combines sensor estimation and velocity control behind a
//! mutex. The `mapping`, `waypoint`, `autonomy`, `actuators`, `transport`, and
//! `search` modules expose their respective capabilities through UniFFI. Keep
//! phone-facing conversion here and algorithm changes in the owning Rust crate.

use std::sync::{Arc, Mutex};
use terra_control::ControllerConfig;
use terra_state::EstimatorConfig;
use terra_types::*;
uniffi::setup_scaffolding!();
mod actuators;
pub use actuators::*;
mod autonomy;
mod capture;
mod connection;
mod depth;
mod detector;
mod drive_loop;
mod inbox;
pub use autonomy::{FpvEffect, WebrtcSignal};
pub use connection::*;
pub use depth::*;
pub use detector::*;
pub use drive_loop::*;
pub use inbox::SensorQueueStats;
mod observability;
pub use observability::ObservationAttachment;
pub mod outdoor;
mod outdoor_bridge;
pub use outdoor::{CourseFix, GpsFix, HeadingFix};
mod publication;
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
mod frames;
mod gate;
mod localization;
mod mapping;
mod mount;
mod timing;
mod waypoint;
pub use frames::*;
pub use gate::*;
pub use localization::*;
pub use mapping::*;
pub use mount::*;
pub use timing::*;
pub use waypoint::*;

/// Shared presentation and router-loss defaults. Local sensor limits are independent.
#[uniffi::export]
pub fn default_connectivity_policy_settings() -> String {
    serde_json::to_string(&terra_runtime::connectivity::ConnectivityPolicySettings::default())
        .expect("finite policy defaults")
}

/// Built-in rover profiles as a JSON array; each entry is accepted by
/// [`MobileController::set_rover_profile`]. The first entry is Terra.
#[uniffi::export]
pub fn rover_profile_presets() -> String {
    serde_json::to_string(&terra_runtime::rover_profile::RoverProfile::presets())
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
        // Until a rover profile is selected (Android has no picker yet),
        // back up with Terra's reverse compensation.
        terra_runtime::rover_profile::RoverProfile::terra().reverse_compensation(ControllerConfig {
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
#[derive(Clone, Debug, uniffi::Record)]
/// Local operator velocity intent, timestamped on the sensor clock.
/// Submission does not bypass the autonomy arbiter or grant hardware authority.
pub struct TwistSetpoint {
    /// Sample or intent time in seconds on the shared monotonic clock.
    pub timestamp: f64,
    /// Requested body forward speed, m/s.
    pub forward: f64,
    /// Requested body yaw rate, rad/s.
    pub yaw_rate: f64,
}
#[derive(Clone, Copy, Debug, PartialEq, uniffi::Enum)]
/// Reason the velocity loop is active or returns stopped wheel efforts.
pub enum SafetyState {
    /// Sensor and target checks permit active control.
    Active,
    /// Required sensors have not initialized.
    SensorNotReady,
    /// Visual tracking is unavailable.
    TrackingLost,
    /// Required sensor observations exceeded their freshness limit.
    StaleSensors,
    /// Velocity intent is unavailable or expired.
    StaleTarget,
    /// Control timing is invalid.
    InvalidTime,
}
#[derive(Clone, Debug, uniffi::Record)]
/// One control tick: normalized wheel efforts, velocities, and safety reason.
pub struct ControlOutput {
    /// Normalized signed left-wheel effort.
    pub left_effort: f64,
    /// Normalized signed right-wheel effort.
    pub right_effort: f64,
    /// Estimated body forward speed, m/s.
    pub estimated_forward: f64,
    /// Estimated body yaw rate, rad/s.
    pub estimated_yaw_rate: f64,
    /// Selected and limited body forward target, m/s.
    pub target_forward: f64,
    /// Selected and limited body yaw target, rad/s.
    pub target_yaw_rate: f64,
    /// Active-loop or stopped-loop state for this tick.
    pub safety: SafetyState,
    /// Efforts routed through the layout from [`MobileController::set_actuator_layout`],
    /// as `[{"id","value"}]` JSON. `None` without a valid layout or when routing fails.
    pub actuator_frame: Option<String>,
    /// Dump-gate servo position chosen by the bucket dock this tick, when the
    /// rover profile names a gate servo. Already in `actuator_frame`; a host
    /// that routes servos itself applies it as that servo's target. `None`
    /// when the actuator layout refuses that servo.
    pub gate_servo: Option<GateServo>,
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
#[derive(Clone, Copy, Debug, PartialEq, uniffi::Record)]
/// One servo target from Rust control.
pub struct GateServo {
    /// Actuator id in the layout.
    pub id: u8,
    /// Normalized position, already clipped to the servo's limits by the layout.
    pub position: f64,
}
impl From<MotorOutput> for ControlOutput {
    fn from(value: MotorOutput) -> Self {
        Self {
            left_effort: value.left,
            right_effort: value.right,
            estimated_forward: value.estimated_forward,
            estimated_yaw_rate: value.estimated_yaw_rate,
            target_forward: value.target_forward,
            target_yaw_rate: value.target_yaw_rate,
            safety: match value.stop_reason {
                StopReason::None => SafetyState::Active,
                StopReason::SensorNotReady => SafetyState::SensorNotReady,
                StopReason::TrackingLost => SafetyState::TrackingLost,
                StopReason::StaleSensors => SafetyState::StaleSensors,
                StopReason::StaleTarget => SafetyState::StaleTarget,
                StopReason::InvalidTime => SafetyState::InvalidTime,
            },
            actuator_frame: None,
            gate_servo: None,
        }
    }
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
struct Brain {
    runtime: terra_runtime::Runtime,
    recorder: Option<terra_experiment::RunRecorder>,
    near_miss: terra_experiment::NearMiss,
    map_gate: publication::MapGate,
    cloud_gate: publication::CloudGate,
    dashboard: Option<terra_transport::ControlPlane>,
    /// Running point capture, if the dashboard asked for one.
    capture: Option<capture::Capture>,
    last_tick: f64,
    actuators: Option<actuators::ActiveLayout>,
    /// Object-map revision and body for the `objects` topic.
    objects_published: Option<(u64, String)>,
    /// Why the layout last refused the dock's gate servo target, journaled once.
    gate_rejected: Option<String>,
}
impl std::ops::Deref for Brain {
    type Target = terra_runtime::Runtime;
    fn deref(&self) -> &Self::Target {
        &self.runtime
    }
}
impl Brain {
    /// An attached, working fleet link owns velocity intent; local sticks
    /// and targets must not overwrite it.
    fn fleet_owns_intent(&self) -> bool {
        self.dashboard.as_ref().is_some_and(|d| !d.failed())
    }
    /// No recorder, or one that has not failed.
    fn recording_healthy(&self) -> bool {
        self.recorder.as_ref().is_none_or(|r| !r.failed())
    }
}
fn fleet_owns_intent() -> ControllerError {
    ControllerError::InvalidInput {
        message: "fleet connection owns velocity intent".into(),
    }
}
impl From<terra_runtime::RuntimeError> for ControllerError {
    fn from(e: terra_runtime::RuntimeError) -> Self {
        Self::InvalidInput {
            message: e.to_string(),
        }
    }
}
/// Controller state plus the sensor samples waiting to be applied to it.
///
/// Sensor pushes only touch the inbox, so they never wait for a running tick.
/// [`Self::lock`] applies every queued sample, in timestamp order, before it
/// hands out the state, so each method sees all samples pushed before the call.
struct BrainCell {
    state: Mutex<Brain>,
    inbox: inbox::SensorInbox,
}
impl BrainCell {
    fn new(brain: Brain) -> Self {
        Self {
            state: Mutex::new(brain),
            inbox: Default::default(),
        }
    }
    fn lock(&self) -> std::sync::LockResult<std::sync::MutexGuard<'_, Brain>> {
        let mut brain = self.state.lock()?;
        for sample in self.inbox.drain() {
            if let Err(error) = apply_sample(&mut brain, sample) {
                self.inbox.reject(error.to_string());
            }
        }
        // The recorder fails on its own thread; admission and the next tick
        // must already see it.
        let recording = brain.recording_healthy();
        brain.runtime.set_recording_healthy(recording);
        Ok(brain)
    }
    /// Lock without waiting and without applying queued samples. Only for reads
    /// that do not depend on sensor data.
    fn try_lock(&self) -> std::sync::TryLockResult<std::sync::MutexGuard<'_, Brain>> {
        self.state.try_lock()
    }
}
#[derive(uniffi::Object)]
/// Shared sensor estimator, autonomy arbiter, and velocity controller.
/// Methods serialize state access with a mutex. Supply timestamped sensors,
/// submit intent, and call [`Self::step`] on the same monotonic clock.
/// Returned efforts still require an independently armed hardware adapter.
pub struct MobileController {
    brain: BrainCell,
    /// Video signaling only. It cannot arm the rover or change the velocity target.
    fpv: Mutex<fpv_signaling::SignalingSession>,
    observation: observability::ObservationState,
    /// MCP sessions, the camera frame for `look`, and the on-phone agent's conversation.
    mcp: Mutex<mcp::McpState>,
}
#[uniffi::export]
impl MobileController {
    #[uniffi::constructor]
    /// Construct an idle controller with validated estimator and control settings.
    ///
    /// # Errors
    /// Returns `InvalidInput` for nonfinite or out-of-range gains, limits, or
    /// freshness settings. No network or hardware connection is opened.
    pub fn new(settings: ControlSettings) -> Result<Arc<Self>, ControllerError> {
        Ok(Arc::new(Self {
            fpv: Mutex::new(fpv_signaling::SignalingSession::phone()),
            observation: Default::default(),
            mcp: Default::default(),
            brain: BrainCell::new(Brain {
                runtime: terra_runtime::Runtime::new(
                    terra_runtime::RuntimeConfig {
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
                )?,
                recorder: None,
                near_miss: Default::default(),
                map_gate: Default::default(),
                cloud_gate: Default::default(),
                dashboard: None,
                capture: None,
                last_tick: 0.,
                actuators: None,
                objects_published: None,
                gate_rejected: None,
            }),
        }))
    }
    /// Queue one body-frame IMU reading for the next [`Self::step`]. Safe to call
    /// from any thread; it never waits for a running tick. The sample is validated
    /// when it is applied; a rejected sample is dropped and counted (see
    /// [`Self::sensor_queue_stats`]).
    ///
    /// # Errors
    /// Returns `InvalidInput` for invalid sensor values or sample ordering and
    /// `Internal` if shared state cannot be locked.
    pub fn push_imu(&self, sample: ImuReading) -> Result<(), ControllerError> {
        self.brain.inbox.push(inbox::Sample::Imu(sample))
    }
    /// Queue one VIO reading for the next [`Self::step`]; when tracked it updates the
    /// planar world pose as it is applied. Never waits for a running tick.
    /// The tracked quaternion is normalized before extracting heading.
    ///
    /// # Errors
    /// Returns `InvalidInput` for invalid readings or sample ordering and
    /// `Internal` for a poisoned mutex. Pose assignment precedes estimator validation.
    pub fn push_vio(&self, sample: VioReading) -> Result<(), ControllerError> {
        self.brain.inbox.push(inbox::Sample::Vio(sample))
    }
    /// Select the same actuation profile used by simulator adapters.
    pub fn set_actuation_mode(&self, feedback: bool) -> Result<(), ControllerError> {
        let mut b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        let at = b.last_tick;
        b.runtime.ingest(terra_runtime::RuntimeEvent::Mode {
            mode: if feedback {
                terra_runtime::ActuationMode::VelocityFeedback
            } else {
                terra_runtime::ActuationMode::DirectEffort
            },
            received_at: at,
        })?;
        Ok(())
    }
    /// Replace the rover drive profiles used to route each tick's efforts into
    /// [`ControlOutput::actuator_frame`]. Servo targets reset to their safe values.
    /// An empty string clears the layout.
    ///
    /// # Errors
    /// Returns `InvalidInput` for malformed or invalid profiles; the layout is
    /// cleared so no frame is produced from stale profiles.
    pub fn set_actuator_layout(&self, profiles_json: String) -> Result<(), ControllerError> {
        let mut b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        b.actuators = None;
        if profiles_json.is_empty() {
            return Ok(());
        }
        b.actuators = Some(actuators::ActiveLayout::parse(&profiles_json).map_err(|e| {
            ControllerError::InvalidInput {
                message: e.to_string(),
            }
        })?);
        Ok(())
    }
    /// Set a normalized servo target, clipped to that servo's limits.
    ///
    /// # Errors
    /// Returns `InvalidInput` without a layout, for an unknown or non-servo ID,
    /// or for a nonfinite value.
    pub fn set_servo_target(&self, id: u8, value: f64) -> Result<(), ControllerError> {
        let mut b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        let layout = b.actuators.as_mut().ok_or(ControllerError::InvalidInput {
            message: "no actuator layout".into(),
        })?;
        layout
            .set_servo(id, value)
            .map_err(|e| ControllerError::InvalidInput {
                message: e.to_string(),
            })
    }
    /// Dump-gate servo and its open and closed positions, set by the person on
    /// the phone. They replace the rover profile's gate and survive profile
    /// changes; `None` returns to the profile's. Local and trusted.
    ///
    /// # Errors
    /// Returns `InvalidInput` for positions outside -1..=1 or equal.
    pub fn set_dump_gate(&self, gate: Option<DumpGate>) -> Result<(), ControllerError> {
        let mut b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        b.runtime
            .set_dock_gate(gate.map(|g| (g.servo_id, g.open_position, g.closed_position)))?;
        Ok(())
    }
    /// Dump outlet ahead of the rover centre, metres. Terra's outlet sits over
    /// the phone, so the apps pass the mount's camera forward offset. `None`
    /// returns to the rover profile's value.
    ///
    /// # Errors
    /// Returns `InvalidInput` outside 0.05..=3 m.
    pub fn set_dock_outlet_ahead(&self, metres: Option<f64>) -> Result<(), ControllerError> {
        let mut b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        b.runtime.set_dock_outlet(metres)?;
        Ok(())
    }
    /// Hold the dump gate open (`Some(true)`) or closed to try its positions,
    /// or end the test (`None`). A running dock always controls the gate.
    ///
    /// # Errors
    /// Returns `InvalidInput` while a dock runs or without a gate servo.
    pub fn test_dump_gate(&self, open: Option<bool>) -> Result<(), ControllerError> {
        let mut b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        b.runtime.test_dock_gate(open)?;
        Ok(())
    }
    /// Drive and plan as the rover described by `profile_json`, one entry of
    /// [`rover_profile_presets`] or an edited copy. Sets the planner footprint,
    /// turning policy, speed limits, velocity-loop gains and wheel deadband, and
    /// restarts the velocity loop. Local and trusted; ARGOS cannot call it.
    ///
    /// # Errors
    /// Returns `InvalidInput` for malformed JSON or out-of-range values; the
    /// current profile then stays in place.
    pub fn set_rover_profile(&self, profile_json: String) -> Result<(), ControllerError> {
        let profile =
            serde_json::from_str(&profile_json).map_err(|e| ControllerError::InvalidInput {
                message: format!("rover profile: {e}"),
            })?;
        let mut b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        b.runtime.apply_rover_profile(profile)?;
        Ok(())
    }
    /// Trusted local normalized manual input; remote envelopes cannot access this ingress.
    pub fn set_manual_input(
        &self,
        forward: f64,
        turn: f64,
        timestamp: f64,
    ) -> Result<(), ControllerError> {
        let mut b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        if b.fleet_owns_intent() {
            return Err(fleet_owns_intent());
        }
        b.runtime.ingest(terra_runtime::RuntimeEvent::LocalManual {
            forward,
            turn,
            received_at: timestamp,
        })?;
        Ok(())
    }
    /// Submit local operator intent to the autonomy arbiter.
    /// `forward` is m/s and `yaw_rate` is rad/s, each limited in magnitude to 5
    /// at ingress. The arbiter may further limit or decline the request.
    ///
    /// # Errors
    /// Returns `InvalidInput` for nonfinite timestamps or velocities, or excessive
    /// velocity magnitudes; returns `Internal` for a poisoned mutex.
    pub fn set_target(&self, target: TwistSetpoint) -> Result<(), ControllerError> {
        let mut brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        if brain.fleet_owns_intent() {
            return Err(fleet_owns_intent());
        }
        brain
            .runtime
            .ingest(terra_runtime::RuntimeEvent::LocalVelocity {
                target: VelocityTarget {
                    timestamp: target.timestamp,
                    forward: target.forward,
                    yaw_rate: target.yaw_rate,
                },
                received_at: target.timestamp,
            })?;
        Ok(())
    }
    /// Run one sensor, remote-intent, autonomy, telemetry, and motor-control tick.
    /// `timestamp` uses the sensor monotonic seconds clock. Stale or unhealthy
    /// inputs produce stopped efforts with a safety reason; successful submission
    /// of an intent does not guarantee motion. Also refreshes [`Self::autonomy_status`].
    ///
    /// # Errors
    /// Returns `Internal` for a poisoned mutex or propagates a rejected controller
    /// target as `InvalidInput`. Timing and freshness stops are returned as output.
    pub fn step(&self, timestamp: f64) -> Result<ControlOutput, ControllerError> {
        let mut brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        if brain.dashboard.as_ref().is_some_and(|d| d.failed()) {
            self.apply_dashboard_link_loss(&mut brain, timestamp);
        }
        let incoming = brain
            .dashboard
            .as_ref()
            .filter(|d| !d.failed())
            .map(|d| d.take_actions())
            .unwrap_or_default();
        brain.last_tick = timestamp;
        capture::advance(&mut brain, timestamp);
        for (kind, bytes, age) in incoming {
            brain.runtime.ingest(terra_runtime::RuntimeEvent::Remote {
                kind,
                payload: bytes,
                received_at: timestamp - age,
            })?;
        }
        let recording = brain.recording_healthy();
        let runtime_output = brain.runtime.step(timestamp)?;
        let start = brain.run_start().unwrap_or(timestamp);
        let run_id = brain.run_id().to_owned();
        for event in &runtime_output.events {
            if let Some(recorder) = brain.recorder.as_ref() {
                let _=recorder.record(serde_json::json!({"kind":"runtime.event","event":event,"time":timestamp-start,"source_time":timestamp,"run_id":run_id}));
            }
            self.observation.event(
                "runtime",
                event["kind"].as_str().unwrap_or("runtime.event"),
                event.clone(),
                &run_id,
                timestamp,
            );
        }
        let fleet_telemetry = brain
            .dashboard
            .is_some()
            .then(|| (runtime_output.telemetry(), runtime_output.search_reports()));
        let mut output = runtime_output.autonomy;
        let status = runtime_output.status;
        // The dock owns the gate: open only on its request, closed otherwise.
        let mut gate = brain.runtime.gate_servo(output.gate_open);
        if let (Some((id, position)), Some(layout)) = (gate, brain.actuators.as_mut()) {
            let rejected = layout.set_servo(id, position).err().map(|e| e.to_string());
            if rejected.is_some() {
                // Not in the actuator frame, so not reported as applied.
                gate = None;
            }
            if rejected != brain.gate_rejected {
                if let Some(error) = &rejected {
                    self.observation.event(
                        "actuators",
                        "actuators.gate_rejected",
                        serde_json::json!({"servo": id, "error": error}),
                        &run_id,
                        timestamp,
                    );
                }
                brain.gate_rejected = rejected;
            }
        }
        let pose = brain.pose().unwrap_or_default();
        let map = brain.map().cloned();
        let clearance = map.as_ref().and_then(|m| {
            terra_navigation::observed_clearance(&m.0, pose.0, &brain.autonomy().footprint())
        });
        if let Some(kind) = brain.near_miss.update(clearance) {
            output.events.push(terra_autonomy::DecisionEvent {
                kind: kind.into(),
                token: None,
                reason: "observed_clearance".into(),
                level: output.status.requested_level,
            });
        }
        let map_publication = if brain.dashboard.is_some() {
            map.as_ref().map(|(grid, time, sequence)| {
                if !publication::fresh_map_sample(timestamp, *time) {
                    return publication::Publication::Skip;
                }
                if !brain.map_gate.is_newer(&run_id, *sequence) {
                    return publication::Publication::Skip;
                }
                if !publication::map_visualization_valid(grid, &run_id) {
                    brain.map_gate.reject_source(run_id.clone(), *sequence);
                    return publication::Publication::Skip;
                }
                let content = (
                    run_id.clone(),
                    grid.width,
                    grid.height,
                    grid.resolution,
                    grid.origin_x,
                    grid.origin_y,
                    grid.occupancy.clone(),
                );
                brain.map_gate.observe(
                    content,
                    run_id.clone(),
                    *sequence,
                    std::time::Instant::now(),
                    std::time::Duration::from_millis(200),
                )
            })
        } else {
            None
        };
        let mut map_full_rejected = false;
        // Telemetry expires after 500 ms, so the cached body is resent every
        // step and only re-serialized when the object map changes.
        if let Some(rover) = brain.dashboard.as_ref().map(|d| d.rover_id)
            && brain
                .objects_published
                .as_ref()
                .is_none_or(|(revision, _)| *revision != brain.objects().revision)
        {
            brain.objects_published = Some((
                brain.objects().revision,
                brain.objects().telemetry(rover, &run_id).to_string(),
            ));
        }
        if let Some(d) = brain.dashboard.as_ref() {
            d.publish(telemetry::EXPERIMENT_STATUS,serde_json::json!({
                "run_id": run_id,
                "run_elapsed": timestamp-start,
                "schema_version": 1,
                "utc": std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default().as_secs_f64(),
                "recording": recording && brain.recorder.is_some(),
                "runtime": status["runtime"],
            }).to_string());
            if let Some((topics, reports)) = fleet_telemetry {
                for (topic, body) in topics {
                    d.publish(topic, body);
                }
                d.publish_batch(telemetry::SEARCH_REPORT, reports);
            }
            if let Some((_, objects)) = &brain.objects_published {
                d.publish(telemetry::OBJECTS, objects.clone());
            }
            if let Some((grid, _, sequence)) = map.as_ref() {
                match map_publication {
                    Some(publication::Publication::Full) => {
                        map_full_rejected = !d.publish_visualization(
                            telemetry::MAP_OCCUPANCY,
                            terra_autonomy::occupancy_telemetry(
                                d.rover_id, &run_id, *sequence, grid,
                            )
                            .to_string(),
                        );
                    }
                    Some(publication::Publication::Heartbeat(sequence)) => d.publish(
                        telemetry::MAP_OCCUPANCY_HEARTBEAT,
                        serde_json::json!({"schema_version":1,"rover_id":d.rover_id,
                            "run_id":run_id,"sequence":sequence})
                        .to_string(),
                    ),
                    _ => {}
                }
            }
            if brain.pose_ready(timestamp) {
                d.publish(
                    telemetry::POSE,
                    serde_json::json!({
                        "rover_id": d.rover_id,
                        "sequence": brain.sequence(),
                        "x": pose.0.x,
                        "y": pose.0.y,
                        "yaw": pose.0.yaw,
                    })
                    .to_string(),
                );
            }
        }
        brain.map_gate.full_enqueued(!map_full_rejected);
        for event in &output.events {
            self.observation.event(
                "autonomy",
                "decision.event",
                serde_json::to_value(event).unwrap_or_default(),
                &run_id,
                timestamp,
            );
        }
        let estimate = brain.estimate(timestamp);
        self.observation.tick(&brain, timestamp, &estimate, &status);
        if let Some(r) = brain.recorder.as_ref() {
            let _ = r.record(serde_json::json!({
                "kind": "control_tick",
                "time": timestamp-start,
                "source_time": timestamp,
                "sequence": brain.sequence(),
                "clearance": clearance,
                "selected": output.twist,
                "source_command": output.intent,
                "status": output.status,
                "goal": output.goal,
                "proposal": status["proposal"],
                "run_id": run_id,
                "exploration": output.exploration,
                "search": output.search,
                "task": output.task,
                "events": output.events,
            }));
        }
        self.observation
            .effort(&brain, timestamp, &runtime_output.motor);
        let mut control: ControlOutput = runtime_output.motor.into();
        control.actuator_frame = brain
            .actuators
            .as_ref()
            .and_then(|layout| layout.frame(control.left_effort, control.right_effort).ok());
        control.gate_servo = gate.map(|(id, position)| GateServo { id, position });
        Ok(control)
    }

    /// Reset estimator, velocity loop, and autonomy; detach dashboard and close recording.
    /// Clears pose and map while retaining configured navigation geometry.
    ///
    /// # Errors
    /// Returns `Internal` for a poisoned mutex. Recorder-close errors are ignored.
    pub fn reset(&self) -> Result<(), ControllerError> {
        let mut brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        self.brain.inbox.clear();
        brain.runtime.reset()?;
        brain.last_tick = 0.;
        brain.dashboard = None;
        brain.map_gate = Default::default();
        brain.cloud_gate = Default::default();
        if let Some(mut r) = brain.recorder.take() {
            let _ = r.close();
        }
        Ok(())
    }
    /// Queue depths and how many sensor samples were dropped or rejected.
    pub fn sensor_queue_stats(&self) -> SensorQueueStats {
        self.brain.inbox.stats()
    }
}

/// Apply one queued sensor sample to the runtime. Called by `step` with the
/// controller locked, in timestamp order.
fn apply_sample(brain: &mut Brain, sample: inbox::Sample) -> Result<(), ControllerError> {
    match sample {
        inbox::Sample::Imu(sample) => {
            brain.runtime.ingest(terra_runtime::RuntimeEvent::Imu {
                sample: ImuSample {
                    timestamp: sample.timestamp,
                    acceleration: Vector3 {
                        x: sample.acceleration_forward,
                        y: sample.acceleration_left,
                        z: sample.acceleration_up,
                    },
                    angular_velocity: Vector3 {
                        x: sample.gyro_roll,
                        y: sample.gyro_pitch,
                        z: sample.gyro_yaw,
                    },
                },
                received_at: sample.timestamp,
            })?;
            Ok(())
        }
        inbox::Sample::Vio(sample) => {
            let event = terra_runtime::RuntimeEvent::Vio {
                sample: VioSample {
                    timestamp: sample.timestamp,
                    position: Vector3 {
                        x: sample.position_x,
                        y: sample.position_y,
                        z: sample.position_z,
                    },
                    orientation: Quaternion {
                        x: sample.quaternion_x,
                        y: sample.quaternion_y,
                        z: sample.quaternion_z,
                        w: sample.quaternion_w,
                    },
                    velocity: Vector3 {
                        x: sample.velocity_x,
                        y: sample.velocity_y,
                        z: sample.velocity_z,
                    },
                    tracked: sample.tracked,
                },
                received_at: sample.timestamp,
            };
            brain.runtime.ingest(event)?;
            Ok(())
        }
        inbox::Sample::Gps(fix, now) => {
            Ok(brain.runtime.ingest(terra_runtime::RuntimeEvent::Gps {
                sample: fix.into(),
                received_at: now,
            })?)
        }
        inbox::Sample::Heading(fix, now) => {
            Ok(brain.runtime.ingest(terra_runtime::RuntimeEvent::Heading {
                sample: fix.into(),
                received_at: now,
            })?)
        }
        inbox::Sample::Course(fix, now) => {
            Ok(brain.runtime.ingest(terra_runtime::RuntimeEvent::Course {
                sample: fix.into(),
                received_at: now,
            })?)
        }
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
    let brain = MobileController::new(default_control_settings())?;
    let (mut v, mut w, mut yaw, mut ax) = (0.0, 0.0, 0.0, 0.0);
    for tick in 0..1000 {
        let t = tick as f64 * 0.01;
        brain.push_imu(ImuReading {
            timestamp: t,
            acceleration_forward: ax,
            acceleration_left: v * w,
            acceleration_up: 0.0,
            gyro_roll: 0.0,
            gyro_pitch: 0.0,
            gyro_yaw: w,
        })?;
        if tick % 5 == 0 {
            brain.push_vio(VioReading {
                timestamp: t,
                position_x: 0.0,
                position_y: 0.0,
                position_z: 0.0,
                quaternion_x: 0.0,
                quaternion_y: 0.0,
                quaternion_z: (yaw * 0.5_f64).sin(),
                quaternion_w: (yaw * 0.5_f64).cos(),
                velocity_x: v * yaw.cos(),
                velocity_y: v * yaw.sin(),
                velocity_z: 0.0,
                tracked: true,
            })?;
        }
        brain.set_target(TwistSetpoint {
            timestamp: t,
            forward: if tick < 300 { 2.0 } else { 0.8 },
            yaw_rate: 0.3,
        })?;
        let output = brain.step(t)?;
        ax = 3.0 * (output.left_effort + output.right_effort) * 0.5 - v - 0.15;
        v += ax * 0.01;
        w += (5.0 * (output.right_effort - output.left_effort) * 0.5 - w) * 0.01;
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
    /// The published status keeps its shape: the apps and dashboards read
    /// these fields, and the brief is the same record cut down.
    #[test]
    fn status_and_brief_keep_their_published_fields() {
        let controller = MobileController::new(default_control_settings()).unwrap();
        assert_eq!(controller.autonomy_status(), "");
        assert_eq!(controller.autonomy_brief(), "");
        controller.step(0.0).unwrap();
        controller.step(0.01).unwrap();
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
        assert_eq!(brief["status"]["safety"], status["status"]["safety"]);
        assert_eq!(brief["goal"]["state"], status["goal"]["state"]);
        assert_eq!(brief["pose_quality"], status["pose_quality"]);
        let typed = controller.autonomy_brief_typed();
        assert_eq!(typed.requested_level, "teleop");
        assert_eq!(typed.goal_state, "idle");
        assert!(typed.pose_degraded);
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
        controller.step(0.0).unwrap();
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

#[cfg(test)]
mod runtime_parity_tests;
