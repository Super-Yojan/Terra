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
pub use autonomy::{FpvEffect, WebrtcSignal};
mod observability;
pub use observability::ObservationAttachment;
pub mod outdoor;
mod outdoor_bridge;
pub use outdoor::{CourseFix, GpsFix, HeadingFix};
#[cfg(test)]
mod navigation_settings;
mod pose_quality;
mod publication;
mod search;
pub use search::{MobileSearchRequest, MobileSearchSnapshot, MobileTargetObservation};
mod frames;
mod mapping;
mod transport;
mod waypoint;
pub use frames::*;
pub use mapping::*;
pub use transport::*;
pub use waypoint::*;

/// Shared presentation and router-loss defaults. Local sensor limits are independent.
#[uniffi::export]
pub fn default_connectivity_policy_settings() -> String {
    serde_json::to_string(&connectivity_policy::ConnectivityPolicySettings::default())
        .expect("finite policy defaults")
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
        ControllerConfig {
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
        }
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
    run_start: Option<f64>,
    map_gate: publication::MapGate,
    cloud_gate: publication::CloudGate,
    dashboard: Option<terra_transport::ControlPlane>,
    /// Running point capture, if the dashboard asked for one.
    capture: Option<capture::Capture>,
    last_tick: f64,
    actuators: Option<actuators::ActiveLayout>,
}
impl std::ops::Deref for Brain {
    type Target = terra_runtime::Runtime;
    fn deref(&self) -> &Self::Target {
        &self.runtime
    }
}
impl std::ops::DerefMut for Brain {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.runtime
    }
}
impl From<terra_runtime::RuntimeError> for ControllerError {
    fn from(e: terra_runtime::RuntimeError) -> Self {
        Self::InvalidInput {
            message: e.to_string(),
        }
    }
}
#[derive(uniffi::Object)]
/// Shared sensor estimator, autonomy arbiter, and velocity controller.
/// Methods serialize state access with a mutex. Supply timestamped sensors,
/// submit intent, and call [`Self::step`] on the same monotonic clock.
/// Returned efforts still require an independently armed hardware adapter.
pub struct MobileController {
    brain: Mutex<Brain>,
    /// Video signaling only. It cannot arm the rover or change the velocity target.
    fpv: Mutex<fpv_signaling::SignalingSession>,
    observation: observability::ObservationState,
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
            brain: Mutex::new(Brain {
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
                run_start: None,
                map_gate: Default::default(),
                cloud_gate: Default::default(),
                dashboard: None,
                capture: None,
                last_tick: 0.,
                actuators: None,
            }),
        }))
    }
    /// Validate and ingest one body-frame IMU reading.
    ///
    /// # Errors
    /// Returns `InvalidInput` for invalid sensor values or sample ordering and
    /// `Internal` if shared state cannot be locked.
    pub fn push_imu(&self, sample: ImuReading) -> Result<(), ControllerError> {
        let mut brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
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
    /// Ingest VIO and, when tracked, update the planar world pose.
    /// The tracked quaternion is normalized before extracting heading.
    ///
    /// # Errors
    /// Returns `InvalidInput` for invalid readings or sample ordering and
    /// `Internal` for a poisoned mutex. Pose assignment precedes estimator validation.
    pub fn push_vio(&self, sample: VioReading) -> Result<(), ControllerError> {
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
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .runtime
            .ingest(event)?;
        Ok(())
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
    /// Trusted local normalized manual input; remote envelopes cannot access this ingress.
    pub fn set_manual_input(
        &self,
        forward: f64,
        turn: f64,
        timestamp: f64,
    ) -> Result<(), ControllerError> {
        let mut b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        if b.dashboard.as_ref().is_some_and(|d| !d.failed()) {
            return Err(ControllerError::InvalidInput {
                message: "fleet connection owns velocity intent".into(),
            });
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
        if brain.dashboard.is_some() {
            return Err(ControllerError::InvalidInput {
                message: "fleet connection owns velocity intent".into(),
            });
        }
        if !target.timestamp.is_finite()
            || !target.forward.is_finite()
            || !target.yaw_rate.is_finite()
            || target.forward.abs() > 5.
            || target.yaw_rate.abs() > 5.
        {
            return Err(InputError::OutOfRange.into());
        }
        brain.autonomy.accept_operator(
            terra_autonomy::TeleopRequest {
                linear: target.forward,
                angular: target.yaw_rate,
                run_id: None,
                authority_revision: None,
                operator_session_id: None,
                sequence: None,
            },
            target.timestamp,
        );
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
        let estimate = brain.estimator.estimate(timestamp);
        let pose = brain.pose.unwrap_or_default();
        let map = brain.map.clone();
        if brain.dashboard.as_ref().is_some_and(|d| d.failed()) && !brain.router_loss_handled {
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
        brain.runtime.recording_healthy = brain.recorder.as_ref().is_none_or(|r| !r.failed());
        let runtime_output = brain.runtime.step(timestamp)?;
        for event in &runtime_output.events {
            if let Some(recorder) = brain.recorder.as_ref() {
                let _=recorder.record(serde_json::json!({"kind":"runtime.event","event":event,"time":timestamp-brain.run_start.unwrap_or(timestamp),"source_time":timestamp,"run_id":brain.run_id}));
            }
            self.observation.event(
                "runtime",
                event["kind"].as_str().unwrap_or("runtime.event"),
                event.clone(),
                &brain.run_id,
                timestamp,
            );
        }
        let mut output = runtime_output.autonomy;
        let outdoor_motion = output.status.safety == "clear";
        let mut outdoor_status = brain.outdoor.status(
            timestamp,
            outdoor_bridge::evidence(&brain, timestamp),
            &brain.run_id,
        );
        if brain.outdoor.selected {
            if brain.outdoor.phase() == "planning"
                && matches!(
                    output.status.reason.as_str(),
                    "obstacle_blocked"
                        | "route_no_path"
                        | "route_goal_blocked"
                        | "heading_requires_spin_or_replan"
                )
            {
                outdoor_status["hold_reason"] = serde_json::json!("route_blocked");
            }
            if !outdoor_motion && output.status.safety != "emergency_stop" {
                output.status.reason = outdoor_status["hold_reason"]
                    .as_str()
                    .unwrap_or(brain.outdoor.phase())
                    .to_string();
            }
            if let Some(goal) = brain.outdoor.mission_status() {
                output.goal = goal;
            }
        }
        let clearance = map.as_ref().and_then(|m| {
            terra_navigation::observed_clearance(&m.0, pose.0, brain.autonomy.planner.config.radius)
        });
        if let Some(kind) = brain.near_miss.update(clearance) {
            output.events.push(terra_autonomy::DecisionEvent {
                kind: kind.into(),
                token: None,
                reason: "observed_clearance".into(),
                level: output.status.requested_level,
            });
        }
        let start = *brain.run_start.get_or_insert(timestamp);
        brain.sequence += 1;
        let wire_proposal = output.proposal.clone().map(|mut p| {
            p.expires_at -= start;
            p
        });
        let map_run = brain.run_id.clone();
        let map_publication = if brain.dashboard.is_some() {
            map.as_ref().map(|(grid, time, sequence)| {
                if !publication::fresh_map_sample(timestamp, *time) {
                    return publication::Publication::Skip;
                }
                if !brain.map_gate.is_newer(&map_run, *sequence) {
                    return publication::Publication::Skip;
                }
                if !publication::map_visualization_valid(grid, &map_run) {
                    brain.map_gate.reject_source(map_run.clone(), *sequence);
                    return publication::Publication::Skip;
                }
                let content = (
                    brain.run_id.clone(),
                    grid.width,
                    grid.height,
                    grid.resolution,
                    grid.origin_x,
                    grid.origin_y,
                    grid.occupancy.clone(),
                );
                brain.map_gate.observe(
                    content,
                    map_run.clone(),
                    *sequence,
                    std::time::Instant::now(),
                    std::time::Duration::from_millis(200),
                )
            })
        } else {
            None
        };
        let quality = pose_quality::value(&brain, timestamp);
        let mut map_full_rejected = false;
        if let Some(d) = brain.dashboard.as_ref() {
            d.publish("experiment/status",serde_json::json!({
                "run_id": brain.run_id,
                "run_elapsed": timestamp-start,
                "schema_version": 1,
                "utc": std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default().as_secs_f64(),
                "recording": brain.recorder.as_ref().is_some_and(|r|!r.failed()),
                "runtime": runtime_output.snapshot.value["runtime"],
            }).to_string());
            let mut status = serde_json::to_value(&output.status).unwrap();
            status["runtime"] = runtime_output.snapshot.value["runtime"].clone();
            status["connectivity"] = serde_json::json!({"policy":brain.connectivity,"revision":brain.connectivity_revision});
            status["navigation"] = brain.navigation.status();
            status["navigation"]["outdoor"] = outdoor_status.clone();
            d.publish("navigation/outdoor", outdoor_status.to_string());
            status["pose_quality"] = quality.clone();
            d.publish("pose/quality", quality.to_string());
            d.publish("autonomy/status", status.to_string());
            d.publish(
                "search/status",
                serde_json::to_string(&output.search).unwrap(),
            );
            d.publish_batch(
                "search/report",
                output
                    .pending_reports
                    .iter()
                    .map(|r| serde_json::to_string(r).unwrap())
                    .collect(),
            );
            d.publish("goal/status", serde_json::to_string(&output.goal).unwrap());
            d.publish(
                "goal/proposal",
                serde_json::to_string(&wire_proposal).unwrap(),
            );
            if let Some(exploration) = &output.exploration {
                d.publish(
                    "exploration/status",
                    serde_json::to_string(exploration).unwrap(),
                );
            }
            if let Some((grid, _, sequence)) = map.as_ref() {
                match map_publication {
                    Some(publication::Publication::Full) => {
                        map_full_rejected = !d.publish_visualization(
                            "map/occupancy",
                            terra_autonomy::occupancy_telemetry(
                                d.rover_id,
                                &brain.run_id,
                                *sequence,
                                grid,
                            )
                            .to_string(),
                        );
                    }
                    Some(publication::Publication::Heartbeat(sequence)) => d.publish(
                        "map/occupancy/heartbeat",
                        serde_json::json!({"schema_version":1,"rover_id":d.rover_id,
                            "run_id":brain.run_id,"sequence":sequence})
                        .to_string(),
                    ),
                    _ => {}
                }
            }
            if brain.pose.is_some()
                && estimate.health == Health::Ready
                && timestamp - pose.1 <= 0.35
            {
                d.publish(
                    "pose",
                    serde_json::json!({
                        "rover_id": d.rover_id,
                        "sequence": brain.sequence,
                        "x": pose.0.x,
                        "y": pose.0.y,
                        "yaw": pose.0.yaw,
                    })
                    .to_string(),
                );
            }
        }
        brain.map_gate.full_enqueued(!map_full_rejected);
        brain.autonomy_json = serde_json::json!({
            "status": output.status,
            "command_origin": output.command_origin,
            "runtime": runtime_output.snapshot.value["runtime"],
            "pose_quality": quality,
            "outdoor": outdoor_status,
            "navigation": brain.navigation.status(),
            "connectivity": {"policy": brain.connectivity, "revision": brain.connectivity_revision,
                "accepted_onboard_mission": autonomy::accepted_mission(&brain)},
            "goal": output.goal,
            "proposal": wire_proposal,
            "exploration": output.exploration,
            "search": output.search,
            "pending_reports": output.pending_reports,
            "twist": output.twist,
        })
        .to_string();
        for event in &output.events {
            self.observation.event(
                "autonomy",
                "decision.event",
                serde_json::to_value(event).unwrap_or_default(),
                &brain.run_id,
                timestamp,
            );
        }
        self.observation.tick(&brain, timestamp, &estimate);
        if let Some(r) = brain.recorder.as_ref() {
            let _ = r.record(serde_json::json!({
                "kind": "control_tick",
                "time": timestamp-start,
                "source_time": timestamp,
                "sequence": brain.sequence,
                "clearance": clearance,
                "selected": output.twist,
                "source_command": output.intent,
                "status": output.status,
                "goal": output.goal,
                "proposal": wire_proposal,
                "run_id": brain.run_id,
                "exploration": output.exploration,
            "search": output.search,
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
        Ok(control)
    }

    /// Reset estimator, velocity loop, and autonomy; detach dashboard and close recording.
    /// Clears pose and map while retaining configured navigation geometry.
    ///
    /// # Errors
    /// Returns `Internal` for a poisoned mutex. Recorder-close errors are ignored.
    pub fn reset(&self) -> Result<(), ControllerError> {
        let mut brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
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
}

#[cfg(test)]
mod navigation_settings_tests;
#[cfg(test)]
mod runtime_parity_tests;
