//! Thread-safe phone bindings around the portable robotics crates.
//!
//! [`MobileController`] combines sensor estimation and velocity control behind a
//! mutex. The `mapping`, `waypoint`, `autonomy`, `actuators`, `transport`, and
//! `search` modules expose their respective capabilities through UniFFI. Keep
//! phone-facing conversion here and algorithm changes in the owning Rust crate.

use std::sync::{Arc, Mutex};
use terra_control::{ControllerConfig, VelocityController};
use terra_state::{EstimatorConfig, VelocityEstimator};
use terra_types::*;
uniffi::setup_scaffolding!();
mod actuators;
pub use actuators::*;
mod autonomy;
mod observability;
pub use observability::ObservationAttachment;
pub mod outdoor;
mod outdoor_bridge;
pub use outdoor::{GpsFix, HeadingFix};
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
    outdoor: outdoor::OutdoorController,
    outdoor_tokens: std::collections::VecDeque<String>,
    outdoor_goal_tokens: std::collections::VecDeque<String>,
    control_limits: (f64, f64),
    estimator: VelocityEstimator,
    imu_time: Option<f64>,
    controller: VelocityController,
    autonomy: terra_autonomy::AutonomyArbiter,
    pose: Option<(terra_navigation::Pose, f64)>,
    map: Option<(terra_mapping::MapSnapshot, f64, u64)>,
    autonomy_json: String,
    recorder: Option<terra_experiment::RunRecorder>,
    near_miss: terra_experiment::NearMiss,
    run_start: Option<f64>,
    run_id: String,
    sequence: u64,
    map_gate: publication::MapGate,
    cloud_gate: publication::CloudGate,
    dashboard: Option<terra_transport::ControlPlane>,
    navigation: navigation_settings::State,
    hardware_observation: Option<(serde_json::Value, std::time::Instant)>,
    hardware_requests: std::collections::VecDeque<(String, std::time::Instant)>,
    hardware_tokens: std::collections::VecDeque<String>,
}
#[derive(uniffi::Object)]
/// Shared sensor estimator, autonomy arbiter, and velocity controller.
/// Methods serialize state access with a mutex. Supply timestamped sensors,
/// submit intent, and call [`Self::step`] on the same monotonic clock.
/// Returned efforts still require an independently armed hardware adapter.
pub struct MobileController {
    brain: Mutex<Brain>,
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
            observation: Default::default(),
            brain: Mutex::new(Brain {
                outdoor: Default::default(),
                outdoor_tokens: Default::default(),
                outdoor_goal_tokens: Default::default(),
                control_limits: (settings.max_forward, settings.max_yaw_rate),
                estimator: VelocityEstimator::new(settings.estimator())?,
                imu_time: None,
                controller: VelocityController::new(settings.controller())?,
                autonomy: terra_autonomy::AutonomyArbiter::default(),
                pose: None,
                map: None,
                autonomy_json: String::new(),
                recorder: None,
                near_miss: terra_experiment::NearMiss::default(),
                run_start: None,
                run_id: format!(
                    "phone-{}",
                    std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .unwrap_or_default()
                        .as_nanos()
                ),
                sequence: 0,
                map_gate: Default::default(),
                cloud_gate: Default::default(),
                dashboard: None,
                navigation: Default::default(),
                hardware_observation: None,
                hardware_requests: Default::default(),
                hardware_tokens: Default::default(),
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
        brain.estimator.push_imu(ImuSample {
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
        })?;
        brain.imu_time = Some(sample.timestamp);
        Ok(())
    }
    /// Ingest VIO and, when tracked, update the planar world pose.
    /// The tracked quaternion is normalized before extracting heading.
    ///
    /// # Errors
    /// Returns `InvalidInput` for invalid readings or sample ordering and
    /// `Internal` for a poisoned mutex. Pose assignment precedes estimator validation.
    pub fn push_vio(&self, sample: VioReading) -> Result<(), ControllerError> {
        if sample.tracked {
            let q = Quaternion {
                x: sample.quaternion_x,
                y: sample.quaternion_y,
                z: sample.quaternion_z,
                w: sample.quaternion_w,
            }
            .normalized()
            .ok_or(InputError::OutOfRange)?;
            let forward = q.rotate(Vector3 {
                x: 1.,
                y: 0.,
                z: 0.,
            });
            self.brain
                .lock()
                .map_err(|_| ControllerError::Internal)?
                .pose = Some((
                terra_navigation::Pose {
                    x: sample.position_x,
                    y: sample.position_y,
                    yaw: forward.y.atan2(forward.x),
                },
                sample.timestamp,
            ));
        }
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .estimator
            .push_vio(VioSample {
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
        let incoming = brain
            .dashboard
            .as_ref()
            .map(|d| d.take_actions())
            .unwrap_or_default();
        for (kind, bytes, age) in incoming {
            self.observation.event("router", "command.received", serde_json::json!({"topic":kind,"queue_age":age,"payload":serde_json::from_slice::<serde_json::Value>(&bytes).ok()}), &brain.run_id, timestamp);
            match kind.as_str() {
                "navigation/settings" => navigation_settings::apply(&mut brain, &bytes, age),
                "navigation/source" | "navigation/resume" => {
                    let _ = outdoor_bridge::request(&mut brain, &kind, &bytes, timestamp, age);
                }
                "hardware" => {
                    let authority: serde_json::Value =
                        serde_json::from_str(&brain.autonomy_json).unwrap_or_default();
                    if let Some(request) = admit_hardware_request(&bytes, age, &authority) {
                        let token = request["token"].as_str().unwrap().to_string();
                        if !brain.hardware_tokens.contains(&token) {
                            if brain.hardware_tokens.len() >= 64 {
                                brain.hardware_tokens.pop_front();
                            }
                            brain.hardware_tokens.push_back(token);
                            // Disarm takes priority and cancels queued arming.
                            if request["action"] == "disarm" {
                                brain.outdoor.pause("disarmed");
                                brain.hardware_requests.clear();
                            }
                            if brain.hardware_requests.len() < 8 {
                                brain.hardware_requests.push_back((
                                    request.to_string(),
                                    std::time::Instant::now()
                                        - std::time::Duration::from_secs_f64(
                                            if request["action"] == "arm" { age } else { 0. },
                                        ),
                                ));
                            }
                        }
                    }
                }
                "autonomy" => {
                    if let Some(r) = terra_autonomy::decode_level(&bytes) {
                        brain.autonomy.set_level(r);
                    }
                }
                "teleop" | "cmd_vel" => {
                    if let Some(r) = terra_autonomy::decode_teleop(&bytes) {
                        let admitted = brain.autonomy.accept_operator_from(
                            r,
                            timestamp - age,
                            terra_autonomy::CommandOrigin::FleetTeleop,
                        );
                        self.observation.event(
                            "autonomy",
                            "command.admission",
                            serde_json::json!({"topic":kind,"admitted":admitted,"age":age}),
                            &brain.run_id,
                            timestamp,
                        );
                    }
                }
                "goal" => {
                    outdoor_bridge::dispatch_goal(&mut brain, &bytes, timestamp, age);
                }
                "safety" => {
                    if let Some(r) = terra_autonomy::decode_safety(&bytes) {
                        let ready = brain.autonomy.is_manual() || estimate.health == Health::Ready;
                        brain.autonomy.set_safety(r, ready);
                    }
                }
                "search" | "search/action" | "search/report/ack" => {
                    if age < 0.5 {
                        let _ = search::dispatch_search(&mut brain, &kind, &bytes, timestamp);
                    }
                }
                "goal/decision" => {
                    if let Some(r) = terra_autonomy::decode_decision(&bytes) {
                        let ready = pose_quality::ready(&brain, timestamp);
                        brain.autonomy.decide_proposal(
                            r,
                            &terra_autonomy::ArbiterInput {
                                now: timestamp,
                                pose: pose.0,
                                pose_time: pose.1,
                                map: map.as_ref().map(|m| &m.0),
                                map_time: map.as_ref().map(|m| m.1),
                                map_revision: map.as_ref().map(|m| m.2).unwrap_or(0),
                                measured: terra_navigation::Twist {
                                    linear: estimate.forward,
                                    angular: estimate.yaw_rate,
                                },
                                healthy: ready,
                            },
                        );
                    }
                }
                _ => {}
            }
        }
        let previous: serde_json::Value =
            serde_json::from_str(&brain.autonomy_json).unwrap_or_default();
        let local_waypoint = matches!(
            previous["status"]["requested_level"].as_str(),
            Some("waypoint" | "waypoint_direct")
        ) && previous["goal"]["state"] == "active";
        let outdoor_motion = outdoor_bridge::update(&mut brain, timestamp);
        // L0 is routed as direct operator effort on the phone. Its authority
        // must survive camera loss while transport/recording failures still stop it.
        let healthy = (brain.autonomy.is_manual() || pose_quality::ready(&brain, timestamp))
            && outdoor_motion
            && (local_waypoint || brain.dashboard.as_ref().is_none_or(|d| !d.failed()))
            && brain.recorder.as_ref().is_none_or(|r| !r.failed());
        brain.autonomy.run_id = brain.run_id.clone();
        let limits = brain.control_limits;
        brain.autonomy.planner.config.max_linear = brain
            .navigation
            .parameters
            .max_linear
            .min(limits.0)
            .min(if brain.outdoor.selected {
                0.3
            } else {
                f64::INFINITY
            });
        brain.autonomy.planner.config.max_angular =
            brain.navigation.parameters.max_angular.min(limits.1);
        let mut output = brain.autonomy.step(terra_autonomy::ArbiterInput {
            now: timestamp,
            pose: pose.0,
            pose_time: pose.1,
            map: map.as_ref().map(|m| &m.0),
            map_time: map.as_ref().map(|m| m.1),
            map_revision: map.as_ref().map(|m| m.2).unwrap_or(0),
            measured: terra_navigation::Twist {
                linear: estimate.forward,
                angular: estimate.yaw_rate,
            },
            healthy,
        });
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
            }).to_string());
            let mut status = serde_json::to_value(&output.status).unwrap();
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
            "pose_quality": quality,
            "outdoor": outdoor_status,
            "navigation": brain.navigation.status(),
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
        if output.reset_controller {
            brain.controller.reset();
        }
        if output.status.safety != "clear" {
            brain.controller.reset();
            let stopped = MotorOutput::stopped(if estimate.health == Health::Ready {
                StopReason::StaleTarget
            } else {
                StopReason::SensorNotReady
            });
            self.observation.effort(&brain, timestamp, &stopped);
            return Ok(stopped.into());
        }
        let turning = (output.command_origin != terra_autonomy::CommandOrigin::LocalManual)
            .then_some(brain.navigation.parameters.turning);
        brain.controller.set_turning_constraint(turning);
        brain.controller.set_target(VelocityTarget {
            timestamp,
            forward: output.twist.linear,
            yaw_rate: output.twist.angular,
        })?;
        let effort = brain.controller.step(estimate);
        self.observation.effort(&brain, timestamp, &effort);
        Ok(effort.into())
    }
    /// Reset estimator, velocity loop, and autonomy; detach dashboard and close recording.
    /// Clears pose and map while retaining configured navigation geometry.
    ///
    /// # Errors
    /// Returns `Internal` for a poisoned mutex. Recorder-close errors are ignored.
    pub fn reset(&self) -> Result<(), ControllerError> {
        let mut brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        brain.outdoor.cancel();
        brain.outdoor.invalidate_sensors();
        brain.outdoor.selected = false;
        brain.outdoor.revision = brain.outdoor.revision.saturating_add(1);
        brain.autonomy = terra_autonomy::AutonomyArbiter::default();
        let planner = brain.navigation.parameters.planner();
        brain.autonomy.configure_planner(planner);
        let turning = brain.navigation.parameters.turning;
        brain.autonomy.configure_turning(turning)?;
        brain.dashboard = None;
        brain.pose = None;
        brain.imu_time = None;
        brain.map = None;
        if let Some(mut r) = brain.recorder.take() {
            let _ = r.close();
        }
        brain.estimator.reset();
        brain.controller.reset();
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

fn admit_hardware_request(
    bytes: &[u8],
    age: f64,
    authority: &serde_json::Value,
) -> Option<serde_json::Value> {
    let r: serde_json::Value = serde_json::from_slice(bytes).ok()?;
    let token = r["token"].as_str()?;
    if !terra_autonomy::valid_token(token) {
        return None;
    }
    match r["action"].as_str()? {
        "disarm" => Some(r),
        "arm"
            if age.is_finite()
                && (0.0..0.5).contains(&age)
                && r["run_id"].is_string()
                && r["run_id"] == authority["status"]["run_id"]
                && r["authority_revision"].is_u64()
                && r["authority_revision"] == authority["status"]["revision"]
                && authority["status"]["safety"] == "clear" =>
        {
            Some(r)
        }
        _ => None,
    }
}
/// Hardware arm freshness/authority checks and stop-priority regression scenarios.
#[cfg(test)]
mod hardware_request_tests {
    use super::*;
    #[test]
    /// Queues disarm before arm and verifies stop-aware draining places disarm last in the control tick.
    fn hardware_disarm_dominates_same_tick_arm() {
        let mut q = std::collections::VecDeque::new();
        terra_transport::enqueue_control_action(
            &mut q,
            "hardware".into(),
            br#"{"action":"disarm","token":"stop"}"#.to_vec(),
            std::time::Instant::now(),
        );
        terra_transport::enqueue_control_action(
            &mut q,
            "hardware".into(),
            br#"{"action":"arm","token":"arm"}"#.to_vec(),
            std::time::Instant::now(),
        );
        let actions = terra_transport::drain_control_actions(&mut q);
        let final_request: serde_json::Value =
            serde_json::from_slice(&actions.last().unwrap().1).unwrap();
        assert_eq!(final_request["action"], "disarm");
    }
    #[test]
    /// Rejects queued arms with stale authority or receipt time while still consuming an old disarm once.
    fn queued_arm_is_revalidated_before_consumption() {
        let c = MobileController::new(default_control_settings()).unwrap();
        let r = r#"{"action":"arm","token":"one","run_id":"run","authority_revision":2}"#;
        {
            let mut b = c.brain.lock().unwrap();
            b.autonomy_json = r#"{"status":{"run_id":"run","revision":3,"safety":"clear"}}"#.into();
            b.hardware_requests
                .push_back((r.into(), std::time::Instant::now()));
        }
        assert!(c.take_hardware_request().is_empty());
        {
            let mut b = c.brain.lock().unwrap();
            b.autonomy_json = r#"{"status":{"run_id":"run","revision":2,"safety":"clear"}}"#.into();
            b.hardware_requests.push_back((
                r.into(),
                std::time::Instant::now() - std::time::Duration::from_secs(1),
            ));
            b.hardware_requests.push_back((
                r#"{"action":"disarm","token":"stop"}"#.into(),
                std::time::Instant::now() - std::time::Duration::from_secs(1),
            ));
        }
        assert!(c.take_hardware_request().contains("disarm"));
        assert!(c.take_hardware_request().is_empty());
    }
    #[test]
    /// Checks the strict half-second arm lease, run/revision binding, and clear-safety requirement; disarm remains admissible when old.
    fn arm_requires_current_run_revision_and_fresh_receipt() {
        let a = serde_json::json!({"status": {"run_id": "run","revision": 2,"safety": "clear"}});
        let r = br#"{"action":"arm","token":"one","run_id":"run","authority_revision":2}"#;
        assert!(admit_hardware_request(r, 0.1, &a).is_some());
        assert!(admit_hardware_request(r, 0.5, &a).is_none());
        assert!(
            admit_hardware_request(
                r,
                0.1,
                &serde_json::json!({"status": {"run_id": "other","revision": 2,"safety": "clear"}})
            )
            .is_none()
        );
        assert!(
            admit_hardware_request(
                r,
                0.1,
                &serde_json::json!({"status": {"run_id": "run","revision": 3,"safety": "clear"}})
            )
            .is_none()
        );
        assert!(admit_hardware_request(r, 0.1, &serde_json::json!({"status": {"run_id": "run","revision": 2,"safety": "emergency_stop"}})).is_none());
        assert!(admit_hardware_request(br#"{"action":"disarm","token":"stop"}"#, 8., &a).is_some());
    }
}

#[cfg(test)]
mod navigation_settings_tests;
