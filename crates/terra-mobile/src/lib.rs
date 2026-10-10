//! Thread-safe UniFFI boundary. Robotics packages have no UniFFI dependencies.
use std::sync::{Arc, Mutex};
use terra_control::{ControllerConfig, VelocityController};
use terra_state::{EstimatorConfig, VelocityEstimator};
use terra_types::*;
uniffi::setup_scaffolding!();
mod actuators;
pub use actuators::*;
mod autonomy;
mod search;
pub use search::{MobileSearchRequest,MobileTargetObservation,MobileSearchSnapshot};
mod mapping;
mod transport;
mod waypoint;
pub use frames::*;
pub use mapping::*;
pub use transport::*;
pub use waypoint::*;

#[derive(Clone, Debug, uniffi::Record)]
pub struct ControlSettings {
    pub linear_kp: f64,
    pub linear_ki: f64,
    pub yaw_kp: f64,
    pub yaw_ki: f64,
    pub linear_feedforward: f64,
    pub yaw_feedforward: f64,
    pub max_forward: f64,
    pub max_yaw_rate: f64,
    pub max_effort: f64,
    pub imu_timeout: f64,
    pub vio_timeout: f64,
    pub target_timeout: f64,
    pub max_step: f64,
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
pub struct ImuReading {
    pub timestamp: f64,
    pub acceleration_forward: f64,
    pub acceleration_left: f64,
    pub acceleration_up: f64,
    pub gyro_roll: f64,
    pub gyro_pitch: f64,
    pub gyro_yaw: f64,
}
#[derive(Clone, Debug, uniffi::Record)]
pub struct VioReading {
    pub timestamp: f64,
    pub position_x: f64,
    pub position_y: f64,
    pub position_z: f64,
    pub quaternion_x: f64,
    pub quaternion_y: f64,
    pub quaternion_z: f64,
    pub quaternion_w: f64,
    pub velocity_x: f64,
    pub velocity_y: f64,
    pub velocity_z: f64,
    pub tracked: bool,
}
#[derive(Clone, Debug, uniffi::Record)]
pub struct TwistSetpoint {
    pub timestamp: f64,
    pub forward: f64,
    pub yaw_rate: f64,
}
#[derive(Clone, Copy, Debug, PartialEq, uniffi::Enum)]
pub enum SafetyState {
    Active,
    SensorNotReady,
    TrackingLost,
    StaleSensors,
    StaleTarget,
    InvalidTime,
}
#[derive(Clone, Debug, uniffi::Record)]
pub struct ControlOutput {
    pub left_effort: f64,
    pub right_effort: f64,
    pub estimated_forward: f64,
    pub estimated_yaw_rate: f64,
    pub target_forward: f64,
    pub target_yaw_rate: f64,
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
pub enum ControllerError {
    #[error("{message}")]
    InvalidInput { message: String },
    #[error("controller lock was poisoned")]
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
    estimator: VelocityEstimator,
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
    last_map_publish: Option<f64>,
    dashboard: Option<terra_transport::ControlPlane>,
    hardware_requests: std::collections::VecDeque<(String, std::time::Instant)>,
    hardware_tokens: std::collections::VecDeque<String>,
}
#[derive(uniffi::Object)]
pub struct MobileController {
    brain: Mutex<Brain>,
}
#[uniffi::export]
impl MobileController {
    #[uniffi::constructor]
    pub fn new(settings: ControlSettings) -> Result<Arc<Self>, ControllerError> {
        Ok(Arc::new(Self {
            brain: Mutex::new(Brain {
                estimator: VelocityEstimator::new(settings.estimator())?,
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
                last_map_publish: None,
                dashboard: None,
                hardware_requests: Default::default(),
                hardware_tokens: Default::default(),
            }),
        }))
    }
    pub fn push_imu(&self, sample: ImuReading) -> Result<(), ControllerError> {
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .estimator
            .push_imu(ImuSample {
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
        Ok(())
    }
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
    pub fn set_target(&self, target: TwistSetpoint) -> Result<(), ControllerError> {
        let mut brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
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
            match kind.as_str() {
                "hardware" => {
                    let authority: serde_json::Value = serde_json::from_str(&brain.autonomy_json).unwrap_or_default();
                    if let Some(request) = admit_hardware_request(&bytes, age, &authority) {
                        let token = request["token"].as_str().unwrap().to_string();
                        if !brain.hardware_tokens.contains(&token) {
                            if brain.hardware_tokens.len() >= 64 { brain.hardware_tokens.pop_front(); }
                            brain.hardware_tokens.push_back(token);
                            // Disarm takes priority and cancels queued arming.
                            if request["action"] == "disarm" { brain.hardware_requests.clear(); }
                            if brain.hardware_requests.len() < 8 { brain.hardware_requests.push_back((request.to_string(), std::time::Instant::now() - std::time::Duration::from_secs_f64(if request["action"] == "arm" { age } else { 0. }))); }
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
                        brain.autonomy.accept_operator(r, timestamp - age);
                    }
                }
                "goal" => {
                    if let Some(g) = terra_waypoint::decode_goal(&bytes) {
                        brain.autonomy.accept_goal(&g, 20_000.);
                    }
                }
                "safety" => {
                    if let Some(r) = terra_autonomy::decode_safety(&bytes) {
                        brain
                            .autonomy
                            .set_safety(r, estimate.health == Health::Ready);
                    }
                }
                "search" | "search/action" | "search/report/ack" => {if age<0.5 {let _=search::dispatch_search(&mut brain,&kind,&bytes,timestamp);}}
                "goal/decision" => {
                    if let Some(r) = terra_autonomy::decode_decision(&bytes) {
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
                                healthy: estimate.health == Health::Ready,
                            },
                        );
                    }
                }
                _ => {}
            }
        }
        let previous: serde_json::Value = serde_json::from_str(&brain.autonomy_json).unwrap_or_default();
        let local_waypoint = matches!(previous["status"]["requested_level"].as_str(), Some("waypoint" | "waypoint_direct")) && previous["goal"]["state"] == "active";
        let healthy = estimate.health == Health::Ready
            && (local_waypoint || brain.dashboard.as_ref().is_none_or(|d| !d.failed()))
            && brain.recorder.as_ref().is_none_or(|r| !r.failed());
        brain.autonomy.run_id = brain.run_id.clone();
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
        let publish_map = map.is_some()
            && brain
                .last_map_publish
                .is_none_or(|last| timestamp - last >= 0.2);
        if publish_map {
            brain.last_map_publish = Some(timestamp);
        }
        if let Some(d) = brain.dashboard.as_ref() {
            d.publish("experiment/status",serde_json::json!({"run_id":brain.run_id,"run_elapsed":timestamp-start,"schema_version":1,"utc":std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default().as_secs_f64(),"recording":brain.recorder.as_ref().is_some_and(|r|!r.failed())}).to_string());
            d.publish(
                "autonomy/status",
                serde_json::to_string(&output.status).unwrap(),
            );
            d.publish("search/status",serde_json::to_string(&output.search).unwrap());
            d.publish_batch("search/report",output.pending_reports.iter().map(|r|serde_json::to_string(r).unwrap()).collect());
            d.publish("goal/status", serde_json::to_string(&output.goal).unwrap());
            d.publish(
                "goal/proposal",
                serde_json::to_string(&wire_proposal).unwrap(),
            );
            if publish_map && let Some((grid, _, sequence)) = map.as_ref() {
                d.publish(
                    "map/occupancy",
                    terra_autonomy::occupancy_telemetry(d.rover_id, &brain.run_id, *sequence, grid)
                        .to_string(),
                );
            }
            if brain.pose.is_some()
                && estimate.health == Health::Ready
                && timestamp - pose.1 <= 0.35
            {
                d.publish("pose",serde_json::json!({"rover_id":d.rover_id,"sequence":brain.sequence,"x":pose.0.x,"y":pose.0.y,"yaw":pose.0.yaw}).to_string());
            }
        }
        brain.autonomy_json =
            serde_json::json!({"status":output.status,"goal":output.goal,"proposal":wire_proposal,"search":output.search,"pending_reports":output.pending_reports,"twist":output.twist})
                .to_string();
        if let Some(r) = brain.recorder.as_ref() {
            let _=r.record(serde_json::json!({"kind":"control_tick","time":timestamp-start,"source_time":timestamp,"sequence":brain.sequence,"clearance":clearance,"selected":output.twist,"source_command":output.intent,"status":output.status,"goal":output.goal,"proposal":wire_proposal,"run_id":brain.run_id,"search":output.search,"events":output.events}));
        }
        if output.reset_controller {
            brain.controller.reset();
        }
        if output.status.safety != "clear" {
            brain.controller.reset();
            return Ok(MotorOutput::stopped(if estimate.health == Health::Ready {
                StopReason::StaleTarget
            } else {
                StopReason::SensorNotReady
            })
            .into());
        }
        brain.controller.set_target(VelocityTarget {
            timestamp,
            forward: output.twist.linear,
            yaw_rate: output.twist.angular,
        })?;
        Ok(brain.controller.step(estimate).into())
    }
    pub fn reset(&self) -> Result<(), ControllerError> {
        let mut brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        brain.autonomy = terra_autonomy::AutonomyArbiter::default();
        brain.dashboard = None;
        brain.pose = None;
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
pub struct BenchmarkReport {
    pub final_forward: f64,
    pub final_yaw_rate: f64,
    pub forward_error: f64,
    pub yaw_error: f64,
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
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn mobile_facade_runs_the_same_sensor_and_motor_loop() {
        let report = run_velocity_benchmark().unwrap();
        assert!(report.passed, "{report:?}");
    }
    #[test]
    fn validates_settings_before_crossing_the_ffi_boundary() {
        let mut settings = default_control_settings();
        settings.max_effort = 1.5;
        assert!(MobileController::new(settings).is_err());
    }
}

fn admit_hardware_request(bytes: &[u8], age: f64, authority: &serde_json::Value) -> Option<serde_json::Value> {
    let r: serde_json::Value = serde_json::from_slice(bytes).ok()?;
    let token = r["token"].as_str()?;
    if !terra_autonomy::valid_token(token) { return None; }
    match r["action"].as_str()? {
        "disarm" => Some(r),
        "arm" if age.is_finite() && (0.0..0.5).contains(&age)
            && r["run_id"].is_string() && r["run_id"] == authority["status"]["run_id"]
            && r["authority_revision"].is_u64() && r["authority_revision"] == authority["status"]["revision"]
            && authority["status"]["safety"] == "clear" => Some(r),
        _ => None,
    }
}
#[cfg(test)]
mod hardware_request_tests {
    use super::*;
    #[test] fn hardware_disarm_dominates_same_tick_arm() {
        let mut q = std::collections::VecDeque::new();
        terra_transport::enqueue_control_action(&mut q, "hardware".into(), br#"{"action":"disarm","token":"stop"}"#.to_vec(), std::time::Instant::now());
        terra_transport::enqueue_control_action(&mut q, "hardware".into(), br#"{"action":"arm","token":"arm"}"#.to_vec(), std::time::Instant::now());
        let actions = terra_transport::drain_control_actions(&mut q);
        let final_request: serde_json::Value = serde_json::from_slice(&actions.last().unwrap().1).unwrap();
        assert_eq!(final_request["action"], "disarm");
    }
    #[test] fn queued_arm_is_revalidated_before_consumption() {
        let c = MobileController::new(default_control_settings()).unwrap();
        let r = r#"{"action":"arm","token":"one","run_id":"run","authority_revision":2}"#;
        {
            let mut b = c.brain.lock().unwrap();
            b.autonomy_json = r#"{"status":{"run_id":"run","revision":3,"safety":"clear"}}"#.into();
            b.hardware_requests.push_back((r.into(), std::time::Instant::now()));
        }
        assert!(c.take_hardware_request().is_empty());
        {
            let mut b = c.brain.lock().unwrap();
            b.autonomy_json = r#"{"status":{"run_id":"run","revision":2,"safety":"clear"}}"#.into();
            b.hardware_requests.push_back((r.into(), std::time::Instant::now() - std::time::Duration::from_secs(1)));
            b.hardware_requests.push_back((r#"{"action":"disarm","token":"stop"}"#.into(), std::time::Instant::now() - std::time::Duration::from_secs(1)));
        }
        assert!(c.take_hardware_request().contains("disarm"));
        assert!(c.take_hardware_request().is_empty());
    }
    #[test] fn arm_requires_current_run_revision_and_fresh_receipt() {
        let a = serde_json::json!({"status":{"run_id":"run","revision":2,"safety":"clear"}});
        let r = br#"{"action":"arm","token":"one","run_id":"run","authority_revision":2}"#;
        assert!(admit_hardware_request(r, 0.1, &a).is_some());
        assert!(admit_hardware_request(r, 0.5, &a).is_none());
        assert!(admit_hardware_request(r, 0.1, &serde_json::json!({"status":{"run_id":"other","revision":2,"safety":"clear"}})).is_none());
        assert!(admit_hardware_request(r, 0.1, &serde_json::json!({"status":{"run_id":"run","revision":3,"safety":"clear"}})).is_none());
        assert!(admit_hardware_request(r, 0.1, &serde_json::json!({"status":{"run_id":"run","revision":2,"safety":"emergency_stop"}})).is_none());
        assert!(admit_hardware_request(br#"{"action":"disarm","token":"stop"}"#, 8., &a).is_some());
    }
}
