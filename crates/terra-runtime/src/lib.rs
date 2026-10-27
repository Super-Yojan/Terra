//! Platform-independent control orchestration. Time and all external evidence are injected.
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::collections::VecDeque;
use terra_autonomy::{ArbiterInput, ArbiterOutput, AutonomyArbiter, CommandOrigin};
use terra_control::{ControllerConfig, VelocityController};
use terra_mapping::{CameraIntrinsics, CameraPose, GroundPlane, MapConfig, MapSnapshot};
use terra_navigation::{Pose, Twist};
use terra_state::{EstimatorConfig, VelocityEstimator};
use terra_types::*;
pub const SOURCE_REVISION: &str = env!("TERRA_RUNTIME_REVISION");
mod command;
pub mod ground;
pub mod mapping;
pub mod navigation;
pub mod outdoor;
pub mod pose_quality;
pub mod rover_profile;
pub mod search;
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RuntimeError {
    InvalidInput { message: String },
}
impl std::fmt::Display for RuntimeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidInput { message } => f.write_str(message),
        }
    }
}
impl std::error::Error for RuntimeError {}
impl From<InputError> for RuntimeError {
    fn from(e: InputError) -> Self {
        invalid(&format!("{e:?}"))
    }
}
impl From<terra_mapping::MapError> for RuntimeError {
    fn from(e: terra_mapping::MapError) -> Self {
        invalid(&format!("{e:?}"))
    }
}
fn invalid(s: &str) -> RuntimeError {
    RuntimeError::InvalidInput { message: s.into() }
}
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ActuationMode {
    DirectEffort,
    #[default]
    VelocityFeedback,
}
#[derive(Clone, Debug)]
pub struct RuntimeConfig {
    pub estimator: EstimatorConfig,
    pub controller: ControllerConfig,
    pub navigation: navigation::Parameters,
    pub mapping: MapConfig,
    pub connectivity: connectivity_policy::ConnectivityPolicySettings,
    pub mode: ActuationMode,
    pub outdoor_supported: bool,
}
impl Default for RuntimeConfig {
    fn default() -> Self {
        Self {
            estimator: Default::default(),
            controller: Default::default(),
            navigation: Default::default(),
            mapping: Default::default(),
            connectivity: Default::default(),
            mode: Default::default(),
            outdoor_supported: false,
        }
    }
}
impl RuntimeConfig {
    pub fn identity(&self) -> String {
        let data = format!(
            "v1:{:?}:{:?}:{:?}:{}:{}:{:?}:{}",
            self.controller,
            self.estimator,
            self.mapping,
            serde_json::to_string(&self.navigation).unwrap_or_default(),
            serde_json::to_string(&self.connectivity).unwrap_or_default(),
            self.mode,
            self.outdoor_supported
        );
        let hash = data.bytes().fold(14695981039346656037_u64, |h, b| {
            (h ^ u64::from(b)).wrapping_mul(1099511628211)
        });
        format!("fnv1a64:{hash:016x}")
    }
}
pub enum RuntimeEvent {
    Imu {
        sample: ImuSample,
        received_at: f64,
    },
    Vio {
        sample: VioSample,
        received_at: f64,
    },
    Depth {
        timestamp: f64,
        received_at: f64,
        intrinsics: CameraIntrinsics,
        camera: CameraPose,
        depth: Vec<f32>,
        ground: Option<GroundPlane>,
    },
    Map {
        snapshot: MapSnapshot,
        timestamp: f64,
        sequence: u64,
        received_at: f64,
    },
    LocalManual {
        forward: f64,
        turn: f64,
        received_at: f64,
    },
    LocalVelocity {
        target: VelocityTarget,
        received_at: f64,
    },
    Remote {
        kind: String,
        payload: Vec<u8>,
        received_at: f64,
    },
    Link {
        connected: bool,
        received_at: f64,
    },
    Recording {
        healthy: bool,
        received_at: f64,
    },
    Hardware {
        status: Value,
        received_at: f64,
    },
    Mode {
        mode: ActuationMode,
        received_at: f64,
    },
    Gps {
        sample: terra_autonomy::outdoor::GpsFix,
        received_at: f64,
    },
    Heading {
        sample: terra_autonomy::outdoor::HeadingFix,
        received_at: f64,
    },
    Course {
        sample: terra_autonomy::outdoor::CourseFix,
        received_at: f64,
    },
    Detector {
        classes: Vec<String>,
        version: String,
        received_at: f64,
    },
    Target {
        sample: terra_autonomy::TargetObservation,
        received_at: f64,
    },
}
#[derive(Clone, Debug, PartialEq)]
pub struct RuntimeSnapshot {
    pub value: Value,
}
pub struct RuntimeOutput {
    pub autonomy: ArbiterOutput,
    pub motor: MotorOutput,
    pub snapshot: RuntimeSnapshot,
    pub events: Vec<Value>,
}
pub struct Runtime {
    pub estimator: VelocityEstimator,
    pub controller: VelocityController,
    pub autonomy: AutonomyArbiter,
    pub pose: Option<(Pose, f64)>,
    pub map: Option<(MapSnapshot, f64, u64)>,
    pub imu_time: Option<f64>,
    pub run_id: String,
    pub sequence: u64,
    pub rover_id: Option<u64>,
    pub autonomy_json: String,
    /// The few status fields the phone's control loop reads every tick,
    /// so it never parses the full status (planner trace included) at 100 Hz.
    pub autonomy_brief: String,
    pub navigation: navigation::State,
    pub outdoor: terra_autonomy::outdoor::OutdoorController,
    /// Buckets and balls seen this run, in the local frame.
    pub objects: terra_autonomy::objects::ObjectMap,
    pub outdoor_tokens: VecDeque<String>,
    pub outdoor_goal_tokens: VecDeque<String>,
    pub hardware_observation: Option<(Value, f64)>,
    pub connectivity: connectivity_policy::ConnectivityPolicySettings,
    pub connectivity_revision: u64,
    pub router_loss_handled: bool,
    pub recording_healthy: bool,
    pub control_limits: (f64, f64),
    pub mode: ActuationMode,
    configuration: RuntimeConfig,
    grid: mapping::MappingState,
    queue: VecDeque<(String, Vec<u8>, f64)>,
    events: Vec<Value>,
    last_step: Option<f64>,
    local_manual: Option<(f64, f64)>,
    last_snapshot: RuntimeSnapshot,
    /// Latest `sensor_unhealthy` hold and when it happened.
    sensor_hold: Option<(f64, terra_autonomy::SensorHold)>,
    sensor_holds: u64,
    sensor_held: bool,
}
/// How long a `sensor_unhealthy` hold stays in the status after it clears.
const SENSOR_HOLD_DISPLAY_SECONDS: f64 = 10.;
impl Runtime {
    pub fn new(config: RuntimeConfig, run_id: String) -> Result<Self, RuntimeError> {
        if run_id.is_empty()
            || !config.navigation.valid()
            || config.connectivity.validate().is_err()
        {
            return Err(invalid("invalid runtime configuration"));
        }
        let configuration = config.clone();
        let mut autonomy = AutonomyArbiter::default();
        autonomy.run_id = run_id.clone();
        if serde_json::to_value(config.navigation.planner()).ok()
            != serde_json::to_value(terra_navigation::PlannerConfig::default()).ok()
        {
            autonomy.configure_planner(config.navigation.planner());
        }
        autonomy.configure_turning(config.navigation.turning)?;
        let mut outdoor = terra_autonomy::outdoor::OutdoorController::default();
        outdoor.supported = config.outdoor_supported;
        let mut navigation = navigation::State::default();
        navigation.parameters = config.navigation;
        Ok(Self {
            estimator: VelocityEstimator::new(config.estimator)?,
            controller: VelocityController::new(config.controller)?,
            autonomy,
            pose: None,
            map: None,
            imu_time: None,
            run_id,
            sequence: 0,
            rover_id: None,
            autonomy_json: String::new(),
            autonomy_brief: String::new(),
            navigation,
            outdoor,
            objects: Default::default(),
            outdoor_tokens: Default::default(),
            outdoor_goal_tokens: Default::default(),
            hardware_observation: None,
            connectivity: config.connectivity,
            connectivity_revision: 0,
            router_loss_handled: false,
            recording_healthy: true,
            control_limits: (
                config.controller.max_forward,
                config.controller.max_yaw_rate,
            ),
            mode: config.mode,
            configuration,
            grid: mapping::MappingState::new(config.mapping)?,
            queue: Default::default(),
            events: vec![],
            last_step: None,
            local_manual: None,
            sensor_hold: None,
            sensor_holds: 0,
            sensor_held: false,
            last_snapshot: RuntimeSnapshot { value: json!({}) },
        })
    }
    pub fn metadata(&self) -> Value {
        json!({"schema_version":1,"configured_mode":self.mode,"source_revision":SOURCE_REVISION,"configuration_identity":self.configuration_identity()})
    }
    /// Replace run authority without discarding the adapter's current sensor clock.
    pub fn begin_run(&mut self, run_id: String) -> Result<(), RuntimeError> {
        if run_id.is_empty() {
            return Err(invalid("invalid run identity"));
        }
        self.run_id = run_id;
        self.queue.clear();
        self.events.clear();
        self.controller.reset();
        self.sequence = 0;
        self.sensor_hold = None;
        self.sensor_holds = 0;
        self.sensor_held = false;
        self.outdoor.cancel();
        self.outdoor.invalidate_sensors();
        self.objects.clear();
        self.outdoor.selected = false;
        self.outdoor.revision = self.outdoor.revision.saturating_add(1);
        self.autonomy = AutonomyArbiter::default();
        self.autonomy.run_id = self.run_id.clone();
        self.autonomy
            .configure_planner(self.navigation.parameters.planner());
        self.autonomy
            .configure_turning(self.navigation.parameters.turning)?;
        Ok(())
    }
    /// Begin a fresh control session while retaining configured policy and calibration.
    pub fn reset(&mut self) -> Result<(), RuntimeError> {
        self.begin_run(self.run_id.clone())?;
        self.estimator.reset();
        self.controller.reset();
        self.pose = None;
        self.imu_time = None;
        self.map = None;
        self.hardware_observation = None;
        self.queue.clear();
        self.events.clear();
        self.last_step = None;
        self.local_manual = None;
        self.last_snapshot = RuntimeSnapshot { value: json!({}) };
        self.autonomy_json.clear();
        self.autonomy_brief.clear();
        self.grid.clear();
        Ok(())
    }
    /// Add one detector sighting in the local frame. Adds latitude and longitude
    /// when the outdoor GPS fix and heading are good enough.
    pub fn observe_object(
        &mut self,
        mut sighting: terra_autonomy::objects::ObjectSighting,
        now: f64,
    ) -> Result<(), RuntimeError> {
        sighting.geo = self.pose.and_then(|(pose, t)| {
            (now - t <= 0.35)
                .then(|| self.outdoor.geolocate(now, pose, sighting.x, sighting.y))
                .flatten()
        });
        self.objects.observe(sighting).map_err(invalid)
    }
    pub fn configuration_identity(&self) -> String {
        let mut config = self.configuration.clone();
        config.mapping = self.grid.config();
        config.navigation = self.navigation.parameters.clone();
        config.connectivity = self.connectivity.clone();
        config.mode = self.mode;
        config.outdoor_supported = self.outdoor.supported;
        config.identity()
    }
    /// Drive and plan as the rover in `profile`: planner footprint, turning,
    /// speed limits and velocity-loop gains change together. The velocity loop
    /// restarts, so call this while the rover is stopped.
    pub fn apply_rover_profile(
        &mut self,
        profile: rover_profile::RoverProfile,
    ) -> Result<(), RuntimeError> {
        if !profile.valid() {
            return Err(invalid("invalid rover profile"));
        }
        let navigation = profile.navigation(&self.navigation.parameters);
        let controller = profile.controller(self.configuration.controller);
        if !navigation.valid() || controller.validate().is_err() {
            return Err(invalid("invalid rover profile"));
        }
        self.autonomy.configure_planner(navigation.planner());
        self.autonomy.configure_turning(navigation.turning)?;
        self.controller = VelocityController::new(controller)?;
        self.configuration.controller = controller;
        self.control_limits = (controller.max_forward, controller.max_yaw_rate);
        self.navigation.parameters = navigation;
        self.navigation.revision = self.navigation.revision.saturating_add(1);
        self.navigation.profile = Some(profile.name.clone());
        self.navigation.result = Some(format!("profile {}", profile.name));
        self.outdoor.revision = self.outdoor.revision.saturating_add(1);
        self.events
            .push(json!({"kind":"rover_profile.applied","profile":profile}));
        Ok(())
    }
    pub fn mapping_handle(&self) -> mapping::MappingState {
        self.grid.clone()
    }
    pub fn attach_mapping(&mut self, map: mapping::MappingState) {
        self.grid = map;
        self.map = None;
    }
    pub fn ingest(&mut self, event: RuntimeEvent) -> Result<(), RuntimeError> {
        let received = match &event {
            RuntimeEvent::Imu { received_at, .. }
            | RuntimeEvent::Vio { received_at, .. }
            | RuntimeEvent::Depth { received_at, .. }
            | RuntimeEvent::Map { received_at, .. }
            | RuntimeEvent::LocalManual { received_at, .. }
            | RuntimeEvent::LocalVelocity { received_at, .. }
            | RuntimeEvent::Remote { received_at, .. }
            | RuntimeEvent::Link { received_at, .. }
            | RuntimeEvent::Recording { received_at, .. }
            | RuntimeEvent::Hardware { received_at, .. }
            | RuntimeEvent::Mode { received_at, .. }
            | RuntimeEvent::Gps { received_at, .. }
            | RuntimeEvent::Heading { received_at, .. }
            | RuntimeEvent::Course { received_at, .. }
            | RuntimeEvent::Detector { received_at, .. }
            | RuntimeEvent::Target { received_at, .. } => *received_at,
        };
        if !received.is_finite() || received < 0.0 {
            return Err(invalid("invalid receipt timestamp"));
        }
        match event {
            RuntimeEvent::Imu { sample, .. } => {
                if sample.timestamp > received {
                    return Err(invalid("future sensor timestamp"));
                }
                self.estimator.push_imu(sample)?;
                self.imu_time = Some(sample.timestamp);
            }
            RuntimeEvent::Vio { sample, .. } => {
                if sample.timestamp > received {
                    return Err(invalid("future sensor timestamp"));
                }
                self.estimator.push_vio(sample)?;
                if sample.tracked {
                    let q = sample
                        .orientation
                        .normalized()
                        .ok_or(InputError::OutOfRange)?;
                    let f = q.rotate(Vector3 {
                        x: 1.,
                        y: 0.,
                        z: 0.,
                    });
                    let pose = Pose {
                        x: sample.position.x,
                        y: sample.position.y,
                        yaw: f.y.atan2(f.x),
                    };
                    self.pose = Some((pose, sample.timestamp));
                    self.outdoor.observe_pose(pose, sample.timestamp);
                }
            }
            RuntimeEvent::Depth {
                timestamp,
                intrinsics,
                camera,
                depth,
                ground,
                ..
            } => {
                if timestamp > received {
                    return Err(invalid("future depth timestamp"));
                }
                let p = self.pose.unwrap_or_default().0;
                self.grid.recenter(p.x, p.y)?;
                match ground {
                    Some(ground) => self.grid.integrate_depth_with_ground_plane(
                        timestamp, intrinsics, camera, &depth, ground,
                    )?,
                    None => self
                        .grid
                        .integrate_depth(timestamp, intrinsics, camera, &depth)?,
                };
                self.map = Some((
                    self.grid.snapshot(),
                    timestamp,
                    self.map.as_ref().map_or(1, |m| m.2 + 1),
                ));
            }
            RuntimeEvent::Map {
                snapshot,
                timestamp,
                sequence,
                ..
            } => {
                if timestamp > received {
                    return Err(invalid("future map timestamp"));
                }
                if self.map.as_ref().is_none_or(|m| timestamp > m.1) {
                    self.map = Some((snapshot, timestamp, sequence));
                }
            }
            RuntimeEvent::LocalManual { forward, turn, .. } => {
                if !forward.is_finite()
                    || !turn.is_finite()
                    || forward.abs() > 1.
                    || turn.abs() > 1.
                {
                    return Err(invalid("invalid manual input"));
                }
                self.local_manual = Some((forward, turn));
                self.autonomy.accept_operator(
                    terra_autonomy::TeleopRequest {
                        linear: forward * 0.5,
                        angular: turn,
                        run_id: None,
                        authority_revision: None,
                        operator_session_id: None,
                        sequence: None,
                    },
                    received,
                );
            }
            RuntimeEvent::LocalVelocity { target, .. } => {
                if !target.timestamp.is_finite()
                    || target.timestamp < 0.0
                    || target.timestamp > received
                    || !target.forward.is_finite()
                    || !target.yaw_rate.is_finite()
                    || target.forward.abs() > 5.0
                    || target.yaw_rate.abs() > 5.0
                {
                    return Err(invalid("invalid local velocity input"));
                }
                self.autonomy.accept_operator(
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
            }
            RuntimeEvent::Remote { kind, payload, .. } => {
                if payload.len() > 4096 || self.queue.len() >= 256 {
                    return Err(invalid("remote request queue full or oversized"));
                }
                self.queue.push_back((kind, payload, received));
            }
            RuntimeEvent::Link { connected, .. } => self.link(connected),
            RuntimeEvent::Recording { healthy, .. } => {
                self.recording_healthy = healthy;
                self.events
                    .push(json!({"kind":"recording.health","healthy":healthy}));
            }
            RuntimeEvent::Hardware { status, .. } => {
                self.hardware_observation = Some((status, received))
            }
            RuntimeEvent::Mode { mode, .. } => {
                if mode != self.mode {
                    self.controller.reset();
                    self.local_manual = None;
                    self.autonomy.clear_operator_intent();
                    self.autonomy.invalidate_remote_authority();
                    self.mode = mode;
                    self.events
                        .push(json!({"kind":"actuation.mode_changed","mode":mode}));
                }
            }
            RuntimeEvent::Gps { sample, .. } => {
                self.outdoor.gps(sample, received).map_err(invalid)?
            }
            RuntimeEvent::Heading { sample, .. } => {
                self.outdoor.heading(sample, received).map_err(invalid)?
            }
            RuntimeEvent::Course { sample, .. } => {
                self.outdoor.course(sample, received).map_err(invalid)?
            }
            RuntimeEvent::Detector {
                classes, version, ..
            } => {
                self.autonomy
                    .set_detector_capability(classes, version, received)
                    .map_err(invalid)?;
            }
            RuntimeEvent::Target { sample, .. } => {
                self.autonomy
                    .accept_target_observation(sample, received)
                    .map_err(invalid)?;
            }
        }
        Ok(())
    }
    pub fn set_connectivity_policy(
        &mut self,
        policy: connectivity_policy::ConnectivityPolicySettings,
    ) -> Result<(), RuntimeError> {
        policy.validate().map_err(invalid)?;
        self.connectivity = policy;
        if self.router_loss_handled
            && self.connectivity.autonomous_router_loss
                == connectivity_policy::RouterLossPolicy::Pause
        {
            self.outdoor.pause("router_loss_paused");
            self.autonomy.pause_for_router_loss();
            self.controller.reset();
        }
        self.connectivity_revision = self.connectivity_revision.saturating_add(1);
        self.events.push(json!({"kind":"connectivity.policy_changed","policy":self.connectivity,"revision":self.connectivity_revision}));
        Ok(())
    }
    pub fn link(&mut self, connected: bool) {
        if connected {
            if self.router_loss_handled {
                self.autonomy.invalidate_remote_authority();
            }
            self.router_loss_handled = false;
        } else if !self.router_loss_handled {
            self.autonomy.invalidate_remote_authority();
            self.queue.clear();
            if self.connectivity.autonomous_router_loss
                == connectivity_policy::RouterLossPolicy::Pause
            {
                self.autonomy.pause_for_router_loss();
                self.outdoor.pause("router_loss_paused");
                self.controller.reset();
            }
            self.router_loss_handled = true;
        }
        self.events
            .push(json!({"kind":if connected{"connection.established"}else{"connection.lost"}}));
    }
    /// Name the cause of a `sensor_unhealthy` hold in the status, and log
    /// each new hold, so holds of a single tick are not silent.
    fn note_sensor_hold(&mut self, output: &mut ArbiterOutput, now: f64, outdoor_motion: bool) {
        if output.status.reason == "sensor_unhealthy" {
            let hold = pose_quality::hold(self, now, outdoor_motion);
            if !self.sensor_held {
                self.sensor_holds += 1;
                self.events.push(json!({"kind":"autonomy.sensor_hold","cause":hold.cause,"pose_age_s":hold.pose_age_s,"imu_age_s":hold.imu_age_s,"count":self.sensor_holds}));
            }
            self.sensor_held = true;
            self.sensor_hold = Some((now, hold));
        } else {
            self.sensor_held = false;
        }
        output.status.sensor_hold = self
            .sensor_hold
            .as_ref()
            .filter(|(at, _)| now - at <= SENSOR_HOLD_DISPLAY_SECONDS)
            .map(|(at, hold)| terra_autonomy::SensorHold {
                age_s: now - at,
                count: self.sensor_holds,
                ..hold.clone()
            });
    }
    pub fn step(&mut self, now: f64) -> Result<RuntimeOutput, RuntimeError> {
        if !now.is_finite() || now < 0.0 || self.last_step.is_some_and(|last| now < last) {
            return Err(invalid("invalid control time"));
        }
        self.apply_pending(now);
        self.last_step = Some(now);
        let estimate = self.estimator.estimate(now);
        let outdoor_motion = outdoor::update(self, now);
        // A paused outdoor mission holds L3 motion only. L0 takeover drives
        // while the mission waits for an explicit resume.
        let manual = self.autonomy.is_manual();
        let healthy = (manual || (pose_quality::ready(self, now) && outdoor_motion))
            && self.recording_healthy;
        self.autonomy.planner.config.max_linear = self
            .navigation
            .parameters
            .max_linear
            .min(self.control_limits.0)
            .min(if self.outdoor.selected {
                0.3
            } else {
                f64::INFINITY
            });
        self.autonomy.planner.config.max_angular = self
            .navigation
            .parameters
            .max_angular
            .min(self.control_limits.1);
        let p = self.pose.unwrap_or_default();
        let map = self.map.as_ref();
        let mut output = self.autonomy.step(ArbiterInput {
            now,
            pose: p.0,
            pose_time: p.1,
            map: map.map(|m| &m.0),
            map_time: map.map(|m| m.1),
            map_revision: map.map_or(0, |m| m.2),
            measured: Twist {
                linear: estimate.forward,
                angular: estimate.yaw_rate,
            },
            healthy,
        });
        self.note_sensor_hold(&mut output, now, outdoor_motion);
        let outdoor_status = self
            .outdoor
            .status(now, outdoor::evidence(self, now), &self.run_id);
        if self.outdoor.selected {
            if let Some(goal) = self.outdoor.mission_status() {
                output.goal = goal;
            }
            if !outdoor_motion && !manual && output.status.safety != "emergency_stop" {
                output.status.reason = outdoor_status["hold_reason"]
                    .as_str()
                    .unwrap_or(self.outdoor.phase())
                    .into();
            }
        }
        if output.reset_controller {
            self.controller.reset();
        }
        let effective_mode = if self.mode == ActuationMode::DirectEffort
            && !matches!(
                output.status.requested_level,
                terra_autonomy::Level::Teleop | terra_autonomy::Level::AssistedTeleop
            ) {
            ActuationMode::VelocityFeedback
        } else {
            self.mode
        };
        let motor = if output.status.safety != "clear" {
            self.controller.reset();
            MotorOutput::stopped(if estimate.health == Health::Ready {
                StopReason::StaleTarget
            } else {
                StopReason::SensorNotReady
            })
        } else if effective_mode == ActuationMode::DirectEffort {
            let (forward, turn) = (output.twist.linear / 0.5, output.twist.angular);
            let managed = output.command_origin != CommandOrigin::LocalManual;
            let turn = if managed
                && !self.navigation.parameters.turning.allow_in_place
                && !output.in_place_turn
            {
                turn.clamp(-forward.abs(), forward.abs())
            } else {
                turn
            };
            let scale = (forward - turn).abs().max((forward + turn).abs()).max(1.);
            let (left, right) = ((forward - turn) / scale, (forward + turn) / scale);
            MotorOutput {
                left,
                right,
                estimated_forward: estimate.forward,
                estimated_yaw_rate: estimate.yaw_rate,
                target_forward: output.twist.linear,
                target_yaw_rate: output.twist.angular,
                stop_reason: StopReason::None,
            }
        } else {
            self.controller.set_turning_constraint(
                (output.command_origin != CommandOrigin::LocalManual).then_some(
                    terra_control::TurningConfig {
                        // The exploration look-around is a slow turn in place.
                        allow_in_place: self.navigation.parameters.turning.allow_in_place
                            || output.in_place_turn,
                        ..self.navigation.parameters.turning
                    },
                ),
            );
            self.controller.set_target(VelocityTarget {
                timestamp: now,
                forward: output.twist.linear,
                yaw_rate: output.twist.angular,
            })?;
            self.controller.step(estimate)
        };
        self.sequence += 1;
        let quality = pose_quality::value(self, now);
        let value = json!({"status":output.status,"command_origin":output.command_origin,"pose_quality":quality,"navigation":self.navigation.status(),"outdoor":outdoor_status,"connectivity":{"policy":self.connectivity,"revision":self.connectivity_revision},"goal":output.goal,"proposal":output.proposal,"exploration":output.exploration,"search":output.search,"pending_reports":output.pending_reports,"twist":output.twist,"runtime":{"schema_version":1,"mode":effective_mode,"configured_mode":self.mode,"source_revision":SOURCE_REVISION,"configuration_identity":self.configuration_identity()}});
        self.autonomy_json = value.to_string();
        self.last_snapshot = RuntimeSnapshot { value };
        let mut events = std::mem::take(&mut self.events);
        events.extend(
            output
                .events
                .iter()
                .map(|e| json!({"kind":"decision.event","event":e})),
        );
        Ok(RuntimeOutput {
            autonomy: output,
            motor,
            snapshot: self.last_snapshot.clone(),
            events,
        })
    }
    pub fn snapshot(&self, _now: f64) -> RuntimeSnapshot {
        self.last_snapshot.clone()
    }
}
