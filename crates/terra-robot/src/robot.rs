//! [`Robot`]: the one object the phone app talks to. Swift owns the
//! loop and the sensors and calls [`Robot::update`] once per control tick:
//!
//! ```text
//! SensorData ─▶ apply sensors and ARGOS requests ─▶ Core::plan (authority, safety, arbiter)
//!            ─▶ level module proposes ─▶ wheels ─▶ SafetyGate ─▶ actuator frame ─▶ MotorCommand
//! ```
//!
//! The app sends what comes back to the motor board and nothing else.
use crate::level::{LevelModule, Motion, Proposal, Tick, module_for};
use crate::safety::{GateInput, SafetyGate, Verdict};
use crate::sensors::SensorData;
use crate::{
    ActuationMode, Core, CoreConfig, CoreOutput, Event, RobotError, actuators::ActiveLayout,
    capture, diagnostics::Diagnostics, publication,
};
use serde_json::json;
use std::sync::Arc;
use terra_autonomy::{Level, Source};
use terra_types::{MotorOutput, StopReason, telemetry};

/// Every fifth 10 ms tick: motor frames go out at 20 Hz.
const ROUTE_EVERY: u64 = 5;

/// What the phone does with the motor link this tick.
#[derive(Clone, Debug, PartialEq)]
pub enum MotorAction {
    /// Send nothing; the rover's own lease keeps it safe.
    Idle,
    /// Write this actuator frame (`[{"id","value"}]` JSON).
    Drive { frame_json: String },
    /// Stop output now. `reason` is for the phone UI.
    Stop { reason: String },
}

/// One tick's answer: what to send, and the efforts and estimates behind it
/// for the phone UI.
#[derive(Clone, Debug)]
pub struct MotorCommand {
    pub action: MotorAction,
    pub motor: MotorOutput,
    /// Dump-gate servo position chosen by the bucket dock, already in the
    /// frame; `None` without a gate servo or when the layout refuses it.
    pub gate_servo: Option<(u8, f64)>,
}

/// Sensor timestamps already applied, so a repeated snapshot adds nothing.
#[derive(Default)]
struct Seen {
    imu: Option<f64>,
    vio: Option<f64>,
    gps: Option<f64>,
    heading: Option<f64>,
    course: Option<f64>,
    stick: Option<f64>,
}

fn newer(seen: &mut Option<f64>, t: f64) -> bool {
    if seen.is_some_and(|s| t <= s) {
        return false;
    }
    *seen = Some(t);
    true
}

pub struct Robot {
    pub(crate) core: Core,
    gate: SafetyGate,
    level: Level,
    active: Box<dyn LevelModule>,
    tick: u64,
    seen: Seen,
    /// Sensor samples the core refused (out of order, invalid values).
    rejected: u64,
    pub(crate) layout: Option<ActiveLayout>,
    /// Why the layout last refused the dock's gate servo target, journaled once.
    gate_rejected: Option<String>,
    pub(crate) dashboard: Option<terra_transport::ControlPlane>,
    pub(crate) recorder: Option<terra_experiment::RunRecorder>,
    pub(crate) map_gate: publication::MapGate,
    /// Running point capture, if the dashboard asked for one.
    pub(crate) capture: Option<capture::Capture>,
    /// Object-map revision and body for the `objects` topic.
    objects_published: Option<(u64, String)>,
    pub(crate) last_tick: f64,
    pub(crate) diagnostics: Arc<Diagnostics>,
}

impl Robot {
    pub fn new(config: CoreConfig, run_id: String) -> Result<Self, RobotError> {
        Ok(Self {
            core: Core::new(config, run_id)?,
            gate: SafetyGate::default(),
            level: Level::Teleop,
            active: module_for(Level::Teleop),
            tick: 0,
            seen: Seen::default(),
            rejected: 0,
            layout: None,
            gate_rejected: None,
            dashboard: None,
            recorder: None,
            map_gate: Default::default(),
            capture: None,
            objects_published: None,
            last_tick: 0.,
            diagnostics: Arc::default(),
        })
    }

    /// Authority, planning and status state, read-only.
    pub fn core(&self) -> &Core {
        &self.core
    }
    /// For settings and requests the core owns (rover profile, dock, search, task).
    pub fn core_mut(&mut self) -> &mut Core {
        &mut self.core
    }
    pub fn run_id(&self) -> &str {
        self.core.run_id()
    }
    pub fn diagnostics(&self) -> Arc<Diagnostics> {
        self.diagnostics.clone()
    }
    /// Time of the latest tick.
    pub fn last_tick(&self) -> f64 {
        self.last_tick
    }
    /// The level whose module is active.
    pub fn level(&self) -> Level {
        self.level
    }
    /// Sensor samples refused since construction.
    pub fn rejected_samples(&self) -> u64 {
        self.rejected
    }
    /// First motor stop cause since [`Self::clear_stop_reason`].
    pub fn stop_reason(&self) -> Option<&str> {
        self.gate.stop_reason()
    }
    pub fn clear_stop_reason(&mut self) {
        self.gate.clear_stop_reason();
    }

    /// One control tick. See the module documentation.
    pub fn update(&mut self, s: &SensorData) -> Result<MotorCommand, RobotError> {
        let now = s.now;
        self.apply_sensors(s);
        self.service_link(now)?;
        let plan = self.core.plan(now)?;

        let requested = plan.output.status.requested_level;
        let operator = self.core.autonomy().operator_command();
        let tick = Tick {
            sensors: s,
            plan: &plan,
            operator,
        };
        if requested != self.level {
            self.active.on_exit();
            self.active = module_for(requested);
            self.active.on_enter(&tick);
            self.level = requested;
        }
        // A bucket dock drives at any level until it ends.
        let proposal = if plan.output.status.active_source == Source::Dock {
            Proposal {
                motion: Motion::Planned,
                input_t: now,
            }
        } else {
            self.active.propose(&tick)
        };
        let motor = self.drive(&plan, proposal.motion, now)?;
        let verdict = self.gate.check(&GateInput {
            sensors: s,
            plan: &plan,
            motor: &motor,
            input_age: now - proposal.input_t,
            pose_ready: self.core.pose_ready(now),
            layout_supports_vio: self.layout_supports_vio_control(),
            fleet_connected: self.fleet_owns_intent(),
        });
        let motor = if let Verdict::Stop(_) = verdict {
            self.core.release_velocity_loop();
            MotorOutput {
                left: 0.,
                right: 0.,
                ..motor
            }
        } else {
            motor
        };
        let gate_open = plan.output.gate_open;
        let output = self.core.finish(plan, motor, now);
        self.report(&output, now);
        let gate_servo = self.apply_gate_servo(gate_open, now);
        let route = self.tick.is_multiple_of(ROUTE_EVERY);
        self.tick += 1;
        let action = match verdict {
            Verdict::Idle => MotorAction::Idle,
            Verdict::Stop(reason) => MotorAction::Stop { reason },
            Verdict::Drive if !route => MotorAction::Idle,
            Verdict::Drive => match self
                .layout
                .as_ref()
                .and_then(|l| l.frame(output.motor.left, output.motor.right).ok())
            {
                Some(frame_json) => MotorAction::Drive { frame_json },
                None => {
                    let reason = format!(
                        "Actuator routing failed: no frame for left={} right={}",
                        output.motor.left, output.motor.right
                    );
                    self.gate.stop(reason.clone());
                    MotorAction::Stop { reason }
                }
            },
        };
        Ok(MotorCommand {
            action,
            motor: output.motor,
            gate_servo,
        })
    }

    /// Apply every sample newer than the last one seen. A refused sample is
    /// counted and dropped; the freshness checks stop the rover if a sensor
    /// really stops.
    fn apply_sensors(&mut self, s: &SensorData) {
        let now = s.now;
        let mut events = Vec::new();
        if let Some(sample) = s.imu.filter(|x| newer(&mut self.seen.imu, x.timestamp)) {
            events.push(Event::Imu {
                sample,
                received_at: sample.timestamp,
            });
        }
        if let Some(sample) = s.vio.filter(|x| newer(&mut self.seen.vio, x.timestamp)) {
            events.push(Event::Vio {
                sample,
                received_at: sample.timestamp,
            });
        }
        if let Some(sample) = s.gps.filter(|x| newer(&mut self.seen.gps, x.timestamp)) {
            events.push(Event::Gps {
                sample,
                received_at: now,
            });
        }
        if let Some(sample) = s
            .heading
            .filter(|x| newer(&mut self.seen.heading, x.timestamp))
        {
            events.push(Event::Heading {
                sample,
                received_at: now,
            });
        }
        if let Some(sample) = s
            .course
            .filter(|x| newer(&mut self.seen.course, x.timestamp))
        {
            events.push(Event::Course {
                sample,
                received_at: now,
            });
        }
        // An attached fleet link owns velocity intent; the phone stick waits.
        if let Some(stick) = s.stick.filter(|x| newer(&mut self.seen.stick, x.t))
            && !self.fleet_owns_intent()
        {
            events.push(Event::LocalManual {
                forward: stick.forward,
                turn: stick.turn,
                received_at: stick.t,
            });
        }
        for event in events {
            if self.core.ingest(event).is_err() {
                self.rejected += 1;
            }
        }
    }

    /// Wheel efforts for a motion. Only planned and operator-velocity
    /// motions run the velocity loop; the others leave it at rest.
    fn drive(&mut self, plan: &crate::Plan, motion: Motion, now: f64) -> Result<MotorOutput, RobotError> {
        let estimate = self.core.estimate(now);
        let direct = |left, right| MotorOutput {
            left,
            right,
            estimated_forward: estimate.forward,
            estimated_yaw_rate: estimate.yaw_rate,
            target_forward: 0.,
            target_yaw_rate: 0.,
            stop_reason: StopReason::None,
        };
        Ok(match motion {
            Motion::Planned => self.core.track(plan, now)?,
            Motion::Velocity(twist) => self.core.track_operator(twist, now)?,
            Motion::Wheels { left, right } => {
                self.core.release_velocity_loop();
                MotorOutput {
                    target_forward: (left + right) * 0.5 * crate::MANUAL_FULL_FORWARD,
                    target_yaw_rate: (right - left) * 0.5,
                    ..direct(left, right)
                }
            }
            Motion::Stop => {
                self.core.release_velocity_loop();
                direct(0., 0.)
            }
        })
    }

    /// The dock owns the gate: open only on its request, closed otherwise.
    fn apply_gate_servo(&mut self, open: bool, now: f64) -> Option<(u8, f64)> {
        let mut gate = self.core.gate_servo(open);
        if let (Some((id, position)), Some(layout)) = (gate, self.layout.as_mut()) {
            let rejected = layout.set_servo(id, position).err();
            if rejected.is_some() {
                // Not in the actuator frame, so not reported as applied.
                gate = None;
            }
            if rejected != self.gate_rejected {
                if let Some(error) = &rejected {
                    self.diagnostics.event(
                        "actuators",
                        "actuators.gate_rejected",
                        json!({"servo": id, "error": error}),
                        self.core.run_id(),
                        now,
                    );
                }
                self.gate_rejected = rejected;
            }
        }
        gate
    }

    /// Velocity feedback (true) or direct effort for operator levels.
    pub fn set_actuation_mode(&mut self, feedback: bool) -> Result<(), RobotError> {
        self.core.ingest(Event::Mode {
            mode: if feedback {
                ActuationMode::VelocityFeedback
            } else {
                ActuationMode::DirectEffort
            },
            received_at: self.last_tick,
        })
    }

    /// Replace the rover drive profiles that turn efforts into frames. Servo
    /// targets reset to their safe values. An empty string clears the layout;
    /// an invalid one clears it too, so no frame comes from stale profiles.
    pub fn set_actuator_layout(&mut self, profiles_json: &str) -> Result<(), RobotError> {
        self.layout = None;
        if profiles_json.is_empty() {
            return Ok(());
        }
        self.layout = Some(ActiveLayout::parse(profiles_json).map_err(crate::invalid_owned)?);
        Ok(())
    }

    /// Set a normalized servo target, clipped to that servo's limits.
    pub fn set_servo_target(&mut self, id: u8, value: f64) -> Result<(), RobotError> {
        self.layout
            .as_mut()
            .ok_or_else(|| crate::invalid_owned("no actuator layout".into()))?
            .set_servo(id, value)
            .map_err(crate::invalid_owned)
    }

    /// The layout has differential drive routes (L2–L5 need them).
    pub fn layout_supports_vio_control(&self) -> bool {
        self.layout
            .as_ref()
            .is_some_and(ActiveLayout::supports_vio_control)
    }

    /// Decode and apply a local autonomy, safety, goal, decision or search
    /// request from the phone UI.
    pub fn local_request(&mut self, kind: &str, payload: &str, now: f64) -> Result<(), RobotError> {
        let outcome = self.core.local_action(kind, payload.as_bytes(), now);
        self.diagnostics.event(
            "operator",
            if outcome.is_ok() {
                "command.dispatched"
            } else {
                "command.rejected"
            },
            json!({"topic":kind,"payload":serde_json::from_str::<serde_json::Value>(payload).ok(),"error":outcome.as_ref().err().map(|e| e.to_string())}),
            self.core.run_id(),
            now,
        );
        outcome
    }

    /// Start a bounded run recording at `path` and a new run identity.
    /// Resets autonomy, velocity control and run timing; keeps navigation geometry.
    pub fn begin_recording(&mut self, path: &str, run_id: String) -> Result<(), RobotError> {
        if !terra_autonomy::valid_token(&run_id) {
            return Err(crate::invalid_owned("invalid run identity".into()));
        }
        let recorder = terra_experiment::RunRecorder::create(
            std::path::Path::new(path),
            json!({
                "run_id": run_id,
                "platform": "phone",
                "runtime": self.core.metadata(),
                "planner": self.core.navigation().parameters.planner(),
                "collision_observation": "unavailable",
            }),
            8192,
        )
        .map_err(|e| crate::invalid_owned(e.to_string()))?;
        if let Some(mut old) = self.recorder.take() {
            old.close().map_err(|e| crate::invalid_owned(e.to_string()))?;
        }
        self.recorder = Some(recorder);
        self.core.begin_run(run_id)
    }

    /// Close the current recording; succeeds when none is active.
    pub fn end_recording(&mut self) -> Result<(), RobotError> {
        if let Some(mut r) = self.recorder.take() {
            r.close().map_err(|e| crate::invalid_owned(e.to_string()))?;
        }
        Ok(())
    }

    /// Add one line to the run recording, if one is open. A refused line
    /// already marks the run incomplete.
    pub fn record(&self, line: serde_json::Value) {
        if let Some(recorder) = self.recorder.as_ref() {
            let _ = recorder.record(line);
        }
    }

    pub(crate) fn recorder_failed(&self) -> bool {
        self.recorder.as_ref().is_some_and(|r| r.failed())
    }

    /// Reset estimator, velocity loop and autonomy; detach the fleet link and
    /// close the recording. Keeps navigation geometry and the layout.
    pub fn reset(&mut self) -> Result<(), RobotError> {
        self.core.reset()?;
        self.seen = Seen::default();
        self.last_tick = 0.;
        self.dashboard = None;
        self.map_gate = Default::default();
        self.gate.reset();
        self.active.on_exit();
        self.level = Level::Teleop;
        self.active = module_for(Level::Teleop);
        if let Some(mut r) = self.recorder.take() {
            let _ = r.close();
        }
        Ok(())
    }

    /// Record, journal and publish this tick.
    fn report(&mut self, output: &CoreOutput, now: f64) {
        let start = self.core.run_start().unwrap_or(now);
        let run_id = self.core.run_id().to_owned();
        for event in &output.events {
            if let Some(recorder) = self.recorder.as_ref() {
                let _ = recorder.record(json!({"kind":"runtime.event","event":event,"time":now-start,"source_time":now,"run_id":run_id}));
            }
            self.diagnostics.event(
                "runtime",
                event["kind"].as_str().unwrap_or("runtime.event"),
                event.clone(),
                &run_id,
                now,
            );
        }
        self.publish(output, &run_id, start, now);
        for event in &output.autonomy.events {
            self.diagnostics.event(
                "autonomy",
                "decision.event",
                serde_json::to_value(event).unwrap_or_default(),
                &run_id,
                now,
            );
        }
        let estimate = self.core.estimate(now);
        self.diagnostics.tick(self, now, &estimate, &output.status);
        if let Some(r) = self.recorder.as_ref() {
            let mut record =
                crate::ControlTickRecord::new(output, &run_id, self.core.sequence(), now - start);
            record.source_time = Some(now);
            let _ = r.record(record.to_value());
        }
        self.diagnostics.effort(self, now, &output.motor);
    }

    /// Fleet telemetry, objects and the occupancy map for ARGOS.
    fn publish(&mut self, output: &CoreOutput, run_id: &str, start: f64, now: f64) {
        let Some(rover) = self.dashboard.as_ref().map(|d| d.rover_id()) else {
            return;
        };
        let map = self.core.map().cloned();
        let map_publication = map.as_ref().map(|(grid, time, sequence)| {
            if !publication::fresh_map_sample(now, *time) || !self.map_gate.is_newer(&run_id.to_owned(), *sequence)
            {
                return publication::Publication::Skip;
            }
            if !grid.telemetry_valid(run_id) {
                self.map_gate.reject_source(run_id.to_owned(), *sequence);
                return publication::Publication::Skip;
            }
            let content = (
                run_id.to_owned(),
                grid.width,
                grid.height,
                grid.resolution,
                grid.origin_x,
                grid.origin_y,
                grid.occupancy.clone(),
            );
            self.map_gate.observe(
                content,
                run_id.to_owned(),
                *sequence,
                std::time::Instant::now(),
                std::time::Duration::from_millis(200),
            )
        });
        // Telemetry expires after 500 ms, so the cached body is resent every
        // tick and only re-serialized when the object map changes.
        let objects = self.core.objects();
        if self
            .objects_published
            .as_ref()
            .is_none_or(|(revision, _)| *revision != objects.revision)
        {
            self.objects_published = Some((objects.revision, objects.telemetry(rover, run_id).to_string()));
        }
        let recording = self.recorder.as_ref().is_some_and(|r| !r.failed());
        let Some(d) = self.dashboard.as_ref() else {
            return;
        };
        d.publish(
            telemetry::EXPERIMENT_STATUS,
            json!({
                "run_id": run_id,
                "run_elapsed": now - start,
                "schema_version": 1,
                "utc": std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default().as_secs_f64(),
                "recording": recording,
                "runtime": output.status["runtime"],
            })
            .to_string(),
        );
        for (topic, body) in output.telemetry() {
            d.publish(topic, body);
        }
        d.publish_batch(telemetry::SEARCH_REPORT, output.search_reports());
        if let Some((_, objects)) = &self.objects_published {
            d.publish(telemetry::OBJECTS, objects.clone());
        }
        let mut map_full_rejected = false;
        if let Some((grid, _, sequence)) = map.as_ref() {
            match map_publication {
                Some(publication::Publication::Full) => {
                    map_full_rejected = !d.publish_visualization(
                        telemetry::MAP_OCCUPANCY,
                        grid.telemetry(d.rover_id(), run_id, *sequence).to_string(),
                    );
                }
                Some(publication::Publication::Heartbeat(sequence)) => d.publish(
                    telemetry::MAP_OCCUPANCY_HEARTBEAT,
                    terra_mapping::MapSnapshot::heartbeat(d.rover_id(), run_id, sequence)
                        .to_string(),
                ),
                _ => {}
            }
        }
        self.map_gate.full_enqueued(!map_full_rejected);
    }
}
