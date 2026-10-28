//! Rust-owned 100 Hz control tick for a phone app.
//!
//! The app pushes sensors into the [`MobileController`] from its own threads
//! and reports link facts; this loop owns the clock, steps the controller,
//! asks the [`MotorGate`] for a command and keeps it for the Bluetooth writer
//! to take. The app reads [`DriveLoopSnapshot`] for its UI. Nothing here
//! calls back into Kotlin or Swift.
use crate::gate::{AutonomyBrief, DriveSource, LinkFacts, MotorCommand, MotorGate, tick_plan};
use crate::{ControlOutput, ControllerError, MobileController, TwistSetpoint};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const PERIOD: Duration = Duration::from_millis(10);
/// Behind by more than this, the loop drops the missed ticks instead of bursting.
const MAX_LAG: Duration = Duration::from_millis(50);

/// What the app's UI reads at its own rate.
#[derive(Clone, Debug, uniffi::Record)]
pub struct DriveLoopSnapshot {
    /// The tick thread is running.
    pub running: bool,
    /// Ticks stepped since start.
    pub tick: u64,
    /// Latest controller output.
    pub output: Option<ControlOutput>,
    /// Latest autonomy brief.
    pub brief: AutonomyBrief,
    /// `connected`, `disconnected` or `failed` (see `dashboard_status`).
    pub dashboard: String,
    /// Router failures handled with `handle_dashboard_link_loss` since start.
    pub link_losses: u32,
    /// Error that stopped the loop.
    pub error: Option<String>,
    /// First motor stop cause recorded by the gate.
    pub stop_reason: Option<String>,
    /// The gate's e-stop latch is set.
    pub emergency_stopped: bool,
}

struct Clock {
    base_s: f64,
    base: Instant,
}

impl Clock {
    fn now(&self) -> f64 {
        self.base_s + self.base.elapsed().as_secs_f64()
    }
}

#[derive(Default)]
struct LoopState {
    tick: u64,
    ble_active: bool,
    ble_ready: bool,
    feedback: bool,
    feedback_compatible: bool,
    tracked: bool,
    /// Phone source: m/s and rad/s. Bluetooth manual: normalized stick.
    local: (f64, f64),
    last_local_sent: Option<(f64, f64)>,
    latest_drive: Option<(String, f64)>,
    pending_stop: Option<String>,
    output: Option<ControlOutput>,
    brief: AutonomyBrief,
    dashboard: String,
    link_losses: u32,
    error: Option<String>,
}

struct Shared {
    controller: Arc<MobileController>,
    gate: Arc<MotorGate>,
    source: DriveSource,
    clock: Mutex<Clock>,
    state: Mutex<LoopState>,
    running: AtomicBool,
}

/// The control tick for one drive session. Create, [`Self::start`], and
/// [`Self::stop`] when the session ends; dropping it also stops the thread.
#[derive(uniffi::Object)]
pub struct PhoneDriveLoop {
    shared: Arc<Shared>,
    thread: Mutex<Option<JoinHandle<()>>>,
}

#[uniffi::export]
impl PhoneDriveLoop {
    #[uniffi::constructor]
    /// `now_s` is the current time on the app's sensor clock (Android:
    /// `elapsedRealtimeNanos` seconds). Ticks advance it with a monotonic clock.
    pub fn new(
        controller: Arc<MobileController>,
        gate: Arc<MotorGate>,
        source: DriveSource,
        now_s: f64,
    ) -> Arc<Self> {
        Arc::new(Self {
            shared: Arc::new(Shared {
                controller,
                gate,
                source,
                clock: Mutex::new(Clock {
                    base_s: now_s,
                    base: Instant::now(),
                }),
                state: Mutex::new(LoopState {
                    dashboard: "disconnected".into(),
                    brief: AutonomyBrief {
                        pose_degraded: true,
                        ..Default::default()
                    },
                    ..Default::default()
                }),
                running: AtomicBool::new(false),
            }),
            thread: Mutex::new(None),
        })
    }

    /// Start the 100 Hz thread. A second call does nothing.
    pub fn start(&self) -> Result<(), ControllerError> {
        let mut thread = self.thread.lock().map_err(|_| ControllerError::Internal)?;
        if thread.is_some() {
            return Ok(());
        }
        self.shared.running.store(true, Ordering::SeqCst);
        let shared = self.shared.clone();
        *thread = Some(
            std::thread::Builder::new()
                .name("terra.tick".into())
                .spawn(move || run(&shared))
                .map_err(|e| ControllerError::InvalidInput {
                    message: e.to_string(),
                })?,
        );
        Ok(())
    }

    /// Stop and join the thread. Pending drive frames are dropped.
    pub fn stop(&self) {
        self.shared.running.store(false, Ordering::SeqCst);
        if let Some(handle) = self.thread.lock().ok().and_then(|mut t| t.take()) {
            let _ = handle.join();
        }
        if let Ok(mut s) = self.shared.state.lock() {
            s.latest_drive = None;
        }
    }

    /// Re-pair the loop clock with the sensor clock, e.g. after the app resumes.
    pub fn resync_clock(&self, now_s: f64) {
        if let Ok(mut clock) = self.shared.clock.lock() {
            *clock = Clock {
                base_s: now_s,
                base: Instant::now(),
            };
        }
    }

    /// Current time on the loop clock (the sensor clock), seconds.
    pub fn now(&self) -> f64 {
        self.shared.clock.lock().map(|c| c.now()).unwrap_or(0.0)
    }

    /// What the Bluetooth link reports. The router state comes from the controller.
    pub fn set_link(
        &self,
        ble_active: bool,
        ble_ready: bool,
        feedback: bool,
        feedback_compatible: bool,
    ) {
        if let Ok(mut s) = self.shared.state.lock() {
            s.ble_active = ble_active;
            s.ble_ready = ble_ready;
            s.feedback = feedback;
            s.feedback_compatible = feedback_compatible;
        }
    }

    /// Whether camera tracking is normal on the latest frame.
    pub fn set_tracking(&self, tracked: bool) {
        if let Ok(mut s) = self.shared.state.lock() {
            s.tracked = tracked;
        }
    }

    /// The phone's own stick. Phone source: forward m/s and yaw rate rad/s.
    /// Bluetooth manual source: normalized forward and turn in `[-1, 1]`.
    /// Ignored while an attached fleet owns velocity intent.
    pub fn set_local_input(&self, forward: f64, turn: f64) {
        if !forward.is_finite() || !turn.is_finite() {
            return;
        }
        if let Ok(mut s) = self.shared.state.lock() {
            s.local = (forward, turn);
        }
    }

    /// The newest motor command for the Bluetooth writer. A stop is returned
    /// once. A drive frame older than `max_age_s` is not returned.
    pub fn take_motor_command(&self, max_age_s: f64) -> MotorCommand {
        let now = self.now();
        let Ok(mut s) = self.shared.state.lock() else {
            return MotorCommand::Stop {
                reason: "Control loop lock failed".into(),
            };
        };
        if let Some(reason) = s.pending_stop.take() {
            return MotorCommand::Stop { reason };
        }
        match &s.latest_drive {
            Some((values, at)) if now - at <= max_age_s => MotorCommand::Drive {
                values_json: values.clone(),
            },
            _ => MotorCommand::Idle,
        }
    }

    /// State for the UI.
    pub fn snapshot(&self) -> DriveLoopSnapshot {
        let running = self.shared.running.load(Ordering::SeqCst);
        let stop_reason = self.shared.gate.stop_reason().ok().flatten();
        let emergency_stopped = self.shared.gate.emergency_stopped().unwrap_or(true);
        let Ok(s) = self.shared.state.lock() else {
            return DriveLoopSnapshot {
                running,
                tick: 0,
                output: None,
                brief: AutonomyBrief::default(),
                dashboard: "failed".into(),
                link_losses: 0,
                error: Some("Control loop lock failed".into()),
                stop_reason,
                emergency_stopped,
            };
        };
        DriveLoopSnapshot {
            running,
            tick: s.tick,
            output: s.output.clone(),
            brief: s.brief.clone(),
            dashboard: s.dashboard.clone(),
            link_losses: s.link_losses,
            error: s.error.clone(),
            stop_reason,
            emergency_stopped,
        }
    }

    /// Run one tick at `now` without the thread. For tests and stepped simulation.
    pub fn step_once(&self, now: f64) -> Result<MotorCommand, ControllerError> {
        tick(&self.shared, now)
    }
}

impl Drop for PhoneDriveLoop {
    fn drop(&mut self) {
        self.stop();
    }
}

fn run(shared: &Shared) {
    let mut next = Instant::now();
    while shared.running.load(Ordering::SeqCst) {
        let now = shared.clock.lock().map(|c| c.now()).unwrap_or(f64::NAN);
        if let Err(error) = tick(shared, now) {
            if let Ok(mut s) = shared.state.lock() {
                s.error = Some(error.to_string());
                s.latest_drive = None;
                s.pending_stop = Some(format!("Control loop stopped: {error}"));
            }
            shared.running.store(false, Ordering::SeqCst);
            return;
        }
        next += PERIOD;
        let current = Instant::now();
        if next > current {
            std::thread::sleep(next - current);
        } else if current - next > MAX_LAG {
            next = current;
        }
    }
}

fn tick(shared: &Shared, now: f64) -> Result<MotorCommand, ControllerError> {
    let controller = &shared.controller;
    let mut s = shared.state.lock().map_err(|_| ControllerError::Internal)?;
    let dashboard = controller.dashboard_status();
    let fleet = dashboard == "connected";
    match shared.source {
        DriveSource::BluetoothManual => {
            controller.set_actuation_mode(false)?;
            // Without a fleet session this is the phone's own debug stick.
            if !fleet {
                controller.set_manual_input(s.local.0, s.local.1, now)?;
            }
        }
        DriveSource::Phone | DriveSource::Simulation => {
            // An attached fleet owns velocity intent; an idle phone stick must
            // not overwrite it. A held stick, and its return to neutral, is sent.
            if !(s.ble_active && fleet)
                && (s.local != (0.0, 0.0) || s.last_local_sent != Some(s.local))
            {
                controller.set_target(TwistSetpoint {
                    timestamp: now,
                    forward: s.local.0,
                    yaw_rate: s.local.1,
                })?;
                s.last_local_sent = Some(s.local);
            }
            controller.set_actuation_mode(!(s.ble_active && !s.feedback))?;
        }
    }
    let output = controller.step(now)?;
    let plan = tick_plan(s.tick);
    if plan.publish && dashboard == "failed" {
        controller.handle_dashboard_link_loss()?;
        s.link_losses += 1;
        s.local = (0.0, 0.0);
    }
    let brief = controller.autonomy_brief_typed();
    if shared.source == DriveSource::BluetoothManual
        && brief.router_emergency_stop
        && !shared.gate.emergency_stopped()?
    {
        // Sensor-free manual drive has no arbiter between router and motors.
        shared.gate.emergency_stop(now)?;
        s.local = (0.0, 0.0);
    }
    let facts = LinkFacts {
        ble_active: s.ble_active,
        ble_ready: s.ble_ready,
        feedback: s.feedback,
        feedback_compatible: s.feedback_compatible,
        dashboard_connected: fleet,
        phone_tracked: s.tracked,
    };
    let command = shared.gate.decide(
        s.tick,
        now,
        shared.source,
        facts,
        brief.clone(),
        output.clone(),
        controller.layout_supports_vio_control(),
    )?;
    match &command {
        MotorCommand::Drive { values_json } => s.latest_drive = Some((values_json.clone(), now)),
        MotorCommand::Stop { reason } => {
            s.latest_drive = None;
            s.pending_stop = Some(reason.clone());
            s.local = (0.0, 0.0);
        }
        MotorCommand::Idle if plan.route => s.latest_drive = None,
        MotorCommand::Idle => {}
    }
    s.output = Some(output);
    s.brief = brief;
    s.dashboard = dashboard;
    s.tick += 1;
    Ok(command)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::default_control_settings;

    const PROFILES: &str = r#"{"schema_version":1,"revision":1,"actuators":[
        {"id":0,"kind":"dc_motor","limits":{"min":-1.0,"max":1.0},"route":{"type":"left_effort"},"safe":{"type":"zero"}},
        {"id":1,"kind":"dc_motor","limits":{"min":-1.0,"max":1.0},"route":{"type":"right_effort"},"safe":{"type":"zero"}}]}"#;

    fn manual_loop() -> Arc<PhoneDriveLoop> {
        let controller = MobileController::new(default_control_settings()).unwrap();
        controller.set_actuator_layout(PROFILES.into()).unwrap();
        PhoneDriveLoop::new(
            controller,
            MotorGate::new(),
            DriveSource::BluetoothManual,
            100.0,
        )
    }

    fn values(command: &MotorCommand) -> Vec<f64> {
        let MotorCommand::Drive { values_json } = command else {
            panic!("expected drive, got {command:?}");
        };
        let rows: Vec<serde_json::Value> = serde_json::from_str(values_json).unwrap();
        rows.iter().map(|r| r["value"].as_f64().unwrap()).collect()
    }

    #[test]
    /// The phone's debug stick reaches the motors through the Rust frame,
    /// only on route ticks and only with a ready link.
    fn manual_stick_drives_through_the_gate() {
        let drive = manual_loop();
        assert_eq!(drive.step_once(100.0).unwrap(), MotorCommand::Idle);
        drive.set_link(true, true, false, true);
        drive.set_local_input(0.5, 0.0);
        let mut commands = Vec::new();
        for i in 1..=10 {
            commands.push(drive.step_once(100.0 + i as f64 * 0.01).unwrap());
        }
        let drives: Vec<_> = commands
            .iter()
            .filter(|c| matches!(c, MotorCommand::Drive { .. }))
            .collect();
        assert_eq!(drives.len(), 2);
        let last = values(drives[1]);
        assert!(last[0] > 0.0 && (last[0] - last[1]).abs() < 1e-6);
        let snapshot = drive.snapshot();
        assert_eq!(snapshot.tick, 11);
        assert_eq!(snapshot.dashboard, "disconnected");
        assert!(snapshot.output.is_some() && snapshot.error.is_none());
    }

    #[test]
    /// A stop is handed out once, and an old frame is not handed out at all.
    fn take_returns_stops_once_and_drops_stale_frames() {
        let drive = manual_loop();
        {
            let mut s = drive.shared.state.lock().unwrap();
            s.latest_drive = Some(("[]".into(), 100.0));
            s.pending_stop = Some("why".into());
        }
        assert_eq!(
            drive.take_motor_command(1e9),
            MotorCommand::Stop {
                reason: "why".into()
            }
        );
        assert!(matches!(
            drive.take_motor_command(1e9),
            MotorCommand::Drive { .. }
        ));
        drive.resync_clock(200.0);
        assert_eq!(drive.take_motor_command(0.15), MotorCommand::Idle);
    }

    #[test]
    /// Without a confirmed mount the phone source holds motors and says why once.
    fn phone_source_holds_for_the_mount() {
        let controller = MobileController::new(default_control_settings()).unwrap();
        controller.set_actuator_layout(PROFILES.into()).unwrap();
        let gate = MotorGate::new();
        let drive = PhoneDriveLoop::new(controller, gate.clone(), DriveSource::Phone, 0.0);
        drive.set_link(true, true, false, true);
        drive.set_tracking(true);
        let first = drive.step_once(0.0).unwrap();
        assert!(matches!(first, MotorCommand::Stop { .. }), "{first:?}");
        assert_eq!(
            drive.take_motor_command(1.0),
            MotorCommand::Stop {
                reason: crate::gate::MOUNT_HOLD_REASON.into()
            }
        );
        for i in 1..10 {
            assert_eq!(drive.step_once(i as f64 * 0.01).unwrap(), MotorCommand::Idle);
        }
        gate.set_mount_confirmed(true).unwrap();
        let next = drive.step_once(0.1).unwrap();
        assert!(matches!(next, MotorCommand::Drive { .. }), "{next:?}");
    }

    #[test]
    /// The thread ticks near 100 Hz on its own and stops when asked.
    fn thread_ticks_and_stops() {
        let drive = manual_loop();
        drive.start().unwrap();
        std::thread::sleep(Duration::from_millis(120));
        drive.stop();
        let snapshot = drive.snapshot();
        assert!(!snapshot.running);
        assert!(
            (5..=20).contains(&snapshot.tick),
            "ticked {} times",
            snapshot.tick
        );
        let after = snapshot.tick;
        std::thread::sleep(Duration::from_millis(30));
        assert_eq!(drive.snapshot().tick, after);
    }
}
