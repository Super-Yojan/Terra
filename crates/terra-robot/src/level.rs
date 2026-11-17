//! The contract every autonomy level implements, and the one place a level
//! name turns into code.
//!
//! [`crate::Robot::update`] asks the active module for a [`Proposal`] each
//! tick, then the safety checks decide whether it may drive. A level switch
//! calls `on_exit` on the old module and `on_enter` on a fresh one, so no
//! level state survives a switch.
use crate::{Plan, SensorData};
use terra_autonomy::Level;
use terra_navigation::Twist;

/// What a level wants the wheels to do this tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Motion {
    Stop,
    /// Normalized wheel efforts in `[-1, 1]`, sent as they are.
    Wheels { left: f64, right: f64 },
    /// An operator twist (m/s, rad/s) tracked by the velocity loop.
    Velocity(Twist),
    /// The arbiter's allowed motion this tick, through the velocity loop
    /// with the rover's turning limits (L1–L5 and the bucket dock).
    Planned,
}

/// A level's motion and the time of the input it came from. The safety
/// gate stops a proposal whose input is older than the lease.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Proposal {
    pub motion: Motion,
    pub input_t: f64,
}

impl Proposal {
    pub fn stop(now: f64) -> Self {
        Self {
            motion: Motion::Stop,
            input_t: now,
        }
    }
}

/// What a level module sees this tick.
pub struct Tick<'a> {
    pub sensors: &'a SensorData,
    /// Authority, safety and the arbiter's decision for this tick.
    pub plan: &'a Plan,
    /// Latest accepted operator command (local stick or ARGOS teleop), m/s
    /// and rad/s, with its receipt time.
    pub operator: Option<(Twist, f64)>,
}

/// The minimum every level implements.
pub trait LevelModule: Send {
    fn on_enter(&mut self, _tick: &Tick) {}
    fn on_exit(&mut self) {}
    fn propose(&mut self, tick: &Tick) -> Proposal;
}

/// The only `match` on a level: a fresh module for it.
pub fn module_for(level: Level) -> Box<dyn LevelModule> {
    match level {
        Level::Teleop => Box::new(crate::levels::Teleop),
        _ => Box::new(crate::levels::Planned),
    }
}
