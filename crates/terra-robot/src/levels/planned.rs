//! L1–L5 until each moves into its own module: the arbiter chooses the
//! motion (assisted teleop, waypoint routes, exploration, search, task) and
//! already holds on stale pose, map or operator input.
use crate::level::{LevelModule, Motion, Proposal, Tick};

pub struct Planned;

impl LevelModule for Planned {
    fn propose(&mut self, tick: &Tick) -> Proposal {
        Proposal {
            motion: Motion::Planned,
            input_t: tick.sensors.now,
        }
    }
}
