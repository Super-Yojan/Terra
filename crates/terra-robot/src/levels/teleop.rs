//! L0: the operator's stick drives the wheels. No pose or map needed.
use crate::level::{LevelModule, Motion, Proposal, Tick};
use crate::{ActuationMode, MANUAL_FULL_FORWARD};
use terra_autonomy::CommandOrigin;

pub struct Teleop;

impl LevelModule for Teleop {
    fn propose(&mut self, tick: &Tick) -> Proposal {
        // No input inside the lease means hold still, not "expired input".
        let Some((twist, t)) = tick
            .operator
            .filter(|(_, t)| tick.sensors.now - t <= crate::safety::LEASE)
        else {
            return Proposal::stop(tick.sensors.now);
        };
        // ARGOS teleop keeps the rover's managed turning limits, which the
        // arbiter applies; only the phone's own stick drives unfiltered.
        if tick.plan.output.command_origin != CommandOrigin::LocalManual {
            return Proposal {
                motion: Motion::Planned,
                input_t: t,
            };
        }
        let motion = match tick.plan.mode {
            ActuationMode::VelocityFeedback => Motion::Velocity(twist),
            ActuationMode::DirectEffort => {
                let (left, right) = mix(twist.linear / MANUAL_FULL_FORWARD, twist.angular);
                Motion::Wheels { left, right }
            }
        };
        Proposal { motion, input_t: t }
    }
}

/// Arcade mix of a normalized stick into wheel efforts, scaled so neither
/// wheel exceeds 1. Positive `turn` is left.
pub fn mix(forward: f64, turn: f64) -> (f64, f64) {
    let scale = (forward - turn).abs().max((forward + turn).abs()).max(1.);
    ((forward - turn) / scale, (forward + turn) / scale)
}

#[cfg(test)]
mod tests {
    use super::mix;
    #[test]
    fn mix_keeps_wheels_in_range() {
        assert_eq!(mix(1., 0.), (1., 1.));
        assert_eq!(mix(0., 1.), (-1., 1.));
        assert_eq!(mix(1., 1.), (0., 1.));
        assert_eq!(mix(0.5, -0.25), (0.75, 0.25));
    }
}
