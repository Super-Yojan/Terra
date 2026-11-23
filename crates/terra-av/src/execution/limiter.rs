//! Acceleration limit: the target twist may speed up only as fast as the
//! vehicle's limits allow, so a controller (or a swap of controllers while
//! moving) cannot make the target jump.
//!
//! Slowing down is never held back. A lower speed is always safe to ask
//! for at once, and a released stick or a "stop" from a controller must
//! not be stretched out by a ramp.
use crate::vehicle::{Limits, Twist};

/// The twist to command now, given the twist commanded `dt` seconds ago
/// (`previous`) and the one wanted now (`wanted`).
pub fn limit_acceleration(previous: Twist, wanted: Twist, limits: &Limits, dt: f64) -> Twist {
    Twist {
        v: limit_axis(previous.v, wanted.v, limits.max_accel * dt),
        w: limit_axis(previous.w, wanted.w, limits.max_yaw_accel * dt),
    }
}

/// One axis (`v` or `w`): go from `previous` toward `wanted`, where moving
/// away from zero is limited to `max_increase` and moving toward zero is not.
///
/// ```text
/// previous  wanted   result
///   0.5      0.2      0.2                    slower, same direction: at once
///   0.2      0.5      0.2 + max_increase     faster: ramp (never past 0.5)
///   0.5     −0.5      0.0 − max_increase     reversal: stop at once, then ramp
/// ```
fn limit_axis(previous: f64, wanted: f64, max_increase: f64) -> f64 {
    // 1. Brake for free: as far toward zero as `wanted` asks.
    let same_direction = previous * wanted > 0.0;
    let braked = if !same_direction {
        0.0
    } else if wanted.abs() < previous.abs() {
        wanted
    } else {
        previous
    };
    // 2. From there, speed up toward `wanted` by at most `max_increase`.
    braked + (wanted - braked).clamp(-max_increase, max_increase)
}
