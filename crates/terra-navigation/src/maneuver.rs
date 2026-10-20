//! Bounded geometric multi-point proposals. Progress is measured, never timed.
use super::*;
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct ManeuverSegment {
    pub start: Pose,
    pub end: Pose,
    pub velocity: Twist,
    pub direction: String,
}
fn wrap(x: f64) -> f64 {
    (x + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}
impl LocalPlanner {
    /// Propose checked arcs to reduce heading error. Every execution tick must
    /// recheck its actual slew-limited command against fresh occupancy.
    pub fn heading_maneuver(
        &self,
        map: &MapSnapshot,
        pose: Pose,
        target_yaw: f64,
        c: &terra_control::TurningConfig,
    ) -> Option<Vec<ManeuverSegment>> {
        if !target_yaw.is_finite() || c.validate().is_err() || !self.valid() || !pose.finite() {
            return None;
        }
        let error = wrap(target_yaw - pose.yaw);
        if error.abs() < 0.08 {
            return None;
        }
        // A forward-only arc remains an option even for large heading errors.
        let patterns: &[&[f64]] = &[&[1.], &[-1., 1., -1.], &[1., -1., 1.]];
        let mut best: Option<(f64, Vec<ManeuverSegment>)> = None;
        for pattern in patterns {
            let mut at = pose;
            let mut segments = Vec::new();
            let mut cost = 0.;
            for sign in *pattern {
                let v = sign
                    * self
                        .config
                        .max_reverse
                        .min(self.config.max_linear)
                        .min(0.15);
                let w = error.signum()
                    * c.max_yaw_rate
                        .min(v.abs() / c.min_turn_radius)
                        .min(error.abs() / pattern.len() as f64 / self.config.horizon);
                if w.abs() < 1e-6 {
                    break;
                }
                let velocity = Twist {
                    linear: v,
                    angular: w,
                };
                if !self.admissible(map, at, velocity) {
                    break;
                }
                let angle = w * self.config.horizon;
                let end = Pose {
                    x: at.x + v / w * ((at.yaw + angle).sin() - at.yaw.sin()),
                    y: at.y - v / w * ((at.yaw + angle).cos() - at.yaw.cos()),
                    yaw: wrap(at.yaw + angle),
                };
                segments.push(ManeuverSegment {
                    start: at,
                    end,
                    velocity,
                    direction: direction(v).into(),
                });
                cost += self.config.reverse_penalty * (-v).max(0.) * self.config.horizon;
                at = end;
            }
            if segments.len() != pattern.len() {
                continue;
            }
            let progress = error.abs() - wrap(target_yaw - at.yaw).abs();
            if progress <= 0.01 {
                continue;
            }
            cost += wrap(target_yaw - at.yaw).abs()
                + 0.5 * (at.x - pose.x).hypot(at.y - pose.y)
                + 0.01 * segments.len() as f64;
            if best.as_ref().is_none_or(|b| cost < b.0) {
                best = Some((cost, segments));
            }
        }
        best.map(|b| b.1)
    }
}
