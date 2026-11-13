//! Near-miss episodes: observed clearance around the rover body dropping
//! below 0.3 m and recovering above 0.4 m. The runtime adds one decision event
//! per transition, and offline metrics count the starts.
use terra_autonomy::EventKind;

/// `DecisionEvent::reason` of a near-miss transition.
pub const REASON: &str = "observed_clearance";
/// Clearance, metres, below which an episode starts.
const ENTER_M: f64 = 0.3;
/// Clearance, metres, above which an episode ends.
const EXIT_M: f64 = 0.4;

/// Clearance episode detector with entry and exit hysteresis.
#[derive(Debug, Default)]
pub struct NearMiss {
    active: bool,
}
impl NearMiss {
    /// The transition this clearance causes, at most once per episode edge.
    /// Missing, negative and nonfinite clearances return `None` and keep the episode state.
    pub fn update(&mut self, clearance: Option<f64>) -> Option<EventKind> {
        let value = clearance.filter(|v| v.is_finite() && *v >= 0.)?;
        if !self.active && value < ENTER_M {
            self.active = true;
            Some(EventKind::NearMissStart)
        } else if self.active && value > EXIT_M {
            self.active = false;
            Some(EventKind::NearMissEnd)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Cross entry and exit clearance thresholds and verify consecutive low-clearance ticks produce only one episode.
    #[test]
    fn near_miss_is_an_episode_not_every_tick() {
        let mut n = NearMiss::default();
        assert_eq!(n.update(Some(0.2)), Some(EventKind::NearMissStart));
        assert_eq!(n.update(Some(0.21)), None);
        assert_eq!(n.update(Some(0.5)), Some(EventKind::NearMissEnd));
        assert_eq!(n.update(None), None);
    }
}
