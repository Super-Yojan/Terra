//! Connectivity affects supervision and presentation, never onboard sensor admission.
use serde::{Deserialize, Serialize};
pub const GPS_DISPLAY_FRESHNESS_SECONDS: f64 = 3.0;
pub const FLEET_DISPLAY_STALE_SECONDS: f64 = 90.0;
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum RouterLossPolicy {
    #[default]
    ContinueAcceptedMission,
    Pause,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default, deny_unknown_fields)]
pub struct ConnectivityPolicySettings {
    pub gps_display_freshness_seconds: f64,
    pub fleet_display_stale_seconds: f64,
    pub autonomous_router_loss: RouterLossPolicy,
}
impl Default for ConnectivityPolicySettings {
    fn default() -> Self {
        Self {
            gps_display_freshness_seconds: GPS_DISPLAY_FRESHNESS_SECONDS,
            fleet_display_stale_seconds: FLEET_DISPLAY_STALE_SECONDS,
            autonomous_router_loss: RouterLossPolicy::ContinueAcceptedMission,
        }
    }
}
impl ConnectivityPolicySettings {
    pub fn validate(&self) -> Result<(), &'static str> {
        if [
            self.gps_display_freshness_seconds,
            self.fleet_display_stale_seconds,
        ]
        .iter()
        .any(|v| !v.is_finite() || *v <= 0.0)
        {
            Err("display freshness must be finite and positive")
        } else {
            Ok(())
        }
    }
}
