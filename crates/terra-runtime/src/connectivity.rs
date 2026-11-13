//! Router-loss policy and link state. Connectivity affects supervision and
//! presentation, never onboard sensor admission.
use super::*;

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

/// The configured policy, its revision, and whether the current link loss
/// has already been handled.
#[derive(Default)]
pub(crate) struct Connectivity {
    pub(crate) policy: ConnectivityPolicySettings,
    pub(crate) revision: u64,
    pub(crate) loss_handled: bool,
}

impl Connectivity {
    fn pauses(&self) -> bool {
        self.policy.autonomous_router_loss == RouterLossPolicy::Pause
    }
    pub(crate) fn status(&self) -> Value {
        json!({"policy": self.policy, "revision": self.revision})
    }
}

impl Runtime {
    pub fn connectivity(&self) -> &ConnectivityPolicySettings {
        &self.connectivity.policy
    }
    pub fn connectivity_revision(&self) -> u64 {
        self.connectivity.revision
    }
    pub fn set_connectivity_policy(
        &mut self,
        policy: ConnectivityPolicySettings,
    ) -> Result<(), RuntimeError> {
        policy.validate().map_err(invalid)?;
        self.connectivity.policy = policy;
        if self.connectivity.loss_handled && self.connectivity.pauses() {
            self.pause_for_router_loss();
        }
        self.connectivity.revision = self.connectivity.revision.saturating_add(1);
        self.refresh_identity();
        self.events.push(json!({"kind":"connectivity.policy_changed","policy":self.connectivity.policy,"revision":self.connectivity.revision}));
        Ok(())
    }
    fn pause_for_router_loss(&mut self) {
        self.outdoor.pause("router_loss_paused");
        self.autonomy.pause_for_router_loss();
        self.controller.reset();
    }
    /// The fleet link came up or went down (see [`RuntimeEvent::Link`]).
    pub(crate) fn link(&mut self, connected: bool) {
        if connected {
            if self.connectivity.loss_handled {
                self.autonomy.invalidate_remote_authority();
            }
            self.connectivity.loss_handled = false;
        } else if !self.connectivity.loss_handled {
            self.autonomy.invalidate_remote_authority();
            self.queue.clear();
            if self.connectivity.pauses() {
                self.pause_for_router_loss();
            }
            self.connectivity.loss_handled = true;
        }
        self.events
            .push(json!({"kind":if connected{"connection.established"}else{"connection.lost"}}));
    }
    /// Handle an unexpected link failure once: apply the router-loss policy
    /// and return what was decided, for the host's journal. `None` when this
    /// loss was already handled.
    pub fn lose_link(&mut self) -> Option<Value> {
        if self.connectivity.loss_handled {
            return None;
        }
        let mission = self.accepted_onboard_mission();
        let goal = self.brief.as_ref().map(|b| b.goal.clone());
        self.link(false);
        Some(json!({
            "policy": self.connectivity.policy, "policy_revision": self.connectivity.revision,
            "accepted_mission_continues": mission,
            "mission": goal,
        }))
    }
    /// Whether the router-loss policy keeps the current mission running:
    /// an accepted onboard goal, or an outdoor mission that is planning.
    pub fn accepted_onboard_mission(&self) -> bool {
        self.connectivity.policy.autonomous_router_loss == RouterLossPolicy::ContinueAcceptedMission
            && if self.outdoor.selected() {
                self.outdoor.active()
                    && self.outdoor.phase() == terra_autonomy::outdoor::OutdoorPhase::Planning
                    && self.autonomy.outdoor_authority()
            } else {
                self.autonomy
                    .accepted_onboard_mission(self.pose.unwrap_or_default().0)
            }
    }
}
