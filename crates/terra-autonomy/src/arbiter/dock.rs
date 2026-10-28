//! Bucket docking inside the arbiter: requests, fixes, and the motion override.
use super::*;

impl AutonomyArbiter {
    /// Replace the docking geometry and tolerances, for example from a rover profile.
    /// Returns false for an invalid config, keeping the old one.
    pub fn configure_dock(&mut self, config: DockConfig) -> bool {
        if !config.valid() {
            return false;
        }
        self.dock_config = config;
        true
    }

    pub fn dock_config(&self) -> &DockConfig {
        &self.dock_config
    }

    /// L4 and L5 dump on their own once aligned; lower levels ask the operator.
    pub fn dump_approval(&self) -> DumpApproval {
        if matches!(
            self.level,
            Level::Explore | Level::TargetSearch | Level::Task
        ) {
            DumpApproval::Auto
        } else {
            DumpApproval::Operator
        }
    }

    pub fn dock_active(&self) -> bool {
        self.dock.as_ref().is_some_and(Dock::active)
    }

    pub fn dock_status(&self, now: f64) -> Option<DockStatus> {
        self.dock.as_ref().map(|d| d.status(now))
    }

    /// Apply a `dock` request. `start` names a bucket the host has already
    /// looked up in the object map (`target`); the host passes `None` when
    /// the id is unknown, not confirmed, or not a bucket.
    pub fn apply_dock_request(
        &mut self,
        r: DockRequest,
        target: Option<DockTarget>,
        pose: Pose,
        now: f64,
    ) -> bool {
        let fp = format!("dock:{:?}:{:?}:{:?}", r.action, r.object_id, r.dock_id);
        if !valid_token(&r.token) || self.duplicate(&r.token, &fp) {
            return false;
        }
        if r.run_id.as_ref().is_some_and(|run| run != &self.run_id) {
            self.acknowledge(r.token, fp, false, "run_changed");
            return false;
        }
        let result = match r.action {
            DockActionKind::Cancel => {
                if let Some(dock) = self.dock.as_mut() {
                    dock.cancel("operator_cancel");
                }
                Ok("dock_cancelled")
            }
            DockActionKind::Dump => match self.dock.as_mut() {
                Some(dock) if Some(dock.id) == r.dock_id && dock.active() => {
                    dock.approve().map(|_| "dump_approved")
                }
                _ => Err("dock_not_current"),
            },
            DockActionKind::Start => {
                if self.stopped {
                    Err("emergency_stop")
                } else if let Some(target) = target {
                    self.next_dock += 1;
                    match Dock::start(
                        self.dock_config.clone(),
                        self.next_dock,
                        r.token.clone(),
                        target,
                        pose,
                        now,
                    ) {
                        Ok(dock) => {
                            if let Some(old) = self.dock.as_mut() {
                                old.cancel("replaced");
                            }
                            self.drain_dock_events();
                            self.dock = Some(dock);
                            self.reset = true;
                            Ok("dock_started")
                        }
                        Err(why) => Err(why),
                    }
                } else {
                    Err("unknown_bucket")
                }
            }
        };
        let ok = result.is_ok();
        let reason = result.unwrap_or_else(|why| why);
        self.acknowledge(r.token, fp, ok, reason);
        self.drain_dock_events();
        ok
    }

    /// One detector sighting in the local world frame, for the active dock.
    pub fn observe_dock(
        &mut self,
        class: &str,
        color: Option<&str>,
        x: f64,
        y: f64,
        pose: Pose,
        now: f64,
    ) {
        if let Some(dock) = self.dock.as_mut() {
            dock.observe(class, color, x, y, pose, now);
        }
    }

    pub(super) fn cancel_dock(&mut self, reason: &str) {
        if let Some(dock) = self.dock.as_mut() {
            dock.cancel(reason);
        }
        self.gate_open = false;
        self.drain_dock_events();
    }

    /// Let the dock drive this tick. Returns true when it did, so normal
    /// motion selection is skipped. `held` comes from the initial safety checks.
    pub(super) fn drive_dock(
        &mut self,
        input: &ArbiterInput<'_>,
        decision: &mut TickDecision,
    ) -> bool {
        let approval = self.dump_approval();
        let Some(dock) = self.dock.as_mut().filter(|d| d.active()) else {
            self.gate_open = false;
            return false;
        };
        let held = decision.held.then_some(decision.reason);
        let tick = dock.step(
            input.now,
            input.pose,
            input.measured,
            input.map,
            approval,
            held,
        );
        self.gate_open = tick.gate_open;
        self.dock_in_place = tick.in_place;
        self.drain_dock_events();
        if decision.held {
            return true;
        }
        decision.source = "dock";
        decision.reason = tick.reason;
        decision.intent_command = tick.twist;
        decision.twist = tick.twist;
        if tick.twist == Twist::default() && tick.reason != "dock_creep" {
            // Zero motion is the dock's intent, not a safety hold.
            self.turning.reset();
        }
        true
    }

    fn drain_dock_events(&mut self) {
        let Some(dock) = self.dock.as_mut() else {
            return;
        };
        let token = dock.token.clone();
        for (kind, reason) in dock.take_events() {
            self.event(kind, Some(token.clone()), &reason);
        }
    }
}
