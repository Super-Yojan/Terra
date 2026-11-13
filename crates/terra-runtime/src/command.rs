use crate::*;
use terra_types::requests::{self, MAX_REQUEST_BYTES, RequestKind, Strip};

/// The `token` of a request payload, if it is a JSON object with one.
fn request_token(bytes: &[u8]) -> Option<String> {
    let v: Value = serde_json::from_slice(bytes).ok()?;
    v["token"].as_str().map(str::to_owned)
}

impl Runtime {
    /// Check a network request against current authority and its receipt
    /// `age`, then strip the authority envelope its decoder does not take.
    pub fn remote_action(
        &self,
        kind: &str,
        bytes: &[u8],
        age: f64,
    ) -> Result<Vec<u8>, &'static str> {
        if bytes.len() > MAX_REQUEST_BYTES {
            return Err("request_too_large");
        }
        let mut v: Value = serde_json::from_slice(bytes).map_err(|_| "invalid_json")?;
        let request = RequestKind::find(kind).ok_or("unsupported_request")?;
        if !request.neutral(&v) {
            if !age.is_finite()
                || !(0.0..terra_types::timing::OPERATOR_COMMAND_FRESHNESS_SECONDS).contains(&age)
            {
                return Err("remote_request_expired");
            }
            if v["run_id"] != self.run_id
                || v["authority_revision"].as_u64() != Some(self.autonomy.authority_revision())
            {
                return Err("current_authority_required");
            }
        }
        let o = v.as_object_mut().ok_or("invalid_json")?;
        if request.strip != Strip::Nothing {
            o.remove("authority_revision");
        }
        if request.strip == Strip::Authority {
            o.remove("run_id");
        }
        serde_json::to_vec(&v).map_err(|_| "invalid_json")
    }
    /// Run every queued network request, emergency stops last.
    pub(crate) fn apply_pending(&mut self, now: f64) {
        let mut queue = std::mem::take(&mut self.queue)
            .into_iter()
            .collect::<Vec<_>>();
        queue.sort_by_key(|(kind, payload, _)| {
            RequestKind::find(kind)
                .is_some_and(|k| serde_json::from_slice::<Value>(payload).is_ok_and(|v| k.stop(&v)))
        });
        for (kind, payload, at) in queue {
            let bytes = match self.remote_action(&kind, &payload, now - at) {
                Ok(bytes) => bytes,
                Err(reason) => {
                    self.autonomy
                        .reject_remote_request(request_token(&payload).as_deref(), reason);
                    self.events.push(json!({"kind":"command.rejected","topic":kind,"reason":reason,"age":now-at}));
                    continue;
                }
            };
            self.events.push(json!({"kind":"command.received","topic":kind,"payload":serde_json::from_slice::<Value>(&bytes).ok(),"received_at":at,"time":now}));
            let revision_before = self.autonomy.authority_revision();
            let accepted = self.dispatch_action(&kind, &bytes, now, at);
            if !accepted
                && self.autonomy.authority_revision() == revision_before
                && !RequestKind::find(&kind).is_some_and(|k| k.records_own_result)
            {
                self.autonomy
                    .reject_remote_request(request_token(&bytes).as_deref(), "invalid_request");
            }
            self.events.push(
                json!({"kind":"command.admission","topic":kind,"admitted":accepted,"age":now-at}),
            );
        }
    }
    /// Run a request from the phone's own UI, which needs no network authority.
    pub fn local_action(&mut self, kind: &str, bytes: &[u8], now: f64) -> Result<(), RuntimeError> {
        if !now.is_finite()
            || now < 0.0
            || bytes.len() > MAX_REQUEST_BYTES
            || !RequestKind::find(kind).is_some_and(|k| k.local)
        {
            return Err(invalid("invalid local request"));
        }
        if self.dispatch_action(kind, bytes, now, now) {
            Ok(())
        } else {
            Err(invalid("request rejected"))
        }
    }
    fn dispatch_action(&mut self, kind: &str, bytes: &[u8], now: f64, at: f64) -> bool {
        match kind {
            requests::AUTONOMY
                if serde_json::from_slice::<Value>(bytes)
                    .is_ok_and(|v| v.get("assigned_level").is_some()) =>
            {
                terra_autonomy::decode_assignment(bytes).is_some_and(|r| {
                    let token = r.token.clone();
                    self.autonomy.assign_level(r);
                    self.autonomy.request_receipt() == Some((token.as_str(), true))
                })
            }
            requests::AUTONOMY => terra_autonomy::decode_level_with_task(bytes)
                .map(|(r, task)| {
                    let token = r.token.clone();
                    self.autonomy.set_level_with_task(r, task);
                    self.autonomy.request_receipt() == Some((token.as_str(), true))
                })
                .unwrap_or(false),
            requests::TELEOP => terra_autonomy::decode_teleop(bytes).is_some_and(|r| {
                self.autonomy
                    .accept_operator_from(r, at, CommandOrigin::FleetTeleop)
            }),
            requests::SAFETY => terra_autonomy::decode_safety(bytes)
                .map(|r| {
                    let reset = r.action == terra_autonomy::SafetyAction::Reset;
                    // A tilt stop clears only once the rover is upright again.
                    // Name the cause so the dashboard can tell the operator what to do.
                    let refusal = if reset && self.tilt.latched() && self.tilt.still_tilted() {
                        Some("still_tilted")
                    } else if !self.autonomy.is_manual()
                        && self.estimator.estimate(now).health != Health::Ready
                    {
                        Some("pose_not_ready")
                    } else {
                        None
                    };
                    let token = r.token.clone();
                    self.autonomy.set_safety_refusing(r, refusal);
                    let ok = self.autonomy.request_receipt() == Some((token.as_str(), true));
                    if ok && reset && self.tilt.clear() {
                        self.autonomy.set_tilt_stop(false);
                    }
                    ok
                })
                .unwrap_or(false),
            requests::NAVIGATION_SETTINGS => navigation::apply(self, bytes, now),
            requests::NAVIGATION_SOURCE | requests::NAVIGATION_RESUME => {
                outdoor::request(self, kind, bytes, now).is_ok()
            }
            requests::GOAL => outdoor::dispatch_goal(self, bytes, now),
            requests::SEARCH | requests::SEARCH_ACTION | requests::SEARCH_REPORT_ACK => {
                search::dispatch_search(self, kind, bytes, now).is_ok()
            }
            requests::TASK => terra_autonomy::decode_task_request(bytes)
                .is_some_and(|r| self.autonomy.start_task(r, now).is_ok()),
            requests::TASK_ACTION => terra_autonomy::decode_task_action(bytes)
                .is_some_and(|a| self.autonomy.apply_task_action(a, now).is_ok()),
            requests::DOCK => terra_autonomy::decode_dock(bytes).is_some_and(|r| {
                let target = r.object_id.and_then(|id| self.objects.dock_target(id));
                let pose = self.pose.unwrap_or_default().0;
                let accepted = self.autonomy.apply_dock_request(r, target, pose, now);
                if self.autonomy.dock_active() {
                    // A gate test ends when a dock takes the gate.
                    self.gate_test = None;
                }
                accepted
            }),
            requests::GOAL_DECISION => {
                if let Some(r) = terra_autonomy::decode_decision(bytes) {
                    let healthy = self.sensors_ready(now);
                    let token = r.token.clone();
                    self.autonomy.decide_proposal(
                        r,
                        &arbiter_input(&self.estimator, self.pose, self.map.as_ref(), now, healthy),
                    );
                    self.autonomy.request_receipt() == Some((token.as_str(), true))
                } else {
                    false
                }
            }
            _ => false,
        }
    }
}
