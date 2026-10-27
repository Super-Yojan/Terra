use crate::*;
impl Runtime {
    pub fn remote_action(
        &self,
        kind: &str,
        bytes: &[u8],
        age: f64,
    ) -> Result<Vec<u8>, &'static str> {
        if bytes.len() > 4096 {
            return Err("request_too_large");
        }
        let mut v: Value = serde_json::from_slice(bytes).map_err(|_| "invalid_json")?;
        if !matches!(
            kind,
            "autonomy"
                | "teleop"
                | "safety"
                | "goal"
                | "goal/decision"
                | "navigation/settings"
                | "navigation/source"
                | "navigation/resume"
                | "search"
                | "search/action"
                | "search/report/ack"
                | "task"
                | "task/action"
        ) {
            return Err("unsupported_request");
        }
        let neutral = (kind == "goal" && v["cancel"] == true)
            || (kind == "safety" && v["action"] == "stop")
            || (kind == "search/action"
                && matches!(v["action"].as_str(), Some("pause" | "cancel")))
            || kind == "search/report/ack"
            || (kind == "task/action" && v["action"] == "cancel")
            || (kind == "teleop"
                && v["linear"].as_f64() == Some(0.)
                && v["angular"].as_f64() == Some(0.));
        if !neutral {
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
        let strip = matches!(kind, "autonomy" | "safety")
            || (kind == "goal" && (!self.outdoor.selected || v["cancel"] == true));
        let o = v.as_object_mut().ok_or("invalid_json")?;
        if strip {
            o.remove("run_id");
            o.remove("authority_revision");
        } else if matches!(
            kind,
            "goal/decision"
                | "search"
                | "search/action"
                | "search/report/ack"
                | "goal"
                | "task"
                | "task/action"
        ) {
            o.remove("authority_revision");
        }
        serde_json::to_vec(&v).map_err(|_| "invalid_json")
    }
    pub(crate) fn apply_pending(&mut self, now: f64) {
        let mut queue = std::mem::take(&mut self.queue)
            .into_iter()
            .collect::<Vec<_>>();
        queue.sort_by_key(|(kind, payload, _)| {
            kind == "safety"
                && serde_json::from_slice::<Value>(payload)
                    .ok()
                    .is_some_and(|v| v["action"] == "stop")
        });
        for (kind, payload, at) in queue {
            let bytes = match self.remote_action(&kind, &payload, now - at) {
                Ok(bytes) => bytes,
                Err(reason) => {
                    let v: Value = serde_json::from_slice(&payload).unwrap_or_default();
                    self.autonomy
                        .reject_remote_request(v["token"].as_str(), reason);
                    self.events.push(json!({"kind":"command.rejected","topic":kind,"reason":reason,"age":now-at}));
                    continue;
                }
            };
            self.events.push(json!({"kind":"command.received","topic":kind,"payload":serde_json::from_slice::<Value>(&bytes).ok(),"received_at":at,"time":now}));
            let revision_before = self.autonomy.authority_revision();
            let accepted = self.dispatch_action(&kind, &bytes, now, at);
            if !accepted
                && self.autonomy.authority_revision() == revision_before
                && !kind.starts_with("navigation/")
            {
                let v: Value = serde_json::from_slice(&bytes).unwrap_or_default();
                self.autonomy
                    .reject_remote_request(v["token"].as_str(), "invalid_request");
            }
            self.events.push(
                json!({"kind":"command.admission","topic":kind,"admitted":accepted,"age":now-at}),
            );
        }
    }
    pub fn local_action(&mut self, kind: &str, bytes: &[u8], now: f64) -> Result<(), RuntimeError> {
        if !now.is_finite()
            || now < 0.0
            || bytes.len() > 4096
            || !matches!(
                kind,
                "autonomy"
                    | "safety"
                    | "goal"
                    | "goal/decision"
                    | "navigation/settings"
                    | "navigation/source"
                    | "navigation/resume"
                    | "search"
                    | "search/action"
                    | "search/report/ack"
                    | "task"
                    | "task/action"
            )
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
            "autonomy"
                if serde_json::from_slice::<Value>(bytes)
                    .is_ok_and(|v| v.get("assigned_level").is_some()) =>
            {
                terra_autonomy::decode_assignment(bytes).is_some_and(|r| {
                    let token = r.token.clone();
                    self.autonomy.assign_level(r);
                    self.autonomy.request_receipt() == Some((token.as_str(), true))
                })
            }
            "autonomy" => terra_autonomy::decode_level_with_task(&bytes)
                .map(|(r, task)| {
                    let token = r.token.clone();
                    self.autonomy.set_level_with_task(r, task);
                    self.autonomy.request_receipt() == Some((token.as_str(), true))
                })
                .unwrap_or(false),
            "teleop" => terra_autonomy::decode_teleop(&bytes).is_some_and(|r| {
                self.autonomy
                    .accept_operator_from(r, at, CommandOrigin::FleetTeleop)
            }),
            "safety" => terra_autonomy::decode_safety(&bytes)
                .map(|r| {
                    let ready = self.autonomy.is_manual()
                        || self.estimator.estimate(now).health == Health::Ready;
                    let token = r.token.clone();
                    self.autonomy.set_safety(r, ready);
                    self.autonomy.request_receipt() == Some((token.as_str(), true))
                })
                .unwrap_or(false),
            "navigation/settings" => navigation::apply(self, &bytes, now - at, now),
            "navigation/source" | "navigation/resume" => {
                outdoor::request(self, &kind, &bytes, now, now - at).is_ok()
            }
            "goal" => outdoor::dispatch_goal(self, &bytes, now, now - at),
            "search" | "search/action" | "search/report/ack" => {
                search::dispatch_search(self, &kind, &bytes, now).is_ok()
            }
            "task" => terra_autonomy::decode_task_request(&bytes).is_some_and(|r| {
                self.autonomy.run_id = self.run_id.clone();
                self.autonomy.start_task(r, now).is_ok()
            }),
            "task/action" => terra_autonomy::decode_task_action(&bytes).is_some_and(|a| {
                self.autonomy.run_id = self.run_id.clone();
                self.autonomy.apply_task_action(a, now).is_ok()
            }),
            "goal/decision" => {
                if let Some(r) = terra_autonomy::decode_decision(&bytes) {
                    let pose = self.pose.unwrap_or_default();
                    let map = self.map.as_ref();
                    let e = self.estimator.estimate(now);
                    let token = r.token.clone();
                    self.autonomy.decide_proposal(
                        r,
                        &ArbiterInput {
                            now,
                            pose: pose.0,
                            pose_time: pose.1,
                            map: map.map(|m| &m.0),
                            map_time: map.map(|m| m.1),
                            map_revision: map.map_or(0, |m| m.2),
                            measured: Twist {
                                linear: e.forward,
                                angular: e.yaw_rate,
                            },
                            healthy: pose_quality::ready(self, now),
                        },
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
