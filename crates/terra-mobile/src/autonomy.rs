//! Phone-facing autonomy bindings and conversion to onboard arbiter inputs.

use crate::*;

/// One video-signaling effect for the phone UI. `kind` is `publish`, `create_offer`,
/// `apply_answer`, `add_ice`, `end_of_candidates`, `stop_media`, or `show`.
/// Unused strings are empty. This record cannot arm or move the rover.
#[derive(uniffi::Record)]
pub struct FpvEffect {
    pub kind: String,
    pub session: String,
    pub topic: String,
    pub payload: String,
    pub sdp: String,
    pub candidate: String,
    pub sdp_mid: String,
    pub sdp_m_line_index: i32,
    pub state: String,
    pub detail: String,
}

fn fpv_effects(
    controller: &MobileController,
    step: impl FnOnce(&mut fpv_signaling::SignalingSession) -> Vec<fpv_signaling::Effect>,
) -> Vec<FpvEffect> {
    let Ok(mut session) = controller.fpv.lock() else {
        return Vec::new();
    };
    step(&mut session)
        .into_iter()
        .map(FpvEffect::from)
        .collect()
}

impl From<fpv_signaling::Effect> for FpvEffect {
    fn from(effect: fpv_signaling::Effect) -> Self {
        let mut out = Self {
            kind: String::new(),
            session: String::new(),
            topic: String::new(),
            payload: String::new(),
            sdp: String::new(),
            candidate: String::new(),
            sdp_mid: String::new(),
            sdp_m_line_index: 0,
            state: String::new(),
            detail: String::new(),
        };
        match effect {
            fpv_signaling::Effect::Publish { topic, payload } => {
                out.kind = "publish".into();
                out.topic = topic.suffix().into();
                out.payload = payload;
            }
            fpv_signaling::Effect::CreateOffer { session } => {
                out.kind = "create_offer".into();
                out.session = session;
            }
            fpv_signaling::Effect::AcceptOffer { session, sdp } => {
                out.kind = "accept_offer".into();
                out.session = session;
                out.sdp = sdp;
            }
            fpv_signaling::Effect::ApplyAnswer { session, sdp } => {
                out.kind = "apply_answer".into();
                out.session = session;
                out.sdp = sdp;
            }
            fpv_signaling::Effect::AddIce {
                session,
                candidate,
                sdp_mid,
                sdp_m_line_index,
            } => {
                out.kind = "add_ice".into();
                out.session = session;
                out.candidate = candidate;
                out.sdp_mid = sdp_mid;
                out.sdp_m_line_index = sdp_m_line_index;
            }
            fpv_signaling::Effect::EndOfCandidates { session } => {
                out.kind = "end_of_candidates".into();
                out.session = session;
            }
            fpv_signaling::Effect::StopMedia { session } => {
                out.kind = "stop_media".into();
                out.session = session;
            }
            fpv_signaling::Effect::Show {
                session,
                state,
                detail,
            } => {
                out.kind = "show".into();
                out.session = session;
                out.state = state.as_str().into();
                out.detail = detail.into();
            }
        }
        out
    }
}

/// One WebRTC signaling sample drained from the fleet session.
/// `kind` is the topic suffix. `payload` is the raw JSON text.
/// The sample is video signaling, not a drive or arm command.
#[derive(uniffi::Record)]
pub struct WebrtcSignal {
    /// Topic suffix such as `webrtc/request` or `webrtc/answer`.
    pub kind: String,
    /// UTF-8 JSON body. Invalid UTF-8 samples are dropped before this record is built.
    pub payload: String,
}

pub(crate) fn accepted_mission(b: &Brain) -> bool {
    b.connectivity.autonomous_router_loss
        == connectivity_policy::RouterLossPolicy::ContinueAcceptedMission
        && if b.outdoor.selected {
            b.outdoor.active() && b.outdoor.phase() == "planning" && b.autonomy.outdoor_authority()
        } else {
            b.autonomy
                .accepted_onboard_mission(b.pose.unwrap_or_default().0)
        }
}

/// Validate network authority before strict request decoding. Local phone APIs
/// bypass this ingress and continue to use their own admission checks.
#[cfg(test)]
pub(crate) fn remote_action(
    b: &Brain,
    kind: &str,
    bytes: &[u8],
    age: f64,
) -> Result<Vec<u8>, &'static str> {
    b.runtime.remote_action(kind, bytes, age)
}

impl MobileController {
    pub(crate) fn apply_dashboard_link_loss(&self, b: &mut Brain, timestamp: f64) {
        if b.router_loss_handled {
            return;
        }
        let mission = accepted_mission(b);
        b.runtime.link(false);
        self.observation.event("router", "connection.loss_decision", serde_json::json!({
            "policy": b.connectivity, "policy_revision": b.connectivity_revision,
            "accepted_mission_continues": mission,
            "mission": serde_json::from_str::<serde_json::Value>(&b.autonomy_json).ok().map(|v| v["goal"].clone()),
        }), &b.run_id, timestamp);
    }
    fn install_dashboard(
        &self,
        service: terra_transport::ControlPlane,
    ) -> Result<(), ControllerError> {
        let mut brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        brain.map_gate = Default::default();
        brain.cloud_gate = Default::default();
        capture::reset(&mut brain);
        brain.runtime.link(true);
        brain.dashboard = Some(service);
        Ok(())
    }
}

/// Levels the shared arbiter accepts today.
///
/// Target search is advertised dynamically only when a fresh detector is available.
#[uniffi::export]
pub fn supported_autonomy_levels() -> Vec<String> {
    [
        terra_autonomy::Level::Teleop,
        terra_autonomy::Level::AssistedTeleop,
        terra_autonomy::Level::Waypoint,
        terra_autonomy::Level::WaypointDirect,
        terra_autonomy::Level::Explore,
        terra_autonomy::Level::Supervised,
    ]
    .into_iter()
    .map(|level| match level {
        terra_autonomy::Level::Teleop => "teleop",
        terra_autonomy::Level::AssistedTeleop => "assisted_teleop",
        terra_autonomy::Level::Waypoint => "waypoint",
        terra_autonomy::Level::Supervised => "supervised",
        terra_autonomy::Level::WaypointDirect => "waypoint_direct",
        terra_autonomy::Level::Explore => "explore",
        terra_autonomy::Level::TargetSearch => "target_search",
    })
    .map(str::to_string)
    .collect()
}

#[uniffi::export]
impl MobileController {
    /// Whether transport replacement can preserve an accepted onboard mission.
    /// Local safety admission is evaluated independently on every execution tick.
    pub fn connectivity_policy_settings(&self) -> String {
        self.brain
            .lock()
            .map(|b| serde_json::to_string(&b.connectivity).expect("validated policy"))
            .unwrap_or_default()
    }
    pub fn accepted_onboard_mission(&self) -> bool {
        self.brain
            .lock()
            .map(|b| accepted_mission(&b))
            .unwrap_or(false)
    }
    pub fn set_connectivity_policy_settings(
        &self,
        settings_json: String,
    ) -> Result<(), ControllerError> {
        if settings_json.len() > 4096 {
            return Err(ControllerError::InvalidInput {
                message: "connectivity policy too large".into(),
            });
        }
        let policy: connectivity_policy::ConnectivityPolicySettings =
            serde_json::from_str(&settings_json).map_err(|_| ControllerError::InvalidInput {
                message: "invalid connectivity policy".into(),
            })?;
        policy
            .validate()
            .map_err(|e| ControllerError::InvalidInput { message: e.into() })?;
        let mut b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        b.runtime.set_connectivity_policy(policy)?;
        self.observation.event(
            "router",
            "connectivity.policy_changed",
            serde_json::json!({"policy": b.connectivity, "revision": b.connectivity_revision}),
            &b.run_id,
            b.imu_time.unwrap_or(0.),
        );
        Ok(())
    }
    /// Automatic transport failure is distinct from explicit operator disconnect.
    pub fn handle_dashboard_link_loss(&self) -> Result<(), ControllerError> {
        {
            let mut b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
            let timestamp = b.imu_time.unwrap_or(0.);
            self.apply_dashboard_link_loss(&mut b, timestamp);
        }
        self.detach_dashboard_link()
    }
    /// Bounded world-space visualization points; never a source of motor authority.
    /// Requires version 1 JSON, at most 4096 points, and at most 262144 bytes.
    /// Published only while a dashboard point capture runs; otherwise accepted and dropped.
    /// Full content is coalesced; unchanged newer samples can emit heartbeats.
    /// Returns `InvalidInput` for size/schema rejection and `Internal` for malformed
    /// JSON or lock failure. No dashboard means no network publication.
    pub fn publish_point_cloud(&self, payload: String) -> Result<(), ControllerError> {
        if payload.len() > 262144 {
            return Err(ControllerError::InvalidInput {
                message: "point cloud too large".into(),
            });
        }
        let mut value: serde_json::Value =
            serde_json::from_str(&payload).map_err(|_| ControllerError::Internal)?;
        if value["version"].as_u64() != Some(1)
            || value["points"].as_array().is_none_or(|p| p.len() > 4096)
        {
            return Err(ControllerError::InvalidInput {
                message: "invalid point cloud".into(),
            });
        }
        let mut brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        // Exact parsed content comparison; preserve original full-payload bytes.
        let frame_id = value["frameID"].as_str().map(str::to_owned);
        let publication = if brain.dashboard.is_some() && capture::thinned_cloud_wanted(&brain) {
            if let (Some(sequence), Some(_)) = (value["sequence"].as_u64(), frame_id.as_ref()) {
                value.as_object_mut().unwrap().remove("sequence");
                brain.cloud_gate.observe(
                    value,
                    frame_id.clone().unwrap(),
                    sequence,
                    std::time::Instant::now(),
                    std::time::Duration::from_millis(500),
                )
            } else {
                brain.cloud_gate = Default::default();
                publication::Publication::Full
            }
        } else {
            publication::Publication::Skip
        };
        let mut full_rejected = false;
        if let Some(dashboard) = brain.dashboard.as_ref() {
            match publication {
                publication::Publication::Full => {
                    full_rejected = !dashboard.publish_visualization("pointcloud", payload);
                }
                publication::Publication::Heartbeat(sequence) => dashboard.publish(
                    "pointcloud/heartbeat",
                    serde_json::json!({"version":1,"rover_id":dashboard.rover_id,
                        "frameID":frame_id,"sequence":sequence})
                    .to_string(),
                ),
                publication::Publication::Skip => {}
            }
        }
        brain.cloud_gate.full_enqueued(!full_rejected);
        Ok(())
    }
    /// Publish positioning quality without changing the controller's local motion frame.
    /// Requires version 1 JSON at most 4096 bytes. With no dashboard this is a no-op.
    /// Returns `InvalidInput` for size, JSON, or version rejection, or `Internal` on lock failure.
    pub fn publish_localization(&self, payload: String) -> Result<(), ControllerError> {
        if payload.len() > 4096 {
            return Err(ControllerError::InvalidInput {
                message: "localization payload too large".into(),
            });
        }
        let mut value: serde_json::Value =
            serde_json::from_str(&payload).map_err(|_| ControllerError::InvalidInput {
                message: "invalid localization JSON".into(),
            })?;
        if value["version"].as_u64() != Some(1) {
            return Err(ControllerError::InvalidInput {
                message: "unsupported localization version".into(),
            });
        }
        let brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        value["run_id"] = serde_json::json!(brain.run_id);
        value["connectivity_policy"] = serde_json::json!(brain.connectivity);
        if let Some(dashboard) = brain.dashboard.as_ref() {
            dashboard.publish("localization", value.to_string());
        }
        Ok(())
    }
    /// Host a loopback-only TCP dashboard endpoint for the selected rover.
    /// Run off the UI thread. Returns `InvalidInput` for endpoint/prefix or network
    /// failure, or `Internal` for a poisoned mutex.
    pub fn host_dashboard(
        &self,
        endpoint: String,
        prefix: String,
        rover_id: u64,
    ) -> Result<(), ControllerError> {
        let service =
            terra_transport::ControlPlane::listen(&endpoint, &prefix, rover_id).map_err(|e| {
                ControllerError::InvalidInput {
                    message: e.to_string(),
                }
            })?;
        self.install_dashboard(service)?;
        Ok(())
    }
    /// Attach an outbound dashboard session to this controller; no hardware arming.
    /// Clears previous remote intent before connecting; run off the UI thread.
    /// Returns `InvalidInput` for transport validation/connection failure or `Internal`
    /// for lock failure. Connection does not arm hardware.
    pub fn connect_dashboard(
        &self,
        endpoint: String,
        prefix: String,
        rover_id: u64,
    ) -> Result<(), ControllerError> {
        self.disconnect_dashboard()?;
        let service = terra_transport::ControlPlane::connect(&endpoint, &prefix, rover_id)
            .map_err(|e| {
                self.observation.router_connect_failed(&e);
                ControllerError::InvalidInput {
                    message: e.to_string(),
                }
            })?;
        self.install_dashboard(service)?;
        Ok(())
    }
    /// Clear remote intent and open an outbound dashboard session with Zenoh credentials.
    /// An empty user omits credential configuration. Run off the UI thread.
    /// Returns `InvalidInput` for transport failure or `Internal` for lock failure.
    pub fn connect_dashboard_authenticated(
        &self,
        endpoint: String,
        prefix: String,
        rover_id: u64,
        user: String,
        password: String,
    ) -> Result<(), ControllerError> {
        self.disconnect_dashboard()?;
        let service = terra_transport::ControlPlane::connect_authenticated(
            &endpoint, &prefix, rover_id, &user, &password,
        )
        .map_err(|e| {
            self.observation.router_connect_failed(&e);
            ControllerError::InvalidInput {
                message: e.to_string(),
            }
        })?;
        self.install_dashboard(service)?;
        Ok(())
    }
    /// Drop network transport without clearing an accepted local mission.
    /// Clears queued hardware requests and tokens; closes the old link asynchronously.
    /// Returns `Internal` for a poisoned mutex.
    pub fn detach_dashboard_link(&self) -> Result<(), ControllerError> {
        let mut b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        b.autonomy.clear_remote_intent();
        let service = b.dashboard.take();
        drop(b);
        // Network teardown must not stall the phone control/Bluetooth producer.
        if let Some(service) = service {
            std::thread::spawn(move || drop(service));
        }
        Ok(())
    }
    /// Replace the dashboard transport while preserving accepted local mission state.
    /// Old hardware requests are discarded. Run off the UI thread. Returns `InvalidInput`
    /// for transport failure or `Internal` for lock failure.
    pub fn reconnect_dashboard(
        &self,
        endpoint: String,
        prefix: String,
        rover_id: u64,
    ) -> Result<(), ControllerError> {
        self.detach_dashboard_link()?;
        let service = terra_transport::ControlPlane::connect(&endpoint, &prefix, rover_id)
            .map_err(|e| {
                self.observation.router_connect_failed(&e);
                ControllerError::InvalidInput {
                    message: e.to_string(),
                }
            })?;
        self.install_dashboard(service)?;
        Ok(())
    }
    /// Replace the link using Zenoh credentials while preserving accepted local mission state.
    /// An empty user omits credential configuration. Old hardware requests are discarded.
    /// Returns `InvalidInput` for transport failure or `Internal` for lock failure.
    pub fn reconnect_dashboard_authenticated(
        &self,
        endpoint: String,
        prefix: String,
        rover_id: u64,
        user: String,
        password: String,
    ) -> Result<(), ControllerError> {
        self.detach_dashboard_link()?;
        let service = terra_transport::ControlPlane::connect_authenticated(
            &endpoint, &prefix, rover_id, &user, &password,
        )
        .map_err(|e| {
            self.observation.router_connect_failed(&e);
            ControllerError::InvalidInput {
                message: e.to_string(),
            }
        })?;
        self.install_dashboard(service)?;
        Ok(())
    }
    /// Clear remote intent while preserving sensors, recording and a latched emergency stop.
    /// Switches authority back to teleop, resets the velocity loop, and closes the
    /// network link. May block during teardown. Returns `Internal` for lock failure.
    pub fn disconnect_dashboard(&self) -> Result<(), ControllerError> {
        let mut brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        brain.outdoor.pause("authority_lost");
        let service = brain.dashboard.take();
        brain.sequence += 1;
        let token = format!("dashboard-detach-{}", brain.sequence);
        brain.autonomy.set_level(terra_autonomy::LevelRequest {
            level: terra_autonomy::Level::Teleop,
            token,
            budget_seconds: None,
            budget_minutes: None,
        });
        brain.controller.reset();
        drop(brain);
        drop(service);
        Ok(())
    }
    /// Store fresh hardware observation JSON and publish it to a connected dashboard.
    /// Used when validating configuration changes. Returns `InvalidInput` for
    /// malformed JSON or more than 4096 bytes, and `Internal` for lock failure.
    pub fn publish_hardware_status(&self, payload: String) -> Result<(), ControllerError> {
        if payload.len() > 4096 || serde_json::from_str::<serde_json::Value>(&payload).is_err() {
            return Err(ControllerError::InvalidInput {
                message: "invalid hardware status".into(),
            });
        }
        let mut b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        b.hardware_observation = Some((
            serde_json::from_str(&payload).unwrap(),
            b.imu_time.unwrap_or(b.last_tick).max(b.last_tick),
        ));
        if let Some(d) = b.dashboard.as_ref() {
            d.publish("hardware/status", payload);
        }
        Ok(())
    }
    /// Drain dashboard-to-phone WebRTC signaling since the previous call.
    /// Returns an empty list when no dashboard is connected or the lock is poisoned.
    /// Samples are not autonomy, hardware, or teleop commands.
    pub fn take_webrtc_signaling(&self) -> Vec<WebrtcSignal> {
        let Ok(brain) = self.brain.lock() else {
            return Vec::new();
        };
        let Some(dashboard) = brain.dashboard.as_ref() else {
            return Vec::new();
        };
        dashboard
            .take_signaling()
            .into_iter()
            .filter_map(|(kind, bytes)| {
                Some(WebrtcSignal {
                    kind,
                    payload: String::from_utf8(bytes).ok()?,
                })
            })
            .collect()
    }
    /// Publish one phone-to-dashboard WebRTC signaling sample on the fleet session.
    /// `kind` must be `webrtc/offer`, `webrtc/ice/phone`, or `webrtc/state`.
    /// `payload` must be JSON of at most 65536 bytes. This does not arm, disarm,
    /// change autonomy, or publish a velocity command.
    ///
    /// Returns `InvalidInput` when the dashboard is absent or the sample is rejected,
    /// and `Internal` when the controller lock is poisoned.
    pub fn publish_webrtc_signaling(
        &self,
        kind: String,
        payload: String,
    ) -> Result<(), ControllerError> {
        if fpv_signaling::decode(&kind, &payload)
            .is_none_or(|message| !message.topic.is_phone_outbound())
        {
            return Err(ControllerError::InvalidInput {
                message: "rejected webrtc signaling".into(),
            });
        }
        let brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        let Some(dashboard) = brain.dashboard.as_ref() else {
            return Err(ControllerError::InvalidInput {
                message: "dashboard is not connected".into(),
            });
        };
        if !dashboard.publish_signaling(&kind, &payload) {
            return Err(ControllerError::InvalidInput {
                message: "rejected webrtc signaling".into(),
            });
        }
        Ok(())
    }
    /// Advance the phone FPV session from one dashboard signaling sample.
    /// Effects are video cues and signaling publishes. They are not drive commands.
    /// `now_ms` is a monotonic millisecond clock. A poisoned lock returns nothing.
    pub fn fpv_receive(&self, kind: String, payload: String, now_ms: u64) -> Vec<FpvEffect> {
        fpv_effects(self, |session| session.receive(&kind, &payload, now_ms))
    }
    /// Fail the call when the offer, answer, or connection has exceeded its timeout.
    pub fn fpv_tick(&self, now_ms: u64) -> Vec<FpvEffect> {
        fpv_effects(self, |session| session.tick(now_ms))
    }
    /// Record the local SDP offer. The returned publish effect carries `webrtc/offer`.
    pub fn fpv_note_offer(&self, session: String, sdp: String, now_ms: u64) -> Vec<FpvEffect> {
        fpv_effects(self, |machine| {
            machine.note_local_description(&session, &sdp, now_ms)
        })
    }
    /// Record one local ICE candidate. An empty candidate ends gathering.
    pub fn fpv_note_ice(
        &self,
        session: String,
        candidate: String,
        sdp_mid: String,
        sdp_m_line_index: i32,
        now_ms: u64,
    ) -> Vec<FpvEffect> {
        fpv_effects(self, |machine| {
            machine.note_local_ice(&session, &candidate, &sdp_mid, sdp_m_line_index, now_ms)
        })
    }
    /// Record a local peer-connection state: `connecting`, `connected`, `failed`, or `closed`.
    pub fn fpv_note_media(&self, session: String, state: String, now_ms: u64) -> Vec<FpvEffect> {
        let Some(state) = fpv_signaling::CallState::parse(&state) else {
            return Vec::new();
        };
        fpv_effects(self, |machine| machine.note_media(&session, state, now_ms))
    }
    /// End the phone's call. When `publish` is set, the effect list includes `webrtc/state` `closed`.
    pub fn fpv_stop(&self, publish: bool) -> Vec<FpvEffect> {
        fpv_effects(self, |machine| machine.stop(publish))
    }
    /// Whether a signaling session is active, including a failed one waiting for stop or restart.
    pub fn fpv_has_session(&self) -> bool {
        self.fpv.lock().is_ok_and(|session| session.has_session())
    }
    /// Transport connectivity only, not acknowledgement by an ARGOS operator.
    /// Returns `connected`, `disconnected`, or `failed`; lock failure also yields `failed`.
    pub fn dashboard_status(&self) -> String {
        self.brain
            .lock()
            .map(|brain| {
                self.observation.router_status(brain.dashboard.as_ref());
                match brain.dashboard.as_ref() {
                    None => "disconnected",
                    Some(service) if service.failed() => "failed",
                    Some(_) => "connected",
                }
                .to_string()
            })
            .unwrap_or_else(|_| "failed".into())
    }
    /// Decode and dispatch local autonomy, safety, goal, decision, or search JSON intent.
    /// `timestamp` is current sensor-clock seconds. Successful dispatch does not
    /// guarantee authority or motion. Returns `InvalidInput` for unsupported kind,
    /// decoding failure, rejected goal/search request; `Internal` for lock failure.
    pub fn autonomy_request(
        &self,
        kind: String,
        payload: String,
        timestamp: f64,
    ) -> Result<(), ControllerError> {
        let outcome = (|| {
            let mut b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
            b.runtime
                .local_action(&kind, payload.as_bytes(), timestamp)
                .map_err(ControllerError::from)
        })();
        let run = self.observation_run_id();
        self.observation.event("operator", if outcome.is_ok() { "command.dispatched" } else { "command.rejected" }, serde_json::json!({"topic":kind,"payload":serde_json::from_str::<serde_json::Value>(&payload).ok(),"error":outcome.as_ref().err().map(|e: &ControllerError|e.to_string())}), &run, timestamp);
        outcome
    }
    /// Set the WGS84 origin using finite decimal-degree latitude and longitude.
    /// Returns `InvalidInput` for invalid coordinates or `Internal` for lock failure.
    pub fn autonomy_origin(&self, latitude: f64, longitude: f64) -> Result<(), ControllerError> {
        terra_waypoint::GeoOrigin::new(latitude, longitude)?;
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .autonomy
            .set_origin(latitude, longitude);
        Ok(())
    }
    /// Install a world-frame occupancy snapshot with source time and revision.
    /// `timestamp` is nonnegative finite sensor-clock seconds; `revision` identifies
    /// the source sample. Dimensions must be 1..=1000 with an exact cell count,
    /// resolution at least 0.01 m, and finite origin coordinates.
    /// Returns `InvalidInput` for failed validation or `Internal` for lock failure.
    pub fn autonomy_map(
        &self,
        grid: OccupancyGrid,
        timestamp: f64,
        revision: u64,
    ) -> Result<(), ControllerError> {
        if !timestamp.is_finite()
            || timestamp < 0.
            || grid.width == 0
            || grid.height == 0
            || grid.width > 1000
            || grid.height > 1000
            || grid.occupancy.len() != grid.width as usize * grid.height as usize
            || !grid.resolution.is_finite()
            || grid.resolution < 0.01
            || !grid.origin_x.is_finite()
            || !grid.origin_y.is_finite()
        {
            return Err(InputError::OutOfRange.into());
        }
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .map = Some((
            terra_mapping::MapSnapshot {
                width: grid.width,
                height: grid.height,
                resolution: grid.resolution,
                origin_x: grid.origin_x,
                origin_y: grid.origin_y,
                occupancy: grid.occupancy,
            },
            timestamp,
            revision,
        ));
        Ok(())
    }
    /// Copy the cached JSON status from the most recent control tick.
    /// Returns an empty string before the first tick or on lock failure.
    pub fn autonomy_status(&self) -> String {
        self.brain
            .lock()
            .map(|b| b.autonomy_json.clone())
            .unwrap_or_default()
    }
    /// Create a bounded run recorder at `path` and start a new run identity.
    /// Resets autonomy, velocity control, and run timing while retaining navigation
    /// geometry. `run_id` must be a valid token. Returns `InvalidInput` for invalid
    /// identity or recorder create/close failure, or `Internal` for lock failure.
    pub fn begin_recording(&self, path: String, run_id: String) -> Result<(), ControllerError> {
        if !terra_autonomy::valid_token(&run_id) {
            return Err(ControllerError::InvalidInput {
                message: "invalid run identity".into(),
            });
        }
        let mut brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        let recorder = terra_experiment::RunRecorder::create(
            std::path::Path::new(&path),
            serde_json::json!({
                "run_id": run_id,
                "platform": "phone",
                "runtime": brain.runtime.metadata(),
                "planner": brain.navigation.parameters.planner(),
                "collision_observation": "unavailable",
            }),
            8192,
        )
        .map_err(|e| ControllerError::InvalidInput {
            message: e.to_string(),
        })?;
        if let Some(mut old) = brain.recorder.take() {
            old.close().map_err(|e| ControllerError::InvalidInput {
                message: e.to_string(),
            })?;
        }
        brain.recorder = Some(recorder);

        brain.runtime.begin_run(run_id)?;
        brain.run_start = None;
        brain.sequence = 0;
        Ok(())
    }
    /// Take and close the current recorder; succeeds when none is active.
    /// Returns `InvalidInput` for recorder close failure or `Internal` for lock failure.
    pub fn end_recording(&self) -> Result<(), ControllerError> {
        if let Some(mut r) = self
            .brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .recorder
            .take()
        {
            r.close().map_err(|e| ControllerError::InvalidInput {
                message: e.to_string(),
            })?;
        }
        Ok(())
    }
}

/// Dashboard teardown scenarios, including sensor retention and latched emergency stop.
#[cfg(test)]
mod dashboard_tests {
    use super::*;
    #[test]
    /// Starts a waypoint with ready sensors, disconnects twice, and checks idle teleop authority
    /// while sensor readiness survives teardown.
    fn dashboard_disconnect_clears_goal_and_authority_without_resetting_sensors() {
        let controller = MobileController::new(default_control_settings()).unwrap();
        controller
            .push_imu(ImuReading {
                timestamp: 1.,
                acceleration_forward: 0.,
                acceleration_left: 0.,
                acceleration_up: 0.,
                gyro_roll: 0.,
                gyro_pitch: 0.,
                gyro_yaw: 0.,
            })
            .unwrap();
        controller
            .push_vio(VioReading {
                timestamp: 1.,
                position_x: 0.,
                position_y: 0.,
                position_z: 0.,
                quaternion_x: 0.,
                quaternion_y: 0.,
                quaternion_z: 0.,
                quaternion_w: 1.,
                velocity_x: 0.,
                velocity_y: 0.,
                velocity_z: 0.,
                tracked: true,
            })
            .unwrap();
        controller
            .autonomy_request(
                "autonomy".into(),
                r#"{"level":"waypoint","token":"level"}"#.into(),
                1.,
            )
            .unwrap();
        controller
            .autonomy_request(
                "goal".into(),
                r#"{"frame":"local","x":2,"y":0,"token":"goal"}"#.into(),
                1.,
            )
            .unwrap();
        controller.disconnect_dashboard().unwrap();
        controller.step(1.).unwrap();
        let status: serde_json::Value =
            serde_json::from_str(&controller.autonomy_status()).unwrap();
        assert_eq!(status["status"]["requested_level"], "teleop");
        assert_eq!(status["goal"]["state"], "idle");
        assert_eq!(controller.dashboard_status(), "disconnected");
        assert_eq!(
            controller
                .brain
                .lock()
                .unwrap()
                .estimator
                .estimate(1.)
                .health,
            Health::Ready
        );
        controller.disconnect_dashboard().unwrap();
    }
    #[test]
    /// Latches an emergency stop before dashboard disconnect and verifies the next tick remains stopped.
    fn dashboard_disconnect_preserves_emergency_stop() {
        let controller = MobileController::new(default_control_settings()).unwrap();
        controller
            .autonomy_request(
                "safety".into(),
                r#"{"action":"stop","token":"stop"}"#.into(),
                1.,
            )
            .unwrap();
        controller.disconnect_dashboard().unwrap();
        let output = controller.step(1.).unwrap();
        assert_eq!(output.left_effort, 0.);
        assert_eq!(output.right_effort, 0.);
        let status: serde_json::Value =
            serde_json::from_str(&controller.autonomy_status()).unwrap();
        assert_eq!(status["status"]["safety"], "emergency_stop");
    }
}

#[cfg(test)]
#[path = "../tests/internal/router_policy.rs"]
mod router_policy_tests;
