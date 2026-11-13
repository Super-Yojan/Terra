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

/// Link variants only the Rust tests use; the apps call
/// `reconnect_dashboard_authenticated`.
impl MobileController {
    /// Host a loopback-only TCP dashboard endpoint for the selected rover.
    pub fn host_dashboard(
        &self,
        endpoint: String,
        prefix: String,
        rover_id: u64,
    ) -> Result<(), ControllerError> {
        self.attach_dashboard(|| {
            terra_transport::ControlPlane::listen(&endpoint, &prefix, rover_id)
        })
    }
    /// Close any fleet link the way the operator does, then connect without credentials.
    pub fn connect_dashboard(
        &self,
        endpoint: String,
        prefix: String,
        rover_id: u64,
    ) -> Result<(), ControllerError> {
        self.disconnect_dashboard()?;
        self.attach_dashboard(|| {
            terra_transport::ControlPlane::connect(&endpoint, &prefix, rover_id)
        })
    }
    /// Replace the link without credentials, keeping an accepted local mission.
    pub fn reconnect_dashboard(
        &self,
        endpoint: String,
        prefix: String,
        rover_id: u64,
    ) -> Result<(), ControllerError> {
        self.detach_dashboard_link()?;
        self.attach_dashboard(|| {
            terra_transport::ControlPlane::connect(&endpoint, &prefix, rover_id)
        })
    }
}

impl MobileController {
    /// Whether an attached, working fleet link owns velocity intent, so local
    /// manual input and targets are refused.
    pub(crate) fn fleet_owns_intent(&self) -> bool {
        self.brain.lock().is_ok_and(|b| b.fleet_owns_intent())
    }
    pub(crate) fn apply_dashboard_link_loss(&self, b: &mut Brain, timestamp: f64) {
        if let Some(decision) = b.runtime.lose_link() {
            self.observation.event(
                "router",
                "connection.loss_decision",
                decision,
                b.run_id(),
                timestamp,
            );
        }
    }
    /// Open a fleet link with `open` and attach it; a failure to open is journaled.
    fn attach_dashboard(
        &self,
        open: impl FnOnce() -> Result<terra_transport::ControlPlane, terra_transport::TransportError>,
    ) -> Result<(), ControllerError> {
        let service = open().map_err(|e| {
            self.observation.router_connect_failed(&e);
            ControllerError::InvalidInput {
                message: e.to_string(),
            }
        })?;
        let mut brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        brain.map_gate = Default::default();
        capture::reset(&mut brain);
        let at = brain.last_tick;
        brain.runtime.ingest(terra_runtime::RuntimeEvent::Link {
            connected: true,
            received_at: at,
        })?;
        // ARGOS rejects a pose/quality report without a numeric rover_id and
        // then falls back to its own yaw-rate guess, so every turn showed as
        // degraded pose (#111).
        brain.runtime.set_rover_id(Some(service.rover_id()));
        brain.dashboard = Some(service);
        Ok(())
    }
}

/// Levels the shared arbiter accepts today.
///
/// Target search is advertised dynamically only when a fresh detector is available,
/// and the L5 `task` level only while the phone advertises a language model.
#[uniffi::export]
pub fn supported_autonomy_levels() -> Vec<String> {
    terra_autonomy::Level::BASE
        .iter()
        .map(|level| level.as_str().to_string())
        .collect()
}

/// Operator-facing label for a wire level name, from [`terra_autonomy::Level::label`].
/// Unknown names are returned unchanged.
#[uniffi::export]
pub fn autonomy_level_label(level: String) -> String {
    terra_autonomy::Level::parse(&level)
        .map(|l| l.label().to_string())
        .unwrap_or(level)
}

#[uniffi::export]
impl MobileController {
    /// Whether transport replacement can preserve an accepted onboard mission.
    /// Local safety admission is evaluated independently on every execution tick.
    pub fn connectivity_policy_settings(&self) -> String {
        self.brain
            .lock()
            .map(|b| serde_json::to_string(b.connectivity()).expect("validated policy"))
            .unwrap_or_default()
    }
    pub fn accepted_onboard_mission(&self) -> bool {
        self.brain
            .lock()
            .map(|b| b.accepted_onboard_mission())
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
        let policy: terra_runtime::connectivity::ConnectivityPolicySettings =
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
            serde_json::json!({"policy": b.connectivity(), "revision": b.connectivity_revision()}),
            b.run_id(),
            b.imu_time().unwrap_or(0.),
        );
        Ok(())
    }
    /// Automatic transport failure is distinct from explicit operator disconnect.
    pub fn handle_dashboard_link_loss(&self) -> Result<(), ControllerError> {
        {
            let mut b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
            let timestamp = b.imu_time().unwrap_or(0.);
            self.apply_dashboard_link_loss(&mut b, timestamp);
        }
        self.detach_dashboard_link()
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
        value["run_id"] = serde_json::json!(brain.run_id());
        value["connectivity_policy"] = serde_json::json!(brain.connectivity());
        if let Some(dashboard) = brain.dashboard.as_ref() {
            dashboard.publish(telemetry::LOCALIZATION, value.to_string());
        }
        Ok(())
    }
    /// Drop network transport without clearing an accepted local mission.
    /// Clears queued hardware requests and tokens; closes the old link asynchronously.
    /// Returns `Internal` for a poisoned mutex.
    pub fn detach_dashboard_link(&self) -> Result<(), ControllerError> {
        let mut b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        b.runtime.detach_operator_link();
        let service = b.dashboard.take();
        drop(b);
        // Network teardown must not stall the phone control/Bluetooth producer.
        if let Some(service) = service {
            std::thread::spawn(move || drop(service));
        }
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
        self.attach_dashboard(|| {
            terra_transport::ControlPlane::connect_authenticated(
                &endpoint, &prefix, rover_id, &user, &password,
            )
        })
    }
    /// Clear remote intent while preserving sensors, recording and a latched emergency stop.
    /// Switches authority back to teleop, resets the velocity loop, and closes the
    /// network link. May block during teardown. Returns `Internal` for lock failure.
    pub fn disconnect_dashboard(&self) -> Result<(), ControllerError> {
        let mut brain = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        let service = brain.dashboard.take();
        brain.runtime.disconnect_operator();
        drop(brain);
        drop(service);
        Ok(())
    }
    /// Store fresh hardware observation JSON and publish it to a connected dashboard.
    /// Used when validating configuration changes. Returns `InvalidInput` for
    /// malformed JSON or more than 4096 bytes, and `Internal` for lock failure.
    pub fn publish_hardware_status(&self, payload: String) -> Result<(), ControllerError> {
        let status = (payload.len() <= 4096)
            .then(|| serde_json::from_str::<serde_json::Value>(&payload).ok())
            .flatten()
            .ok_or_else(|| ControllerError::InvalidInput {
                message: "invalid hardware status".into(),
            })?;
        let mut b = self.brain.lock().map_err(|_| ControllerError::Internal)?;
        let received_at = b.imu_time().unwrap_or(b.last_tick).max(b.last_tick);
        b.runtime.ingest(terra_runtime::RuntimeEvent::Hardware {
            status: terra_runtime::HardwareStatus::from_json(&status),
            received_at,
        })?;
        if let Some(d) = b.dashboard.as_ref() {
            d.publish(telemetry::HARDWARE_STATUS, payload);
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
            .runtime
            .set_geo_origin(latitude, longitude)?;
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
        self.brain
            .lock()
            .map_err(|_| ControllerError::Internal)?
            .runtime
            .ingest(terra_runtime::RuntimeEvent::Map {
                snapshot: terra_mapping::MapSnapshot {
                    width: grid.width,
                    height: grid.height,
                    resolution: grid.resolution,
                    origin_x: grid.origin_x,
                    origin_y: grid.origin_y,
                    occupancy: grid.occupancy,
                },
                timestamp,
                sequence: revision,
                received_at: timestamp,
            })?;
        Ok(())
    }
    /// Copy the cached JSON status from the most recent control tick.
    /// Returns an empty string before the first tick or on lock failure.
    pub fn autonomy_status(&self) -> String {
        self.brain
            .lock()
            .map(|b| b.status_json().to_owned())
            .unwrap_or_default()
    }
    /// The same JSON shape as [`Self::autonomy_status`] cut to
    /// `status.{run_id, requested_level, effective_level, safety, reason}`,
    /// `pose_quality` and `goal.state`. In L3 the full status carries the
    /// planner trace (about 30 KB); parsing that several times per 100 Hz
    /// tick backed up the control queue until IMU samples went stale.
    pub fn autonomy_brief(&self) -> String {
        self.brain
            .lock()
            .map(|b| b.brief_json().to_owned())
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
                "planner": brain.navigation().parameters.planner(),
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
            controller.brain.lock().unwrap().estimate(1.).health,
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

#[cfg(test)]
mod label_tests {
    use super::autonomy_level_label;
    #[test]
    fn wire_names_map_to_operator_labels() {
        assert_eq!(autonomy_level_label("waypoint".into()), "L3 Planned");
        assert_eq!(
            autonomy_level_label("target_search".into()),
            "L4 Explore + search"
        );
        assert_eq!(autonomy_level_label("unknown".into()), "unknown");
    }
}
