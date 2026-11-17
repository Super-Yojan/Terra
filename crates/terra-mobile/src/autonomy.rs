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
        Ok(self.robot()?.attach_link(|| {
            terra_transport::ControlPlane::listen(&endpoint, &prefix, rover_id)
        })?)
    }
    /// Close any fleet link the way the operator does, then connect without credentials.
    pub fn connect_dashboard(
        &self,
        endpoint: String,
        prefix: String,
        rover_id: u64,
    ) -> Result<(), ControllerError> {
        self.disconnect_dashboard()?;
        Ok(self.robot()?.attach_link(|| {
            terra_transport::ControlPlane::connect(&endpoint, &prefix, rover_id)
        })?)
    }
    /// Replace the link without credentials, keeping an accepted local mission.
    pub fn reconnect_dashboard(
        &self,
        endpoint: String,
        prefix: String,
        rover_id: u64,
    ) -> Result<(), ControllerError> {
        let mut robot = self.robot()?;
        robot.detach_link();
        Ok(robot.attach_link(|| {
            terra_transport::ControlPlane::connect(&endpoint, &prefix, rover_id)
        })?)
    }
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
    /// The connectivity policy as JSON.
    pub fn connectivity_policy_settings(&self) -> String {
        self.robot()
            .map(|r| serde_json::to_string(r.core().connectivity()).expect("validated policy"))
            .unwrap_or_default()
    }
    /// Whether transport replacement can preserve an accepted onboard mission.
    pub fn accepted_onboard_mission(&self) -> bool {
        self.robot()
            .map(|r| r.core().accepted_onboard_mission())
            .unwrap_or(false)
    }
    /// Replace the connectivity policy (JSON).
    pub fn set_connectivity_policy_settings(
        &self,
        settings_json: String,
    ) -> Result<(), ControllerError> {
        Ok(self.robot()?.set_connectivity_policy(&settings_json)?)
    }
    /// Automatic transport failure is distinct from explicit operator disconnect.
    pub fn handle_dashboard_link_loss(&self) -> Result<(), ControllerError> {
        self.robot()?.handle_link_loss();
        Ok(())
    }
    /// Publish positioning quality (version 1 JSON). No-op without a link.
    pub fn publish_localization(&self, payload: String) -> Result<(), ControllerError> {
        Ok(self.robot()?.publish_localization(&payload)?)
    }
    /// Drop network transport without clearing an accepted local mission.
    pub fn detach_dashboard_link(&self) -> Result<(), ControllerError> {
        self.robot()?.detach_link();
        Ok(())
    }
    /// Replace the link using Zenoh credentials while preserving accepted
    /// local mission state. An empty user omits credentials.
    pub fn reconnect_dashboard_authenticated(
        &self,
        endpoint: String,
        prefix: String,
        rover_id: u64,
        user: String,
        password: String,
    ) -> Result<(), ControllerError> {
        let mut robot = self.robot()?;
        robot.detach_link();
        Ok(robot.attach_link(|| {
            terra_transport::ControlPlane::connect_authenticated(
                &endpoint, &prefix, rover_id, &user, &password,
            )
        })?)
    }
    /// Clear remote intent while keeping sensors, recording and a latched
    /// emergency stop; back to teleop. May block during teardown.
    pub fn disconnect_dashboard(&self) -> Result<(), ControllerError> {
        let service = self.robot()?.disconnect_link();
        drop(service);
        Ok(())
    }
    /// Store a hardware observation (JSON) and publish it to ARGOS.
    pub fn publish_hardware_status(&self, payload: String) -> Result<(), ControllerError> {
        Ok(self.robot()?.publish_hardware_status(payload)?)
    }
    /// Dashboard-to-phone WebRTC signaling since the previous call.
    /// Samples are never autonomy, hardware or teleop commands.
    pub fn take_webrtc_signaling(&self) -> Vec<WebrtcSignal> {
        self.robot()
            .map(|r| r.take_signaling())
            .unwrap_or_default()
            .into_iter()
            .map(|(kind, payload)| WebrtcSignal { kind, payload })
            .collect()
    }
    /// Publish one phone-to-dashboard WebRTC signaling sample: `webrtc/offer`,
    /// `webrtc/ice/phone` or `webrtc/state`, JSON of at most 65536 bytes.
    pub fn publish_webrtc_signaling(
        &self,
        kind: String,
        payload: String,
    ) -> Result<(), ControllerError> {
        if fpv_signaling::decode(&kind, &payload)
            .is_none_or(|message| !message.topic.is_phone_outbound())
        {
            return Err(invalid("rejected webrtc signaling"));
        }
        Ok(self.robot()?.publish_signaling(&kind, &payload)?)
    }
    /// Advance the phone FPV session from one dashboard signaling sample.
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
    /// `connected`, `disconnected` or `failed`; lock failure also yields `failed`.
    pub fn dashboard_status(&self) -> String {
        self.robot()
            .map(|r| r.link_status())
            .unwrap_or("failed")
            .to_string()
    }
    /// Decode and apply a local autonomy, safety, goal, decision or search
    /// request. Dispatch does not guarantee authority or motion.
    pub fn autonomy_request(
        &self,
        kind: String,
        payload: String,
        timestamp: f64,
    ) -> Result<(), ControllerError> {
        Ok(self.robot()?.local_request(&kind, &payload, timestamp)?)
    }
    /// Set the WGS84 origin in decimal degrees.
    pub fn autonomy_origin(&self, latitude: f64, longitude: f64) -> Result<(), ControllerError> {
        terra_waypoint::GeoOrigin::new(latitude, longitude)?;
        Ok(self.robot()?.core_mut().set_geo_origin(latitude, longitude)?)
    }
    /// Install a world-frame occupancy snapshot with source time and revision.
    pub fn autonomy_map(
        &self,
        grid: OccupancyGrid,
        timestamp: f64,
        revision: u64,
    ) -> Result<(), ControllerError> {
        Ok(self.robot()?.core_mut().ingest(terra_robot::Event::Map {
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
        })?)
    }
    /// Full status JSON from the latest tick; empty before the first tick.
    pub fn autonomy_status(&self) -> String {
        self.robot()
            .map(|r| r.core().status_json().to_owned())
            .unwrap_or_default()
    }
    /// The status cut to `status.{run_id, requested_level, effective_level,
    /// safety, reason}`, `pose_quality` and `goal.state`.
    pub fn autonomy_brief(&self) -> String {
        self.robot()
            .map(|r| r.core().brief_json().to_owned())
            .unwrap_or_default()
    }
    /// Start a bounded run recording at `path` with a new run identity.
    pub fn begin_recording(&self, path: String, run_id: String) -> Result<(), ControllerError> {
        Ok(self.robot()?.begin_recording(&path, run_id)?)
    }
    /// Close the current recording; succeeds when none is active.
    pub fn end_recording(&self) -> Result<(), ControllerError> {
        Ok(self.robot()?.end_recording()?)
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
        let mut sensors = crate::tests::sensors(1.);
        sensors.imu = Some(ImuReading {
            timestamp: 1.,
            acceleration_forward: 0.,
            acceleration_left: 0.,
            acceleration_up: 0.,
            gyro_roll: 0.,
            gyro_pitch: 0.,
            gyro_yaw: 0.,
        });
        sensors.vio = Some(VioReading {
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
        });
        controller.update(sensors.clone()).unwrap();
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
        controller.update(sensors).unwrap();
        let status: serde_json::Value =
            serde_json::from_str(&controller.autonomy_status()).unwrap();
        assert_eq!(status["status"]["requested_level"], "teleop");
        assert_eq!(status["goal"]["state"], "idle");
        assert_eq!(controller.dashboard_status(), "disconnected");
        assert_eq!(
            controller.robot().unwrap().core().estimate(1.).health,
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
        let output = controller.update(crate::tests::sensors(1.)).unwrap();
        assert_eq!(output.left_effort, 0.);
        assert_eq!(output.right_effort, 0.);
        let status: serde_json::Value =
            serde_json::from_str(&controller.autonomy_status()).unwrap();
        assert_eq!(status["status"]["safety"], "emergency_stop");
    }
}

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
