//! Simulator-side Zenoh client: leased velocity intent out, posed depth frames in.
//!
//! No Terra app opens this session. Zorvane's bridge tests drive the simulator
//! through [`RoverConnection`] and check its depth packets with
//! [`decode_depth_frame`], so this is the Terra half of that wire contract.

use crate::{TransportError, validate};
use std::{
    sync::{Arc, Mutex},
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};
use terra_types::telemetry::{
    AUTONOMY_STATUS, DEFAULT_MAX_BYTES, SEARCH_REPORT, SEARCH_REPORT_MAX_RECORDS, SEARCH_STATUS,
};
use zenoh::Wait;
const LEASE: Duration = Duration::from_millis(250);
/// Maximum cached authority age, seconds, when issuing a new remote mission
/// request. This does not extend manual intent leases or any onboard sensor limit.
const REMOTE_AUTHORITY_CACHE_SECONDS: f64 = 90.0;
#[derive(Default)]
struct State {
    target: Option<(f64, f64, Instant)>,
    target_generation: Option<(String, u64)>,
    actions: std::collections::VecDeque<(String, String)>,
    authority: String,
    authority_received: Option<Instant>,
    search: Option<(serde_json::Value, Instant)>,
    reports: std::collections::BTreeMap<(String, String, String), (serde_json::Value, Instant)>,
    shutdown: bool,
    error: Option<String>,
}
/// Thread-backed client for one rover’s leased velocity intent and latest telemetry.
/// The worker publishes targets at 20 Hz, defaulting to zero after 250 ms without
/// a refresh. Actions are queued separately. Dropping the client joins the worker.
///
/// ```no_run
/// # fn main() -> Result<(), terra_transport::TransportError> {
/// let rover = terra_transport::RoverConnection::connect(
///     "tcp/127.0.0.1:7447", "terra/rover", 9,
/// )?;
/// rover.set_target(0.2, 0.0)?;
/// rover.disconnect();
/// # Ok(())
/// # }
/// ```
pub struct RoverConnection {
    state: Arc<Mutex<State>>,
    depth: Arc<Mutex<DepthInbox>>,
    worker: Mutex<Option<JoinHandle<()>>>,
}
fn payload(linear: f64, angular: f64) -> Result<String, TransportError> {
    if !linear.is_finite() || !angular.is_finite() || linear.abs() > 2.0 || angular.abs() > 2.0 {
        return Err(TransportError::InvalidTarget);
    }
    Ok(serde_json::json!({"linear": linear,"angular": angular}).to_string())
}
/// Latest simulator depth frame whose header carries an exposure-aligned pose.
/// Intrinsics use terra-mapping's integer-pixel principal point: documented
/// pixel-centre `(width/2, height/2)` minus 0.5.
#[derive(Clone, Debug)]
pub struct RemoteDepthFrame {
    /// Source depth-frame sequence number.
    pub sequence: u64,
    /// Exposure time in source-clock seconds.
    pub timestamp: f64,
    /// Depth image width, pixels.
    pub width: u32,
    /// Depth image height, pixels.
    pub height: u32,
    /// Horizontal focal length, pixels (equal to `fy` for this wire format).
    pub fx: f64,
    /// Vertical focal length, pixels, derived from vertical field of view.
    pub fy: f64,
    /// Horizontal principal point, integer-pixel coordinates: width / 2 − 0.5.
    pub cx: f64,
    /// Vertical principal point, integer-pixel coordinates: height / 2 − 0.5.
    pub cy: f64,
    /// Optical camera origin world X, metres.
    pub camera_x: f64,
    /// Optical camera origin world Y, metres.
    pub camera_y: f64,
    /// Optical camera origin world Z, metres.
    pub camera_z: f64,
    /// Optical-camera-to-world quaternion X component, as received.
    pub quaternion_x: f64,
    /// Optical-camera-to-world quaternion Y component, as received.
    pub quaternion_y: f64,
    /// Optical-camera-to-world quaternion Z component, as received.
    pub quaternion_z: f64,
    /// Optical-camera-to-world quaternion scalar component, as received.
    pub quaternion_w: f64,
    /// Body world X, metres; falls back to camera X if body pose is omitted.
    pub body_x: f64,
    /// Body world Y, metres; falls back to camera Y if body pose is omitted.
    pub body_y: f64,
    /// Body heading from world +X toward +Y, radians; optical-forward fallback if omitted.
    pub body_yaw: f64,
    /// Row-major little-endian decoded axial depths, metres; NaNs are preserved.
    pub depth_metres: Vec<f32>,
}
struct DepthInbox {
    latest: Option<RemoteDepthFrame>,
    accepted: u64,
}
fn json_finite(value: Option<&serde_json::Value>) -> Option<f64> {
    let number = value?.as_f64()?;
    number.is_finite().then_some(number)
}
/// Heading of optical +Z in the world. Used when a packet has no body pose.
fn optical_forward_yaw(x: f64, y: f64, z: f64, w: f64) -> Option<f64> {
    let forward =
        terra_types::Quaternion { x, y, z, w }
            .normalized()?
            .rotate(terra_types::Vector3 {
                z: 1.0,
                ..terra_types::Vector3::ZERO
            });
    (forward.x.is_finite() && forward.y.is_finite()).then_some(forward.y.atan2(forward.x))
}
/// Decode a Terra depth packet. Frames without an exposure camera pose return `None`.
/// The packet is a version 1 JSON header, newline, then exactly width × height
/// little-endian `f32` pixels (`32FC1_LE`). Dimensions must be 1..=2048,
/// exposure seconds nonnegative, vertical FOV in (0, π) radians, and pose
/// components finite. Missing body pose falls back to optical-forward heading.
/// Returns `None` for invalid framing, header, geometry, or pose fallback.
/// Pixel values and an explicitly supplied camera quaternion are not normalized
/// or range-checked; NaN depths remain available to downstream filtering.
pub fn decode_depth_frame(bytes: &[u8]) -> Option<RemoteDepthFrame> {
    let split = bytes.iter().position(|byte| *byte == b'\n')?;
    let header: serde_json::Value = serde_json::from_slice(&bytes[..split]).ok()?;
    if header.get("version").and_then(serde_json::Value::as_u64) != Some(1)
        || header.get("encoding").and_then(serde_json::Value::as_str) != Some("32FC1_LE")
    {
        return None;
    }
    let width = u32::try_from(header.get("width")?.as_u64()?).ok()?;
    let height = u32::try_from(header.get("height")?.as_u64()?).ok()?;
    if width == 0 || height == 0 || width > 2048 || height > 2048 {
        return None;
    }
    let count = width as usize * height as usize;
    if count > 4_194_304 {
        return None;
    }
    let pixels = &bytes[split + 1..];
    if pixels.len() != count * 4 {
        return None;
    }
    let timestamp = json_finite(header.get("exposure_time"))?;
    if timestamp < 0.0 {
        return None;
    }
    let vertical_fov = json_finite(header.get("vertical_fov"))?;
    if vertical_fov <= 0.0 || vertical_fov >= std::f64::consts::PI {
        return None;
    }
    let camera = header.get("camera")?;
    let camera_x = json_finite(camera.get("x"))?;
    let camera_y = json_finite(camera.get("y"))?;
    let camera_z = json_finite(camera.get("z"))?;
    let quaternion_x = json_finite(camera.get("qx"))?;
    let quaternion_y = json_finite(camera.get("qy"))?;
    let quaternion_z = json_finite(camera.get("qz"))?;
    let quaternion_w = json_finite(camera.get("qw"))?;
    let (body_x, body_y, body_yaw) = if let Some(body) = header.get("body") {
        (
            json_finite(body.get("x"))?,
            json_finite(body.get("y"))?,
            json_finite(body.get("yaw"))?,
        )
    } else {
        (
            camera_x,
            camera_y,
            optical_forward_yaw(quaternion_x, quaternion_y, quaternion_z, quaternion_w)?,
        )
    };
    let fy = height as f64 / (2.0 * (vertical_fov / 2.0).tan());
    if !fy.is_finite() || fy < 1e-6 {
        return None;
    }
    let mut depth_metres = Vec::with_capacity(count);
    for chunk in pixels.as_chunks::<4>().0 {
        depth_metres.push(f32::from_le_bytes(*chunk));
    }
    Some(RemoteDepthFrame {
        sequence: header.get("sequence")?.as_u64()?,
        timestamp,
        width,
        height,
        fx: fy,
        fy,
        cx: width as f64 / 2.0 - 0.5,
        cy: height as f64 / 2.0 - 0.5,
        camera_x,
        camera_y,
        camera_z,
        quaternion_x,
        quaternion_y,
        quaternion_z,
        quaternion_w,
        body_x,
        body_y,
        body_yaw,
        depth_metres,
    })
}
impl RoverConnection {
    /// Opens a direct TCP client. Run off the UI thread; connection timeout is 2 seconds.
    /// Requires a valid `tcp/` or `tls/` endpoint of at most 512 bytes and a
    /// non-wildcard Zenoh key prefix of at most 256 bytes. Uses explicit discovery-free
    /// client configuration. Returns `InvalidSettings` for validation failure or
    /// `Network` for configuration, connection, or worker creation failure.
    pub fn connect(endpoint: &str, prefix: &str, rover_id: u64) -> Result<Self, TransportError> {
        validate(endpoint, prefix)?;
        let mut config = zenoh::Config::default();
        let values = [
            ("mode", "\"client\"".to_string()),
            (
                "connect/endpoints",
                serde_json::json!([endpoint]).to_string(),
            ),
            ("connect/timeout_ms", "2000".to_string()),
            ("connect/exit_on_failure", "true".to_string()),
            ("scouting/multicast/enabled", "false".to_string()),
        ];
        for (key, value) in values {
            config
                .insert_json5(key, &value)
                .map_err(|e| TransportError::Network(e.to_string()))?;
        }
        let session = zenoh::open(config)
            .wait()
            .map_err(|e| TransportError::Network(e.to_string()))?;
        let state = Arc::new(Mutex::new(State::default()));
        let shared = state.clone();
        let depth = Arc::new(Mutex::new(DepthInbox {
            latest: None,
            accepted: 0,
        }));
        let inbox = depth.clone();
        let key = format!("{prefix}/{rover_id}/teleop");
        let base = format!("{prefix}/{rover_id}");
        let depth_key = format!("{prefix}/{rover_id}/camera/depth");
        let worker = thread::Builder::new()
            .name("terra-mobile-zenoh".into())
            .spawn(move || {
                let subscriber = session
                    .declare_subscriber(depth_key)
                    .callback(move |sample| {
                        if let Some(frame) = decode_depth_frame(&sample.payload().to_bytes()) {
                            let mut inbox = inbox.lock().unwrap();
                            if inbox
                                .latest
                                .as_ref()
                                .is_none_or(|previous| frame.sequence > previous.sequence)
                            {
                                inbox.latest = Some(frame);
                                inbox.accepted += 1;
                            }
                        }
                    })
                    .wait();
                let subscriber = match subscriber {
                    Ok(subscriber) => subscriber,
                    Err(error) => {
                        shared.lock().unwrap().error = Some(error.to_string());
                        shared.lock().unwrap().shutdown = true;
                        let _ = session.close().wait();
                        return;
                    }
                };
                let status_shared = shared.clone();
                let authority = session
                    .declare_subscriber(format!("{base}/**"))
                    .callback(move |sample| {
                        let key = sample.key_expr().as_str();
                        let Some(topic) = [AUTONOMY_STATUS, SEARCH_STATUS, SEARCH_REPORT]
                            .into_iter()
                            .find(|t| key.strip_suffix(t).is_some_and(|k| k.ends_with('/')))
                        else {
                            return;
                        };
                        let bytes = sample.payload().to_bytes();
                        if bytes.len() <= DEFAULT_MAX_BYTES
                            && let Ok(text) = String::from_utf8(bytes.to_vec())
                        {
                            let mut state = status_shared.lock().unwrap();
                            if topic == SEARCH_REPORT {
                                if let Ok(value) = serde_json::from_str::<serde_json::Value>(&text)
                                {
                                    let ids = (
                                        value["run_id"].as_str(),
                                        value["search_id"].as_str(),
                                        value["report_id"].as_str(),
                                    );
                                    if let (Some(run), Some(search), Some(report)) = ids {
                                        state
                                            .reports
                                            .retain(|_, (_, at)| at.elapsed().as_secs_f64() < 2.5);
                                        let key = (run.into(), search.into(), report.into());
                                        if state.reports.len() < SEARCH_REPORT_MAX_RECORDS
                                            || state.reports.contains_key(&key)
                                        {
                                            state.reports.insert(key, (value, Instant::now()));
                                        }
                                    }
                                }
                            } else if topic == SEARCH_STATUS {
                                if let Ok(value) = serde_json::from_str::<serde_json::Value>(&text)
                                {
                                    state.search = Some((value, Instant::now()));
                                }
                            } else {
                                let previous: serde_json::Value =
                                    serde_json::from_str(&state.authority).unwrap_or_default();
                                let current: serde_json::Value =
                                    serde_json::from_str(&text).unwrap_or_default();
                                if previous["run_id"] != current["run_id"]
                                    || previous["revision"] != current["revision"]
                                {
                                    state.target = None;
                                    state.target_generation = None;
                                }
                                state.authority = text;
                                state.authority_received = Some(Instant::now());
                            }
                        }
                    })
                    .wait();
                let authority = match authority {
                    Ok(authority) => authority,
                    Err(error) => {
                        shared.lock().unwrap().error = Some(error.to_string());
                        shared.lock().unwrap().shutdown = true;
                        drop(subscriber);
                        let _ = session.close().wait();
                        return;
                    }
                };
                let operator_session_id = format!(
                    "phone-ui-{}",
                    std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .unwrap_or_default()
                        .as_nanos()
                );
                let mut sequence = 0_u64;
                loop {
                    let actions = std::mem::take(&mut shared.lock().unwrap().actions);
                    for (kind, payload) in actions {
                        if let Err(e) = session.put(format!("{base}/{kind}"), payload).wait() {
                            shared.lock().unwrap().error = Some(e.to_string());
                        }
                    }

                    let (shutdown, linear, angular, generation) = {
                        let state = shared.lock().unwrap();
                        let target = state.target.filter(|(_, _, t)| t.elapsed() < LEASE);
                        let (linear, angular) = target.map_or((0.0, 0.0), |(v, w, _)| (v, w));
                        (
                            state.shutdown,
                            if state.shutdown { 0.0 } else { linear },
                            if state.shutdown { 0.0 } else { angular },
                            state.target_generation.clone(),
                        )
                    };
                    let mut bytes = payload(linear, angular).expect("validated target");
                    if let Some((run, revision)) = generation {
                        sequence = sequence.saturating_add(1);
                        let mut value: serde_json::Value =
                            serde_json::from_str(&bytes).expect("validated target");
                        value["run_id"] = serde_json::json!(run);
                        value["authority_revision"] = serde_json::json!(revision);
                        value["operator_session_id"] = serde_json::json!(operator_session_id);
                        value["sequence"] = serde_json::json!(sequence);
                        bytes = value.to_string();
                    }
                    if let Err(error) = session
                        .put(key.clone(), bytes)
                        .congestion_control(zenoh::qos::CongestionControl::Drop)
                        .wait()
                    {
                        shared.lock().unwrap().error = Some(error.to_string());
                        break;
                    }
                    if shutdown || session.is_closed() {
                        break;
                    }
                    thread::sleep(Duration::from_millis(50));
                }
                drop(authority);
                drop(subscriber);
                let _ = session.close().wait();
                shared.lock().unwrap().shutdown = true;
            })
            .map_err(|e| TransportError::Network(e.to_string()))?;
        Ok(Self {
            state,
            depth,
            worker: Mutex::new(Some(worker)),
        })
    }
    /// Consumes the newest posed depth frame. Older unconsumed frames are dropped.
    pub fn take_depth(&self) -> Option<RemoteDepthFrame> {
        self.depth.lock().unwrap().latest.take()
    }
    /// Refresh the 250 ms command lease. Targets are limited to ±2 m/s and ±2 rad/s.
    /// Returns `InvalidTarget` for nonfinite/out-of-range velocities and revokes
    /// the previous target; returns `Closed` when the worker is shutting down.
    /// Success refreshes intent only and does not guarantee rover motion.
    pub fn set_target(&self, linear: f64, angular: f64) -> Result<(), TransportError> {
        // Invalid targets revoke the previous lease immediately.
        if let Err(error) = payload(linear, angular) {
            self.state.lock().unwrap().target = None;
            return Err(error);
        }
        let mut state = self.state.lock().unwrap();
        if state.shutdown {
            return Err(TransportError::Closed);
        }
        let authority: serde_json::Value =
            serde_json::from_str(&state.authority).unwrap_or_default();
        state.target_generation = if state
            .authority_received
            .is_some_and(|at| at.elapsed().as_secs_f64() < 0.5)
        {
            authority["run_id"]
                .as_str()
                .zip(authority["revision"].as_u64())
                .map(|(run, revision)| (run.into(), revision))
        } else {
            None
        };
        state.target = Some((linear, angular, Instant::now()));
        Ok(())
    }
    /// Queue an action suffix and JSON payload for the worker. Accepts every
    /// [`terra_types::requests`] topic except `teleop` (sent with
    /// [`Self::set_target`]) and `navigation/settings`; JSON must fit
    /// [`terra_types::requests::MAX_REQUEST_BYTES`].
    ///
    /// # Errors
    /// Returns `InvalidTarget` for topic/JSON/size rejection, `Closed` after shutdown,
    /// or `Network` when the 64-action queue is full. Success means queued, not delivered.
    pub fn send_action(&self, kind: &str, payload: &str) -> Result<(), TransportError> {
        use terra_types::requests::{self, RequestKind};
        let request = RequestKind::find(kind)
            .filter(|k| ![requests::TELEOP, requests::NAVIGATION_SETTINGS].contains(&k.topic));
        let Some(request) = request.filter(|_| payload.len() <= requests::MAX_REQUEST_BYTES) else {
            return Err(TransportError::InvalidTarget);
        };
        let mut value: serde_json::Value =
            serde_json::from_str(payload).map_err(|_| TransportError::InvalidTarget)?;
        let mut state = self.state.lock().unwrap();
        if state.shutdown {
            return Err(TransportError::Closed);
        }
        if state.actions.len() >= 64 {
            return Err(TransportError::Network("action queue full".into()));
        }
        if !request.neutral(&value) {
            let authority: serde_json::Value =
                serde_json::from_str(&state.authority).unwrap_or_default();
            if !state
                .authority_received
                .is_some_and(|at| at.elapsed().as_secs_f64() < REMOTE_AUTHORITY_CACHE_SECONDS)
                || !authority["run_id"].is_string()
                || authority["revision"].as_u64().is_none()
                || (value.get("run_id").is_some() && value["run_id"] != authority["run_id"])
                || (value.get("authority_revision").is_some()
                    && value["authority_revision"] != authority["revision"])
            {
                return Err(TransportError::Network(
                    "current vehicle authority required".into(),
                ));
            }
            value["run_id"] = authority["run_id"].clone();
            value["authority_revision"] = authority["revision"].clone();
        }
        state.actions.push_back((kind.into(), value.to_string()));
        Ok(())
    }
    /// Return cached autonomy JSON augmented with receipt `age` seconds, optional
    /// search state and `search_age`, and reports received within 2.5 seconds.
    /// Malformed authority text falls back to an object; a non-object yields `{}`.
    /// Missing receipt times are null. This is telemetry, not an authority decision.
    pub fn autonomy_status(&self) -> String {
        let state = self.state.lock().unwrap();
        let mut value: serde_json::Value =
            serde_json::from_str(&state.authority).unwrap_or_else(|_| serde_json::json!({}));
        if !value.is_object() {
            return "{}".into();
        }
        value["age"] =
            serde_json::json!(state.authority_received.map(|t| t.elapsed().as_secs_f64()));
        if let Some((search, at)) = state.search.as_ref() {
            value["search"] = search.clone();
            value["search_age"] = serde_json::json!(at.elapsed().as_secs_f64());
        }
        value["reports"] = serde_json::json!(
            state
                .reports
                .values()
                .filter(|(_, at)| at.elapsed().as_secs_f64() < 2.5)
                .map(|(v, _)| v)
                .collect::<Vec<_>>()
        );
        value.to_string()
    }
    /// Return a human-readable worker error, disconnection, or session/frame-count summary.
    /// An open session does not imply that the rover accepted the latest command.
    pub fn status(&self) -> String {
        let state = self.state.lock().unwrap();
        if let Some(error) = &state.error {
            format!("Transport error: {error}")
        } else if state.shutdown {
            "Disconnected".into()
        } else {
            let frames = self.depth.lock().unwrap().accepted;
            format!("Zenoh session open · commands at 20 Hz · depth frames {frames}")
        }
    }
    /// Worker sends zero before closing. Rover-side 500 ms watchdog remains independent.
    /// Requests shutdown and joins the worker; repeated calls are harmless.
    /// The zero publish is best effort and can fail with the network.
    pub fn disconnect(&self) {
        self.state.lock().unwrap().shutdown = true;
        if let Some(worker) = self.worker.lock().unwrap().take() {
            let _ = worker.join();
        }
    }
}
impl Drop for RoverConnection {
    fn drop(&mut self) {
        self.disconnect();
    }
}
/// Deterministic contract and regression scenarios for this module.
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    /// Checks the velocity JSON shape and rejection of NaN or excessive speed.
    fn validates_target_contract() {
        let p: serde_json::Value = serde_json::from_str(&payload(1.0, 0.3).unwrap()).unwrap();
        assert_eq!(p, serde_json::json!({"linear": 1.0,"angular": 0.3}));
        assert!(payload(f64::NAN, 0.0).is_err());
        assert!(payload(2.1, 0.0).is_err());
    }
    #[test]
    /// Uses a loopback peer to observe rover-scoped motion, expiry of the 250 ms lease,
    /// then a final zero on disconnect. Ignored by default because local TCP sockets are required.
    fn publishes_selected_rover_expires_lease_and_disconnects_with_zero() {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let endpoint = format!("tcp/127.0.0.1:{}", listener.local_addr().unwrap().port());
        drop(listener);
        let mut config = zenoh::Config::default();
        config.insert_json5("mode", "\"peer\"").unwrap();
        config
            .insert_json5(
                "listen/endpoints",
                &serde_json::json!([endpoint]).to_string(),
            )
            .unwrap();
        config
            .insert_json5("scouting/multicast/enabled", "false")
            .unwrap();
        let server = zenoh::open(config).wait().unwrap();
        let subscriber = server
            .declare_subscriber("terra/rover/*/teleop")
            .wait()
            .unwrap();
        let client = RoverConnection::connect(&endpoint, "terra/rover", 7).unwrap();
        client.set_target(1.0, 0.3).unwrap();
        let deadline = Instant::now() + Duration::from_secs(3);
        let mut moving = false;
        let mut stopped = false;
        while Instant::now() < deadline {
            if let Ok(Some(sample)) = subscriber.recv_timeout(Duration::from_millis(100)) {
                assert_eq!(sample.key_expr().as_str(), "terra/rover/7/teleop");
                let p: serde_json::Value =
                    serde_json::from_slice(&sample.payload().to_bytes()).unwrap();
                if p["linear"] == 1.0 {
                    moving = true;
                } else if moving && p["linear"] == 0.0 {
                    stopped = true;
                    break;
                }
            }
        }
        assert!(
            moving && stopped,
            "must publish motion then expire lease to zero"
        );
        client.set_target(-0.5, 0.0).unwrap();
        thread::sleep(Duration::from_millis(100));
        client.disconnect();
        assert_eq!(client.status(), "Disconnected");
        assert!(client.set_target(1.0, 0.0).is_err());
        let mut last = None;
        while let Ok(Some(sample)) = subscriber.recv_timeout(Duration::from_millis(100)) {
            last = Some(
                serde_json::from_slice::<serde_json::Value>(&sample.payload().to_bytes()).unwrap(),
            );
        }
        assert_eq!(last.unwrap()["linear"], 0.0);
        server.close().wait().unwrap();
    }
    /// Build a one-pixel version 1 little-endian packet with explicit exposure, optical pose,
    /// and body pose; `sequence` and axial depth in metres vary across scenarios.
    fn posed_depth_packet(sequence: u64, metres: f32) -> Vec<u8> {
        let header = serde_json::json!({
            "rover_id": 9,
            "version": 1,
            "width": 1,
            "height": 1,
            "sequence": sequence,
            "received_at": 1.0,
            "encoding": "32FC1_LE",
            "vertical_fov": 60.0_f64.to_radians(),
            "near": 0.05,
            "far": 30.0,
            "exposure_time": 0.5,
            "camera": {"x": 0.0, "y": 0.0, "z": 0.5, "qx": -0.5, "qy": 0.5, "qz": -0.5, "qw": 0.5},
            "body": {"x": 0.0, "y": 0.0, "yaw": 0.0}
        });
        let mut bytes = serde_json::to_vec(&header).unwrap();
        bytes.push(b'\n');
        bytes.extend_from_slice(&metres.to_le_bytes());
        bytes
    }
    #[test]
    /// Checks exposure-time/pose/intrinsic decoding, missing-pose rejection, and optical-forward
    /// body-yaw fallback while preserving NaN depth pixels.
    fn decodes_exposure_pose_and_rejects_frames_without_it() {
        let frame = decode_depth_frame(&posed_depth_packet(3, 2.0)).unwrap();
        assert_eq!(frame.sequence, 3);
        assert!((frame.timestamp - 0.5).abs() < 1e-9);
        assert!((frame.camera_z - 0.5).abs() < 1e-9);
        assert!(frame.body_yaw.abs() < 1e-9);
        assert!((frame.cx).abs() < 1e-9 && frame.cy.abs() < 1e-9);
        let expected_fy = 1.0 / (2.0 * (60.0_f64.to_radians() / 2.0).tan());
        assert!((frame.fy - expected_fy).abs() < 1e-9);
        assert_eq!(frame.depth_metres, vec![2.0]);
        let bare = posed_depth_packet(1, 2.0);
        let split = bare.iter().position(|byte| *byte == b'\n').unwrap();
        let mut header: serde_json::Value = serde_json::from_slice(&bare[..split]).unwrap();
        header.as_object_mut().unwrap().remove("camera");
        header.as_object_mut().unwrap().remove("body");
        header.as_object_mut().unwrap().remove("exposure_time");
        let mut without_pose = serde_json::to_vec(&header).unwrap();
        without_pose.push(b'\n');
        without_pose.extend_from_slice(&bare[split + 1..]);
        assert!(decode_depth_frame(&without_pose).is_none());
        let mut no_body = posed_depth_packet(2, f32::NAN);
        let split = no_body.iter().position(|byte| *byte == b'\n').unwrap();
        let mut header: serde_json::Value = serde_json::from_slice(&no_body[..split]).unwrap();
        header.as_object_mut().unwrap().remove("body");
        no_body = serde_json::to_vec(&header).unwrap();
        no_body.push(b'\n');
        no_body.extend_from_slice(&f32::NAN.to_le_bytes());
        let fallback = decode_depth_frame(&no_body).unwrap();
        assert!(
            fallback.body_yaw.abs() < 1e-6,
            "optical +Z yaw {}",
            fallback.body_yaw
        );
        assert!(fallback.depth_metres[0].is_nan());
    }
    #[test]
    /// Publishes successive one-pixel depth packets over loopback and verifies consume-once
    /// retrieval plus frame-count status. Ignored by default because TCP sockets are required.
    fn returns_each_posed_depth_frame_once() {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let endpoint = format!("tcp/127.0.0.1:{}", listener.local_addr().unwrap().port());
        drop(listener);
        let mut config = zenoh::Config::default();
        config.insert_json5("mode", "\"peer\"").unwrap();
        config
            .insert_json5(
                "listen/endpoints",
                &serde_json::json!([endpoint]).to_string(),
            )
            .unwrap();
        config
            .insert_json5("scouting/multicast/enabled", "false")
            .unwrap();
        let server = zenoh::open(config).wait().unwrap();
        let client = RoverConnection::connect(&endpoint, "terra/rover", 9).unwrap();
        // The subscriber is declared on the worker thread after connect returns, so publish until it is live.
        // Republish until the asynchronous subscriber returns the requested sequence, bounded to three seconds.
        let wait_for = |sequence: u64, metres: f32| {
            let deadline = Instant::now() + Duration::from_secs(3);
            while Instant::now() < deadline {
                server
                    .put(
                        "terra/rover/9/camera/depth",
                        posed_depth_packet(sequence, metres),
                    )
                    .congestion_control(zenoh::qos::CongestionControl::Drop)
                    .wait()
                    .unwrap();
                if let Some(frame) = client.take_depth()
                    && frame.sequence == sequence
                {
                    return frame;
                }
                thread::sleep(Duration::from_millis(20));
            }
            panic!(
                "timed out waiting for depth sequence {sequence}: {}",
                client.status()
            );
        };
        let first = wait_for(1, 2.0);
        assert_eq!(first.depth_metres, vec![2.0]);
        assert!(client.take_depth().is_none());
        let second = wait_for(2, 3.5);
        assert_eq!(second.depth_metres, vec![3.5]);
        assert!((second.camera_z - 0.5).abs() < 1e-6);
        assert!(client.take_depth().is_none());
        assert!(client.status().contains("depth frames"));
        client.disconnect();
        server.close().wait().unwrap();
    }
}
