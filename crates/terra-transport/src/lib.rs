//! Leased, latest-command-only Zenoh velocity publisher for Terra.
use std::{
    sync::{Arc, Mutex},
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};
use zenoh::Wait;
const LEASE: Duration = Duration::from_millis(250);
#[derive(Debug)]
pub enum TransportError {
    InvalidSettings,
    InvalidTarget,
    Closed,
    Network(String),
}
impl std::fmt::Display for TransportError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for TransportError {}
#[derive(Default)]
struct State {
    target: Option<(f64, f64, Instant)>,
    actions: std::collections::VecDeque<(String, String)>,
    authority: String,
    shutdown: bool,
    error: Option<String>,
}
pub struct RoverConnection {
    state: Arc<Mutex<State>>,
    depth: Arc<Mutex<DepthInbox>>,
    worker: Mutex<Option<JoinHandle<()>>>,
}
fn validate(endpoint: &str, prefix: &str) -> Result<(), TransportError> {
    if endpoint.len() > 512
        || !endpoint.starts_with("tcp/")
        || endpoint.contains(['\n', '\r', '#', '?'])
        || endpoint.parse::<zenoh::config::EndPoint>().is_err()
        || prefix.len() > 256
        || prefix.contains('*')
        || zenoh::key_expr::KeyExpr::new(prefix).is_err()
    {
        return Err(TransportError::InvalidSettings);
    }
    Ok(())
}
fn payload(linear: f64, angular: f64) -> Result<String, TransportError> {
    if !linear.is_finite() || !angular.is_finite() || linear.abs() > 2.0 || angular.abs() > 2.0 {
        return Err(TransportError::InvalidTarget);
    }
    Ok(serde_json::json!({"linear":linear,"angular":angular}).to_string())
}
/// Latest simulator depth frame whose header carries an exposure-aligned pose.
/// Intrinsics use terra-mapping's integer-pixel principal point: documented
/// pixel-centre `(width/2, height/2)` minus 0.5.
#[derive(Clone, Debug)]
pub struct RemoteDepthFrame {
    pub sequence: u64,
    pub timestamp: f64,
    pub width: u32,
    pub height: u32,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub camera_x: f64,
    pub camera_y: f64,
    pub camera_z: f64,
    pub quaternion_x: f64,
    pub quaternion_y: f64,
    pub quaternion_z: f64,
    pub quaternion_w: f64,
    pub body_x: f64,
    pub body_y: f64,
    pub body_yaw: f64,
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
/// Yaw of optical +Z after `terra_types::Quaternion::rotate`. Used when a packet has no body pose.
fn optical_forward_yaw(x: f64, y: f64, z: f64, w: f64) -> Option<f64> {
    let norm = x.hypot(y).hypot(z).hypot(w);
    if !norm.is_finite() || norm < 1e-12 {
        return None;
    }
    let (x, y, z, w) = (x / norm, y / norm, z / norm, w / norm);
    // t = 2 * cross(q, (0,0,1)) = (2y, -2x, 0); result = v + w*t + cross(q, t).
    let tx = 2.0 * y;
    let ty = -2.0 * x;
    let fx = w * tx + 2.0 * x * z;
    let fy = w * ty + 2.0 * y * z;
    (fx.is_finite() && fy.is_finite()).then_some(fy.atan2(fx))
}
/// Decode a Terra depth packet. Frames without an exposure camera pose return `None`.
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
        let key = format!("{prefix}/{rover_id}/cmd_vel");
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
                    .declare_subscriber(format!("{base}/autonomy/status"))
                    .callback(move |sample| {
                        let bytes = sample.payload().to_bytes();
                        if bytes.len() <= 65536
                            && let Ok(text) = String::from_utf8(bytes.to_vec())
                        {
                            status_shared.lock().unwrap().authority = text;
                        }
                    })
                    .wait()
                    .ok();
                loop {
                    let actions = std::mem::take(&mut shared.lock().unwrap().actions);
                    for (kind, payload) in actions {
                        if let Err(e) = session.put(format!("{base}/{kind}"), payload).wait() {
                            shared.lock().unwrap().error = Some(e.to_string());
                        }
                    }

                    let (shutdown, linear, angular) = {
                        let state = shared.lock().unwrap();
                        let target = state.target.filter(|(_, _, t)| t.elapsed() < LEASE);
                        let (linear, angular) = target.map_or((0.0, 0.0), |(v, w, _)| (v, w));
                        (
                            state.shutdown,
                            if state.shutdown { 0.0 } else { linear },
                            if state.shutdown { 0.0 } else { angular },
                        )
                    };
                    let bytes = payload(linear, angular).expect("validated target");
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
        state.target = Some((linear, angular, Instant::now()));
        Ok(())
    }
    pub fn send_action(&self, kind: &str, payload: &str) -> Result<(), TransportError> {
        if !matches!(kind, "autonomy" | "safety" | "goal" | "goal/decision")
            || payload.len() > 2048
            || serde_json::from_str::<serde_json::Value>(payload).is_err()
        {
            return Err(TransportError::InvalidTarget);
        }
        let mut state = self.state.lock().unwrap();
        if state.shutdown {
            return Err(TransportError::Closed);
        }
        if state.actions.len() >= 64 {
            return Err(TransportError::Network("action queue full".into()));
        }
        state.actions.push_back((kind.into(), payload.into()));
        Ok(())
    }
    pub fn autonomy_status(&self) -> String {
        self.state.lock().unwrap().authority.clone()
    }
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
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn validates_topic_endpoint_and_target_contract() {
        assert!(validate("tcp/127.0.0.1:7447", "terra/rover").is_ok());
        assert!(validate("udp/127.0.0.1:7447", "terra/rover").is_err());
        assert!(validate("tcp/127.0.0.1:7447", "terra/*").is_err());
        let p: serde_json::Value = serde_json::from_str(&payload(1.0, 0.3).unwrap()).unwrap();
        assert_eq!(p, serde_json::json!({"linear":1.0,"angular":0.3}));
        assert!(payload(f64::NAN, 0.0).is_err());
        assert!(payload(2.1, 0.0).is_err());
    }
    #[test]
    #[ignore = "requires local TCP sockets"]
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
            .declare_subscriber("terra/rover/*/cmd_vel")
            .wait()
            .unwrap();
        let client = RoverConnection::connect(&endpoint, "terra/rover", 7).unwrap();
        client.set_target(1.0, 0.3).unwrap();
        let deadline = Instant::now() + Duration::from_secs(3);
        let mut moving = false;
        let mut stopped = false;
        while Instant::now() < deadline {
            if let Ok(Some(sample)) = subscriber.recv_timeout(Duration::from_millis(100)) {
                assert_eq!(sample.key_expr().as_str(), "terra/rover/7/cmd_vel");
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
    #[ignore = "requires local TCP sockets"]
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
mod control_plane;
pub use control_plane::ControlPlane;
/// Bounded intent inbox with protected distinct emergency stops.
pub type ControlAction = (String, Vec<u8>, Instant);
fn stop_token(a: &ControlAction) -> Option<String> {
    if a.0 != "safety" {
        return None;
    }
    let v: serde_json::Value = serde_json::from_slice(&a.1).ok()?;
    let o = v.as_object()?;
    let token = o.get("token")?.as_str()?;
    (o.len() == 2
        && v["action"] == "stop"
        && (1..=64).contains(&token.len())
        && token
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"._:-".contains(&b)))
    .then(|| token.into())
}
fn is_stop(a: &ControlAction) -> bool {
    stop_token(a).is_some()
}
pub fn enqueue_control_action(
    q: &mut std::collections::VecDeque<ControlAction>,
    kind: String,
    payload: Vec<u8>,
    received: Instant,
) {
    let action = (kind, payload, received);
    if let Some(token) = stop_token(&action) {
        q.retain(|a| stop_token(a).as_deref() != Some(&token));
    }
    if q.len() >= 256 {
        if let Some(i) = q.iter().position(|a| !is_stop(a)) {
            q.remove(i);
        } else {
            return;
        }
    }
    q.push_back(action);
}
/// A stop dominates every other request received in the same control tick, including reset.
pub fn drain_control_actions(
    q: &mut std::collections::VecDeque<ControlAction>,
) -> Vec<ControlAction> {
    let mut actions = std::mem::take(q).into_iter().collect::<Vec<_>>();
    actions.sort_by_key(is_stop);
    actions
}
