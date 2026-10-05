//! Nonblocking, latest-frame Zenoh bridge and leased remote differential drive.
use crate::{
    depth_camera::{DepthCamera, DepthCameraConfig, DepthFrame},
    rgb_camera::RgbFrame,
    terra::{DriveCommand, KeyboardControlled, MAX_ROVERS, Rover, RoverFleet, RoverId},
};
use bevy::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, Ordering},
};
use std::time::{Duration, Instant};
use zenoh::Wait;

#[derive(Clone, Resource)]
pub struct ZenohBridgeConfig {
    pub enabled: bool,
    pub prefix: String,
    pub listen: String,
    pub config_file: Option<String>,
    pub frame_rate: f32,
    pub command_timeout: Duration,
}
impl Default for ZenohBridgeConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            prefix: "terra/rover".into(),
            listen: "tcp/127.0.0.1:7447".into(),
            config_file: None,
            frame_rate: 10.0,
            command_timeout: Duration::from_millis(500),
        }
    }
}
impl ZenohBridgeConfig {
    pub fn from_env() -> Self {
        Self {
            enabled: std::env::var("TERRA_ZENOH").map_or(true, |v| v != "0"),
            prefix: std::env::var("TERRA_ZENOH_PREFIX").unwrap_or_else(|_| "terra/rover".into()),
            listen: std::env::var("TERRA_ZENOH_LISTEN")
                .unwrap_or_else(|_| "tcp/127.0.0.1:7447".into()),
            config_file: std::env::var("TERRA_ZENOH_CONFIG").ok(),
            ..default()
        }
    }
    fn validate(&self) -> Result<(), &'static str> {
        if !self.frame_rate.is_finite()
            || self.frame_rate < 0.1
            || self.frame_rate > 60.0
            || self.command_timeout.is_zero()
        {
            return Err("frame rate must be in [0.1,60] and command timeout must be positive");
        }
        if self.prefix.contains('*') || zenoh::key_expr::KeyExpr::new(self.prefix.as_str()).is_err()
        {
            return Err("prefix must be a concrete Zenoh key expression");
        }
        Ok(())
    }
    fn session_config(&self) -> zenoh::Result<zenoh::Config> {
        if let Some(path) = &self.config_file {
            return zenoh::Config::from_file(path);
        }
        let mut config = zenoh::Config::default();
        config.insert_json5("mode", "\"peer\"")?;
        config.insert_json5(
            "listen/endpoints",
            &serde_json::json!([self.listen]).to_string(),
        )?;
        config.insert_json5("scouting/multicast/enabled", "false")?;
        Ok(config)
    }
}
#[derive(Default)]
pub struct TerraZenohPlugin {
    pub config: ZenohBridgeConfig,
}
impl Plugin for TerraZenohPlugin {
    fn build(&self, app: &mut App) {
        if !self.config.enabled {
            return;
        }
        self.config
            .validate()
            .expect("invalid Terra Zenoh configuration");
        let shared = Arc::new(Shared::default());
        let thread_shared = shared.clone();
        let config = self.config.clone();
        std::thread::Builder::new()
            .name("terra-zenoh".into())
            .spawn(move || {
                if let Err(error) = run_transport(&config, &thread_shared) {
                    warn!("Terra Zenoh transport stopped: {error}");
                }
                thread_shared.connected.store(false, Ordering::Release);
            })
            .expect("could not start Zenoh worker");
        app.insert_resource(self.config.clone())
            .insert_resource(Bridge { shared })
            .add_systems(
                PreUpdate,
                (
                    receive_fleet_request.before(crate::terra::reconcile_fleet),
                    claim_rovers.after(crate::terra::reconcile_fleet),
                ),
            )
            .add_systems(FixedUpdate, apply_remote_commands)
            .add_systems(Update, publish_frames);
    }
}
#[derive(Default)]
struct Outgoing {
    frames: BTreeMap<(u64, &'static str), Vec<u8>>,
    fleet: Option<Vec<u8>>,
}
type LeasedCommand = Option<(DriveCommand, Instant)>;
#[derive(Default)]
struct Shared {
    commands: Mutex<BTreeMap<u64, LeasedCommand>>,
    fleet_request: Mutex<Option<usize>>,
    outgoing: Mutex<Outgoing>,
    connected: AtomicBool,
    stop: AtomicBool,
}
#[derive(Resource)]
struct Bridge {
    shared: Arc<Shared>,
}
impl Drop for Bridge {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::Release);
    }
}
#[derive(Component)]
struct ZenohControlled;
fn receive_fleet_request(bridge: Res<Bridge>, fleet: Option<ResMut<RoverFleet>>) {
    if let Some(count) = bridge.shared.fleet_request.lock().unwrap().take()
        && let Some(mut fleet) = fleet
    {
        let _ = fleet.set_count(count);
    }
}
fn claim_rovers(
    mut commands: Commands,
    bridge: Res<Bridge>,
    rovers: Query<(Entity, &RoverId, Option<&ZenohControlled>), With<Rover>>,
) {
    let mut incoming = bridge.shared.commands.lock().unwrap();
    let ids: std::collections::BTreeSet<_> = rovers.iter().map(|(_, id, _)| id.0).collect();
    incoming.retain(|id, _| ids.contains(id));
    for (entity, id, controlled) in &rovers {
        incoming.entry(id.0).or_default();
        if controlled.is_none() {
            commands
                .entity(entity)
                .remove::<KeyboardControlled>()
                .insert(ZenohControlled);
        }
    }
}
type RemoteRovers<'w, 's> = Query<
    'w,
    's,
    (&'static RoverId, &'static mut DriveCommand),
    (With<Rover>, With<ZenohControlled>),
>;

fn apply_remote_commands(
    bridge: Res<Bridge>,
    config: Res<ZenohBridgeConfig>,
    keys: Option<Res<ButtonInput<KeyCode>>>,
    mut rovers: RemoteRovers<'_, '_>,
) {
    let ready = bridge.shared.connected.load(Ordering::Acquire)
        && !keys.is_some_and(|keys| keys.pressed(KeyCode::Space));
    let incoming = bridge.shared.commands.lock().unwrap();
    for (id, mut request) in &mut rovers {
        *request = if ready {
            fresh_command(
                incoming.get(&id.0).copied().flatten(),
                Instant::now(),
                config.command_timeout,
            )
        } else {
            DriveCommand::default()
        };
    }
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Twist {
    linear: f32,
    angular: f32,
}
fn decode_command(bytes: &[u8]) -> Option<DriveCommand> {
    if bytes.len() > 2048 {
        return None;
    }
    let Twist { linear, angular } = serde_json::from_slice(bytes).ok()?;
    (linear.is_finite() && angular.is_finite()).then_some(DriveCommand { linear, angular })
}
fn fresh_command(
    command: Option<(DriveCommand, Instant)>,
    now: Instant,
    timeout: Duration,
) -> DriveCommand {
    command
        .filter(|(_, received)| now.saturating_duration_since(*received) < timeout)
        .map(|(command, _)| command)
        .unwrap_or_default()
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct FleetRequest {
    count: usize,
}
fn decode_fleet_request(bytes: &[u8]) -> Option<usize> {
    if bytes.len() > 2048 {
        return None;
    }
    let request: FleetRequest = serde_json::from_slice(bytes).ok()?;
    (request.count <= MAX_ROVERS).then_some(request.count)
}
fn run_transport(config: &ZenohBridgeConfig, shared: &Arc<Shared>) -> zenoh::Result<()> {
    let session = zenoh::open(config.session_config()?).wait()?;
    let incoming = shared.clone();
    let prefix = config.prefix.clone();
    let _subscriber = session
        .declare_subscriber(format!("{}/*/cmd_vel", prefix))
        .callback(move |sample| {
            let key = sample.key_expr().as_str();
            let Some(id) = key
                .strip_prefix(&format!("{prefix}/"))
                .and_then(|key| key.strip_suffix("/cmd_vel"))
                .and_then(|id| id.parse::<u64>().ok())
            else {
                return;
            };
            if let Some(command) = decode_command(&sample.payload().to_bytes())
                && let Some(slot) = incoming.commands.lock().unwrap().get_mut(&id)
            {
                *slot = Some((command, Instant::now()));
            }
        })
        .wait()?;
    let incoming = shared.clone();
    let _fleet_subscriber = session
        .declare_subscriber(format!("{}/fleet/size", config.prefix))
        .callback(move |sample| {
            if let Some(count) = decode_fleet_request(&sample.payload().to_bytes()) {
                *incoming.fleet_request.lock().unwrap() = Some(count);
            }
        })
        .wait()?;
    shared.connected.store(true, Ordering::Release);
    info!(
        "Terra Zenoh ready: {}/<id>/cmd_vel, {}/<id>/camera/{{rgb,depth}} (depth includes exposure pose), {}/fleet/{{size,state}}",
        config.prefix, config.prefix, config.prefix
    );
    while !shared.stop.load(Ordering::Acquire) {
        let outgoing = std::mem::take(&mut *shared.outgoing.lock().unwrap());
        for ((id, kind), payload) in outgoing.frames {
            if !shared.commands.lock().unwrap().contains_key(&id) {
                continue;
            }
            session
                .put(format!("{}/{id}/camera/{kind}", config.prefix), payload)
                .encoding(zenoh::bytes::Encoding::APPLICATION_OCTET_STREAM)
                .congestion_control(zenoh::qos::CongestionControl::Drop)
                .wait()?;
        }
        if let Some(state) = outgoing.fleet {
            session
                .put(format!("{}/fleet/state", config.prefix), state)
                .encoding(zenoh::bytes::Encoding::APPLICATION_JSON)
                .congestion_control(zenoh::qos::CongestionControl::Drop)
                .wait()?;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    shared.connected.store(false, Ordering::Release);
    session.close().wait()
}
#[derive(Serialize)]
struct FrameHeader {
    rover_id: u64,
    version: u8,
    width: u32,
    height: u32,
    sequence: u64,
    received_at: f64,
    encoding: &'static str,
    vertical_fov: f32,
    near: f32,
    far: f32,
    /// Simulation time paired with this GPU copy. Omitted when the frame has no exposure.
    #[serde(skip_serializing_if = "Option::is_none")]
    exposure_time: Option<f64>,
    /// Optical pose in the robotics world (position X/Y/Z, quaternion from optical axes). Depth only.
    #[serde(skip_serializing_if = "Option::is_none")]
    camera: Option<CameraPoseHeader>,
    /// Rover body origin and heading in the same robotics frame, sampled with the exposure.
    #[serde(skip_serializing_if = "Option::is_none")]
    body: Option<BodyPoseHeader>,
}
#[derive(Serialize)]
struct CameraPoseHeader {
    x: f64,
    y: f64,
    z: f64,
    qx: f64,
    qy: f64,
    qz: f64,
    qw: f64,
}
#[derive(Serialize)]
struct BodyPoseHeader {
    x: f64,
    y: f64,
    yaw: f64,
}
fn packet(header: FrameHeader, pixels: &[u8]) -> Option<Vec<u8>> {
    let mut bytes = serde_json::to_vec(&header).ok()?;
    bytes.push(b'\n');
    bytes.extend_from_slice(pixels);
    Some(bytes)
}
#[cfg(test)]
fn encode_depth(frame: &DepthFrame) -> Option<Vec<u8>> {
    encode_depth_with_config(0, frame, &DepthCameraConfig::default())
}
fn encode_depth_with_config(
    id: u64,
    frame: &DepthFrame,
    config: &DepthCameraConfig,
) -> Option<Vec<u8>> {
    if frame.depth_metres.len() != frame.width as usize * frame.height as usize {
        return None;
    }
    let pixels: Vec<u8> = frame
        .depth_metres
        .iter()
        .flat_map(|value| value.to_le_bytes())
        .collect();
    let (exposure_time, camera, body) = frame.exposure.map_or((None, None, None), |exposure| {
        let pose = crate::occupancy_map::camera_pose(&exposure.camera_transform);
        let (x, y, yaw) = crate::occupancy_map::body_pose(&exposure.body_transform);
        (
            Some(exposure.timestamp),
            Some(CameraPoseHeader {
                x: pose.position.x,
                y: pose.position.y,
                z: pose.position.z,
                qx: pose.orientation.x,
                qy: pose.orientation.y,
                qz: pose.orientation.z,
                qw: pose.orientation.w,
            }),
            Some(BodyPoseHeader { x, y, yaw }),
        )
    });
    packet(
        FrameHeader {
            rover_id: id,
            version: 1,
            width: frame.width,
            height: frame.height,
            sequence: frame.sequence,
            received_at: frame.received_at,
            encoding: "32FC1_LE",
            vertical_fov: config.vertical_fov,
            near: config.near,
            far: config.far,
            exposure_time,
            camera,
            body,
        },
        &pixels,
    )
}
#[derive(Default)]
struct PublishState {
    last_publish: Option<Instant>,
    sequences: BTreeMap<u64, (Option<u64>, Option<u64>)>,
    fleet_ids: Vec<u64>,
    last_fleet_publish: Option<Instant>,
}
fn publish_frames(
    bridge: Res<Bridge>,
    config: Res<ZenohBridgeConfig>,
    depth: Query<(&RoverId, &DepthCamera, &DepthFrame)>,
    rgb: Query<(&RoverId, &RgbFrame, &crate::rgb_camera::RgbCamera)>,
    rovers: Query<&RoverId, With<Rover>>,
    mut state: Local<PublishState>,
) {
    let now = Instant::now();
    if !bridge.shared.connected.load(Ordering::Acquire)
        || state.last_publish.is_some_and(|last| {
            now.duration_since(last) < Duration::from_secs_f32(1.0 / config.frame_rate)
        })
    {
        return;
    }
    state.last_publish = Some(now);
    let mut ids: Vec<_> = rovers.iter().map(|id| id.0).collect();
    ids.sort();
    state
        .sequences
        .retain(|id, _| ids.binary_search(id).is_ok());
    let mut outgoing = bridge.shared.outgoing.lock().unwrap();
    outgoing
        .frames
        .retain(|(id, _), _| ids.binary_search(id).is_ok());
    if state.fleet_ids != ids
        || state
            .last_fleet_publish
            .is_none_or(|last| now.duration_since(last) >= Duration::from_secs(1))
    {
        outgoing.fleet = Some(
            serde_json::to_vec(
                &serde_json::json!({"count": ids.len(), "max_count": MAX_ROVERS, "ids": ids}),
            )
            .unwrap(),
        );
        state.fleet_ids = ids.clone();
        state.last_fleet_publish = Some(now);
    }
    for (id, camera, frame) in &depth {
        if ids.binary_search(&id.0).is_err() {
            continue;
        }
        let sequences = state.sequences.entry(id.0).or_default();
        if frame.sequence > 0
            && sequences.0 != Some(frame.sequence)
            && let Some(packet) = encode_depth_with_config(id.0, frame, &camera.config)
        {
            outgoing.frames.insert((id.0, "depth"), packet);
            sequences.0 = Some(frame.sequence);
        }
    }
    for (id, frame, camera) in &rgb {
        if ids.binary_search(&id.0).is_err() {
            continue;
        }
        let sequences = state.sequences.entry(id.0).or_default();
        if frame.sequence > 0
            && sequences.1 != Some(frame.sequence)
            && frame.rgba.len() == frame.width as usize * frame.height as usize * 4
            && let Some(packet) = packet(
                FrameHeader {
                    rover_id: id.0,
                    version: 1,
                    width: frame.width,
                    height: frame.height,
                    sequence: frame.sequence,
                    received_at: frame.received_at,
                    encoding: "RGBA8_SRGB",
                    vertical_fov: camera.config.vertical_fov,
                    near: camera.config.near,
                    far: camera.config.far,
                    exposure_time: None,
                    camera: None,
                    body: None,
                },
                &frame.rgba,
            )
        {
            outgoing.frames.insert((id.0, "rgb"), packet);
            sequences.1 = Some(frame.sequence);
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn accepts_finite_twist_and_rejects_invalid_or_oversized_messages() {
        let command = decode_command(br#"{"linear":1.0,"angular":-0.5}"#).unwrap();
        assert_eq!(command.linear, 1.0);
        assert_eq!(command.angular, -0.5);
        for bytes in [
            b"bad".as_slice(),
            br#"{"linear":1e100,"angular":0}"#,
            br#"{"linear":1}"#,
        ] {
            assert!(decode_command(bytes).is_none());
        }
        assert!(decode_command(&vec![b' '; 2049]).is_none());
    }
    #[test]
    fn timeout_and_missing_commands_stop_the_rover() {
        use std::time::{Duration, Instant};
        let now = Instant::now();
        let timeout = Duration::from_millis(500);
        let command = DriveCommand {
            linear: 1.0,
            angular: 0.2,
        };
        assert_eq!(
            fresh_command(Some((command, now)), now, timeout).linear,
            1.0
        );
        assert_eq!(
            fresh_command(Some((command, now)), now + timeout, timeout).linear,
            0.0
        );
        assert_eq!(fresh_command(None, now, timeout).angular, 0.0);
    }
    #[test]
    fn depth_wire_format_preserves_metres_nan_and_metadata() {
        let frame = crate::depth_camera::DepthFrame {
            width: 2,
            height: 1,
            sequence: 7,
            received_at: 1.25,
            depth_metres: vec![2.5, f32::NAN],
            exposure: None,
        };
        let packet = encode_depth(&frame).unwrap();
        let split = packet.iter().position(|b| *b == b'\n').unwrap();
        let header: serde_json::Value = serde_json::from_slice(&packet[..split]).unwrap();
        assert_eq!(header["encoding"], "32FC1_LE");
        assert_eq!(header["sequence"], 7);
        assert_eq!(header["width"], 2);
        assert_eq!(header["received_at"], 1.25);
        assert!(header.get("camera").is_none());
        assert!(header.get("body").is_none());
        assert!(header.get("exposure_time").is_none());
        let data = &packet[split + 1..];
        assert_eq!(data.len(), 8);
        assert_eq!(f32::from_le_bytes(data[..4].try_into().unwrap()), 2.5);
        assert!(f32::from_le_bytes(data[4..].try_into().unwrap()).is_nan());
    }
    #[test]
    fn depth_packet_pose_round_trips_into_the_phone_decoder() {
        use crate::depth_camera::{DepthCameraConfig, DepthExposure, DepthFrame};
        use crate::occupancy_map::{body_pose, camera_pose};
        use terra_mapping::{CameraIntrinsics, CameraPose, LocalOccupancyMap, MapConfig};
        let camera_transform = GlobalTransform::from(Transform::from_xyz(-0.25, 0.5, -0.25));
        let body_transform = GlobalTransform::from(Transform::from_xyz(0.0, 0.2, -1.0));
        let frame = DepthFrame {
            width: 1,
            height: 1,
            sequence: 4,
            received_at: 3.0,
            depth_metres: vec![2.0],
            exposure: Some(DepthExposure {
                sensor: Entity::PLACEHOLDER,
                timestamp: 1.5,
                camera_transform,
                body_transform,
            }),
        };
        let config = DepthCameraConfig {
            resolution: UVec2::ONE,
            vertical_fov: 60.0_f32.to_radians(),
            ..default()
        };
        let packet = encode_depth_with_config(0, &frame, &config).unwrap();
        let decoded = terra_transport::decode_depth_frame(&packet).expect("posed depth");
        let pose = camera_pose(&camera_transform);
        let (body_x, body_y, body_yaw) = body_pose(&body_transform);
        assert_eq!(decoded.sequence, 4);
        assert!((decoded.timestamp - 1.5).abs() < 1e-9);
        assert!((decoded.camera_x - pose.position.x).abs() < 1e-5);
        assert!((decoded.camera_y - pose.position.y).abs() < 1e-5);
        assert!((decoded.camera_z - pose.position.z).abs() < 1e-5);
        assert!((decoded.quaternion_x - pose.orientation.x).abs() < 1e-5);
        assert!((decoded.quaternion_y - pose.orientation.y).abs() < 1e-5);
        assert!((decoded.quaternion_z - pose.orientation.z).abs() < 1e-5);
        assert!((decoded.quaternion_w - pose.orientation.w).abs() < 1e-5);
        assert!((decoded.body_x - body_x).abs() < 1e-5);
        assert!((decoded.body_y - body_y).abs() < 1e-5);
        assert!((decoded.body_yaw - body_yaw).abs() < 1e-5);
        assert!((decoded.cx).abs() < 1e-9 && decoded.cy.abs() < 1e-9);
        let mut map = LocalOccupancyMap::new(MapConfig {
            pixel_stride: 1,
            ..MapConfig::default()
        })
        .unwrap();
        map.recenter(decoded.body_x, decoded.body_y).unwrap();
        map.integrate_depth(
            decoded.timestamp,
            CameraIntrinsics {
                width: decoded.width,
                height: decoded.height,
                fx: decoded.fx,
                fy: decoded.fy,
                cx: decoded.cx,
                cy: decoded.cy,
            },
            CameraPose {
                position: terra_types::Vector3 {
                    x: decoded.camera_x,
                    y: decoded.camera_y,
                    z: decoded.camera_z,
                },
                orientation: terra_types::Quaternion {
                    x: decoded.quaternion_x,
                    y: decoded.quaternion_y,
                    z: decoded.quaternion_z,
                    w: decoded.quaternion_w,
                },
            },
            &decoded.depth_metres,
        )
        .unwrap();
        let grid = map.snapshot();
        let col = ((2.25 - grid.origin_x) / grid.resolution).floor() as usize;
        let row = ((0.25 - grid.origin_y) / grid.resolution).floor() as usize;
        assert!(
            grid.occupancy[row * grid.width as usize + col] > 50,
            "endpoint (2.25, 0.25) must be occupied"
        );
        assert!(
            terra_transport::decode_depth_frame(
                &encode_depth(&DepthFrame {
                    exposure: None,
                    ..frame
                })
                .unwrap()
            )
            .is_none()
        );
    }
    #[test]
    #[ignore = "requires local TCP sockets; tests two real Zenoh sessions"]
    fn zenoh_round_trip_frames_drive_and_deadman_stop() {
        let socket = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let endpoint = format!("tcp/{}", socket.local_addr().unwrap());
        drop(socket);
        let config = ZenohBridgeConfig {
            listen: endpoint.clone(),
            command_timeout: Duration::from_millis(100),
            ..default()
        };
        let mut app = App::new();
        app.add_plugins(MinimalPlugins)
            .add_plugins(TerraZenohPlugin { config });
        let rover = app
            .world_mut()
            .spawn((
                Rover,
                RoverId(0),
                KeyboardControlled,
                DriveCommand::default(),
            ))
            .id();
        let deadline = Instant::now() + Duration::from_secs(10);
        while !app
            .world()
            .resource::<Bridge>()
            .shared
            .connected
            .load(Ordering::Acquire)
        {
            assert!(Instant::now() < deadline, "bridge failed to open");
            std::thread::sleep(Duration::from_millis(5));
        }
        let mut client_config = zenoh::Config::default();
        client_config.insert_json5("mode", "\"client\"").unwrap();
        client_config
            .insert_json5(
                "connect/endpoints",
                &serde_json::json!([endpoint.clone()]).to_string(),
            )
            .unwrap();
        client_config
            .insert_json5("scouting/multicast/enabled", "false")
            .unwrap();
        let client = zenoh::open(client_config).wait().unwrap();
        let subscriber = client
            .declare_subscriber("terra/rover/0/camera/*")
            .wait()
            .unwrap();
        let camera_config = DepthCameraConfig {
            resolution: UVec2::new(2, 1),
            ..default()
        };
        let depth_entity = app
            .world_mut()
            .spawn((
                RoverId(0),
                DepthCamera {
                    config: camera_config.clone(),
                    depth_texture: default(),
                    preview_texture: default(),
                },
                DepthFrame {
                    width: 2,
                    height: 1,
                    sequence: 1,
                    received_at: 1.0,
                    depth_metres: vec![2.0, f32::NAN],
                    exposure: None,
                },
            ))
            .id();
        let rgb_entity = app
            .world_mut()
            .spawn((
                RoverId(0),
                crate::rgb_camera::RgbCamera {
                    config: camera_config,
                },
                RgbFrame {
                    width: 2,
                    height: 1,
                    sequence: 1,
                    received_at: 1.1,
                    rgba: vec![10, 20, 30, 255, 40, 50, 60, 255],
                },
            ))
            .id();
        let mut python = std::env::var("TERRA_ZENOH_TEST_PYTHON").ok().map(|python| {
            std::process::Command::new(python)
                .arg(concat!(
                    env!("CARGO_MANIFEST_DIR"),
                    "/tools/test_zenoh_client.py"
                ))
                .env("TERRA_TEST_ZENOH_ENDPOINT", endpoint.clone())
                .spawn()
                .expect("could not start Python interoperability test")
        });
        let mut received_rgb = false;
        let mut received_depth = false;
        let mut moved = false;
        while Instant::now() < deadline && !(received_rgb && received_depth && moved) {
            if python.is_none() {
                client
                    .put("terra/rover/0/cmd_vel", r#"{"linear":1.0,"angular":0.25}"#)
                    .wait()
                    .unwrap();
            }
            // Force new frames to cover discovery without retaining old samples.
            app.world_mut()
                .get_mut::<DepthFrame>(depth_entity)
                .unwrap()
                .sequence += 1;
            app.world_mut()
                .get_mut::<RgbFrame>(rgb_entity)
                .unwrap()
                .sequence += 1;
            app.update();
            app.world_mut().run_schedule(FixedUpdate);
            moved |= app.world().get::<DriveCommand>(rover).unwrap().linear == 1.0;
            while let Ok(Some(sample)) = subscriber.try_recv() {
                let payload = sample.payload().to_bytes();
                let split = payload.iter().position(|b| *b == b'\n').unwrap();
                let header: serde_json::Value = serde_json::from_slice(&payload[..split]).unwrap();
                let pixels = &payload[split + 1..];
                assert_eq!(pixels.len(), 8);
                if header["encoding"] == "32FC1_LE" {
                    assert_eq!(f32::from_le_bytes(pixels[..4].try_into().unwrap()), 2.0);
                    assert!(f32::from_le_bytes(pixels[4..].try_into().unwrap()).is_nan());
                    received_depth = true;
                } else {
                    assert_eq!(header["encoding"], "RGBA8_SRGB");
                    assert_eq!(pixels, [10, 20, 30, 255, 40, 50, 60, 255]);
                    received_rgb = true;
                }
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        assert!(
            received_rgb && received_depth && moved,
            "RGB={received_rgb}, depth={received_depth}, drove={moved}"
        );
        if let Some(python) = python.as_mut() {
            assert!(
                python.wait().unwrap().success(),
                "Python interoperability failed"
            );
        }
        assert!(app.world().get::<KeyboardControlled>(rover).is_none());
        std::thread::sleep(Duration::from_millis(150));
        app.world_mut().run_schedule(FixedUpdate);
        assert_eq!(app.world().get::<DriveCommand>(rover).unwrap().linear, 0.0);
        assert_eq!(app.world().get::<DriveCommand>(rover).unwrap().angular, 0.0);
        client.close().wait().unwrap();
        let shared = app.world().resource::<Bridge>().shared.clone();
        drop(app);
        while shared.connected.load(Ordering::Acquire) && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(5));
        }
        assert!(
            !shared.connected.load(Ordering::Acquire),
            "worker did not shut down"
        );
    }
    #[test]
    fn drive_commands_are_isolated_between_rovers() {
        let mut app = App::new();
        let shared = Arc::new(Shared::default());
        shared.connected.store(true, Ordering::Release);
        shared.commands.lock().unwrap().insert(
            0,
            Some((
                DriveCommand {
                    linear: 1.0,
                    angular: 0.0,
                },
                Instant::now(),
            )),
        );
        app.insert_resource(Bridge { shared })
            .insert_resource(ZenohBridgeConfig::default())
            .add_systems(FixedUpdate, apply_remote_commands);
        let first = app
            .world_mut()
            .spawn((
                Rover,
                crate::terra::RoverId(0),
                ZenohControlled,
                DriveCommand::default(),
            ))
            .id();
        let second = app
            .world_mut()
            .spawn((
                Rover,
                crate::terra::RoverId(1),
                ZenohControlled,
                DriveCommand::default(),
            ))
            .id();
        app.world_mut().run_schedule(FixedUpdate);
        assert_eq!(app.world().get::<DriveCommand>(first).unwrap().linear, 1.0);
        assert_eq!(app.world().get::<DriveCommand>(second).unwrap().linear, 0.0);
    }
    #[test]
    fn fleet_requests_validate_count_and_apply_to_the_resource() {
        assert_eq!(decode_fleet_request(br#"{"count":3}"#), Some(3));
        assert_eq!(decode_fleet_request(br#"{"count":0}"#), Some(0));
        for input in [
            br#"{"count":33}"#.as_slice(),
            br#"{"count":-1}"#,
            br#"{"count":1.5}"#,
            b"invalid",
        ] {
            assert!(decode_fleet_request(input).is_none());
        }
        let shared = Arc::new(Shared::default());
        *shared.fleet_request.lock().unwrap() = Some(3);
        let mut app = App::new();
        app.insert_resource(Bridge { shared })
            .init_resource::<RoverFleet>()
            .add_systems(Update, receive_fleet_request);
        app.update();
        assert_eq!(app.world().resource::<RoverFleet>().count(), 3);
    }
    #[test]
    #[ignore = "requires local TCP sockets; verifies live fleet resizing"]
    fn zenoh_resizes_fleet_and_keeps_drive_commands_independent() {
        let socket = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let endpoint = format!("tcp/{}", socket.local_addr().unwrap());
        drop(socket);
        let mut app = App::new();
        app.add_plugins(MinimalPlugins)
            .insert_resource(crate::terra::DriveConfig::default())
            .insert_resource(crate::physics::RoverPhysicsConfig::default())
            .init_resource::<RoverFleet>()
            .add_systems(PreUpdate, crate::terra::reconcile_fleet)
            .add_plugins(TerraZenohPlugin {
                config: ZenohBridgeConfig {
                    listen: endpoint.clone(),
                    ..default()
                },
            });
        let deadline = Instant::now() + Duration::from_secs(10);
        while !app
            .world()
            .resource::<Bridge>()
            .shared
            .connected
            .load(Ordering::Acquire)
        {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(5));
        }
        let mut config = zenoh::Config::default();
        config.insert_json5("mode", "\"client\"").unwrap();
        config
            .insert_json5(
                "connect/endpoints",
                &serde_json::json!([endpoint]).to_string(),
            )
            .unwrap();
        config
            .insert_json5("scouting/multicast/enabled", "false")
            .unwrap();
        let client = zenoh::open(config).wait().unwrap();
        let states = client
            .declare_subscriber("terra/rover/fleet/state")
            .wait()
            .unwrap();
        for target in [3, 1, 2, 0] {
            let mut acknowledged = false;
            while Instant::now() < deadline {
                client
                    .put(
                        "terra/rover/fleet/size",
                        serde_json::json!({"count": target}).to_string(),
                    )
                    .wait()
                    .unwrap();
                app.update();
                app.world_mut().run_schedule(FixedUpdate);
                while let Ok(Some(state)) = states.try_recv() {
                    let state: serde_json::Value =
                        serde_json::from_slice(&state.payload().to_bytes()).unwrap();
                    acknowledged |= state["count"] == target;
                }
                let world = app.world_mut();
                let count = world
                    .query_filtered::<&RoverId, With<Rover>>()
                    .iter(world)
                    .count();
                if count == target && acknowledged {
                    break;
                }
                std::thread::sleep(Duration::from_millis(10));
            }
            let world = app.world_mut();
            let mut ids: Vec<_> = world
                .query_filtered::<&RoverId, With<Rover>>()
                .iter(world)
                .map(|id| id.0)
                .collect();
            ids.sort();
            assert_eq!(ids.len(), target);
            assert!(acknowledged);
            if target == 2 {
                assert_eq!(ids, [0, 3]);
            }
            if target == 3 {
                let mut independent = false;
                while Instant::now() < deadline {
                    client
                        .put("terra/rover/0/cmd_vel", r#"{"linear":1,"angular":0}"#)
                        .wait()
                        .unwrap();
                    client
                        .put("terra/rover/1/cmd_vel", r#"{"linear":-1,"angular":0.5}"#)
                        .wait()
                        .unwrap();
                    app.update();
                    app.world_mut().run_schedule(FixedUpdate);
                    let world = app.world_mut();
                    let mut requests: Vec<_> = world
                        .query_filtered::<(&RoverId, &DriveCommand), With<Rover>>()
                        .iter(world)
                        .map(|(id, command)| (id.0, command.linear))
                        .collect();
                    requests.sort_by_key(|(id, _)| *id);
                    independent = requests == [(0, 1.0), (1, -1.0), (2, 0.0)];
                    if independent {
                        break;
                    }
                    std::thread::sleep(Duration::from_millis(5));
                }
                assert!(independent);
            }
        }
        client.close().wait().unwrap();
    }
    #[test]
    #[ignore = "requires local TCP sockets; mobile adapter drives real Avian physics"]
    fn mobile_adapter_drives_avian_and_disconnect_stops() {
        use avian3d::prelude::*;
        let socket = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let endpoint = format!("tcp/{}", socket.local_addr().unwrap());
        drop(socket);
        let mut app = App::new();
        app.add_plugins((
            MinimalPlugins,
            TransformPlugin,
            bevy::asset::AssetPlugin::default(),
            bevy::mesh::MeshPlugin,
            crate::physics::TerraPhysicsPlugin,
            crate::velocity_controller::TerraVelocityControlPlugin,
            TerraZenohPlugin {
                config: ZenohBridgeConfig {
                    listen: endpoint.clone(),
                    ..default()
                },
            },
        ))
        .insert_resource(crate::terra::DriveConfig::default())
        .insert_resource(Time::<Fixed>::from_hz(100.0))
        .insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
            Duration::from_millis(10),
        ));
        app.finish();
        app.cleanup();
        app.world_mut().spawn((
            RigidBody::Static,
            Collider::cuboid(100.0, 0.2, 100.0),
            Transform::from_xyz(0.0, -0.1, 0.0),
        ));
        let rover = app
            .world_mut()
            .spawn((
                Rover,
                RoverId(9),
                crate::physics::RoverBody::default(),
                Transform::from_xyz(0.0, 0.1, 0.0),
                DriveCommand::default(),
                crate::terra::WheelSpeeds::default(),
                crate::terra::Odometry::default(),
            ))
            .id();
        let deadline = Instant::now() + Duration::from_secs(10);
        while !app
            .world()
            .resource::<Bridge>()
            .shared
            .connected
            .load(Ordering::Acquire)
        {
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(5));
        }
        app.update();
        let client =
            terra_transport::RoverConnection::connect(&endpoint, "terra/rover", 9).unwrap();
        for _ in 0..700 {
            client.set_target(1.0, 0.2).unwrap();
            app.update();
            std::thread::sleep(Duration::from_millis(1));
        }
        let command = app.world().get::<DriveCommand>(rover).unwrap();
        assert_eq!(command.linear, 1.0);
        assert_eq!(command.angular, 0.2);
        assert!(
            app.world()
                .get::<Position>(rover)
                .unwrap()
                .0
                .with_y(0.0)
                .length()
                > 1.0
        );
        assert!(app.world().get::<LinearVelocity>(rover).unwrap().length() > 0.5);
        client.disconnect();
        for _ in 0..600 {
            app.update();
            std::thread::sleep(Duration::from_millis(1));
        }
        assert_eq!(app.world().get::<DriveCommand>(rover).unwrap().linear, 0.0);
        assert!(app.world().get::<LinearVelocity>(rover).unwrap().length() < 0.05);
        assert!(app.world().get::<AngularVelocity>(rover).unwrap().y.abs() < 0.05);
        let shared = app.world().resource::<Bridge>().shared.clone();
        drop(app);
        while shared.connected.load(Ordering::Acquire) && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(5));
        }
        assert!(!shared.connected.load(Ordering::Acquire));
    }
}
