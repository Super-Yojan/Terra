//! High-level go-to-waypoint command. ARGOS publishes one goal; Terra's
//! existing velocity loop tracks the twist this module produces.

use bevy::prelude::*;
use serde::Serialize;

pub const MAX_GOAL_BYTES: usize = 2048;

#[derive(Clone, Debug, PartialEq)]
pub enum GoalCommand {
    Cancel,
    Local {
        x: f64,
        y: f64,
        yaw: Option<f64>,
        token: Option<String>,
    },
    Wgs84 {
        latitude: f64,
        longitude: f64,
        yaw: Option<f64>,
        token: Option<String>,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub struct ResolvedGoal {
    pub x: f32,
    pub y: f32,
    pub yaw: Option<f32>,
    pub token: Option<String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum GoalState {
    Idle,
    Active,
    Arrived,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct GoalStatus {
    pub state: GoalState,
    pub goal_id: u64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub token: Option<String>,
    pub distance: f64,
    pub x: f64,
    pub y: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub yaw: Option<f64>,
}

impl GoalStatus {
    pub fn idle() -> Self {
        Self {
            state: GoalState::Idle,
            goal_id: 0,
            token: None,
            distance: 0.0,
            x: 0.0,
            y: 0.0,
            yaw: None,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct WaypointConfig {
    pub cruise: f32,
    pub yaw_gain: f32,
    pub max_yaw_rate: f32,
    pub arrive_radius: f32,
    pub align_yaw: f32,
    pub slow_radius: f32,
    pub heading_gate: f32,
}

impl Default for WaypointConfig {
    fn default() -> Self {
        Self {
            cruise: 1.0,
            yaw_gain: 1.6,
            max_yaw_rate: 1.2,
            arrive_radius: 0.75,
            align_yaw: 0.12,
            slow_radius: 3.0,
            heading_gate: 0.55,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct PursuitOutput {
    pub linear: f32,
    pub angular: f32,
    pub distance: f32,
    pub arrived: bool,
}

/// Pose in the robotics frame shared with depth `body`: `x = -Bevy Z`,
/// `y = -Bevy X`, yaw 0 faces +x (Bevy −Z).
pub fn robotics_pose(translation: Vec3, rotation: Quat) -> (f32, f32, f32) {
    let forward = rotation * Vec3::NEG_Z;
    let rx = -forward.z;
    let ry = -forward.x;
    (-translation.z, -translation.x, ry.atan2(rx))
}

pub fn decode_goal(bytes: &[u8]) -> Option<GoalCommand> {
    if bytes.is_empty() || bytes.len() > MAX_GOAL_BYTES {
        return None;
    }
    let value: serde_json::Value = serde_json::from_slice(bytes).ok()?;
    let object = value.as_object()?;
    if object.contains_key("cancel") {
        if object.len() == 1 && object.get("cancel") == Some(&serde_json::Value::Bool(true)) {
            return Some(GoalCommand::Cancel);
        }
        return None;
    }
    let frame = object.get("frame")?.as_str()?;
    let yaw = optional_yaw(object)?;
    let token = optional_token(object)?;
    match frame {
        "local" => {
            if !allowed(object, &["frame", "x", "y"], &["yaw", "token"]) {
                return None;
            }
            let x = finite(object.get("x")?)?;
            let y = finite(object.get("y")?)?;
            if x.abs() > 20_000.0 || y.abs() > 20_000.0 {
                return None;
            }
            Some(GoalCommand::Local { x, y, yaw, token })
        }
        "wgs84" => {
            if !allowed(
                object,
                &["frame", "latitude", "longitude"],
                &["yaw", "token"],
            ) {
                return None;
            }
            let latitude = finite(object.get("latitude")?)?;
            let longitude = finite(object.get("longitude")?)?;
            if !(-85.0..=85.0).contains(&latitude) || !(-180.0..=180.0).contains(&longitude) {
                return None;
            }
            Some(GoalCommand::Wgs84 {
                latitude,
                longitude,
                yaw,
                token,
            })
        }
        _ => None,
    }
}

pub fn resolve_goal(
    command: &GoalCommand,
    anchor: Option<&crate::geo::GeoAnchor>,
    half_extent: f64,
) -> Option<ResolvedGoal> {
    let (x, y, yaw, token) = match command {
        GoalCommand::Cancel => return None,
        GoalCommand::Local { x, y, yaw, token } => (*x, *y, *yaw, token.clone()),
        GoalCommand::Wgs84 {
            latitude,
            longitude,
            yaw,
            token,
        } => {
            let (x, y) = anchor?.local_xy(*latitude, *longitude);
            (x, y, *yaw, token.clone())
        }
    };
    if !x.is_finite() || !y.is_finite() || !inside_square(x, y, half_extent) {
        return None;
    }
    Some(ResolvedGoal {
        x: x as f32,
        y: y as f32,
        yaw: yaw.map(|yaw| yaw as f32),
        token,
    })
}

pub fn pursuit(
    x: f32,
    y: f32,
    yaw: f32,
    goal_x: f32,
    goal_y: f32,
    goal_yaw: Option<f32>,
    config: &WaypointConfig,
) -> PursuitOutput {
    let dx = goal_x - x;
    let dy = goal_y - y;
    let distance = dx.hypot(dy);
    if !distance.is_finite() || !yaw.is_finite() {
        return PursuitOutput {
            linear: 0.0,
            angular: 0.0,
            distance: 0.0,
            arrived: false,
        };
    }
    if distance <= config.arrive_radius {
        let Some(target) = goal_yaw else {
            return PursuitOutput {
                linear: 0.0,
                angular: 0.0,
                distance,
                arrived: true,
            };
        };
        let error = wrap(target - yaw);
        if error.abs() <= config.align_yaw {
            return PursuitOutput {
                linear: 0.0,
                angular: 0.0,
                distance,
                arrived: true,
            };
        }
        return PursuitOutput {
            linear: 0.0,
            angular: (error * config.yaw_gain).clamp(-config.max_yaw_rate, config.max_yaw_rate),
            distance,
            arrived: false,
        };
    }
    let error = wrap(dy.atan2(dx) - yaw);
    let angular = (error * config.yaw_gain).clamp(-config.max_yaw_rate, config.max_yaw_rate);
    let linear = if error.abs() >= config.heading_gate {
        0.0
    } else {
        let aligned = 1.0 - error.abs() / config.heading_gate;
        let proximity = if distance < config.slow_radius {
            (distance / config.slow_radius).max(0.2)
        } else {
            1.0
        };
        config.cruise * aligned * proximity
    };
    PursuitOutput {
        linear,
        angular,
        distance,
        arrived: false,
    }
}

pub fn encode_status(status: &GoalStatus) -> Option<Vec<u8>> {
    let bytes = serde_json::to_vec(status).ok()?;
    (bytes.len() <= 65_536).then_some(bytes)
}

fn inside_square(x: f64, y: f64, half_extent: f64) -> bool {
    let bevy_x = -y;
    let bevy_z = -x;
    bevy_x.abs() <= half_extent && bevy_z.abs() <= half_extent
}

fn wrap(angle: f32) -> f32 {
    (angle + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}

fn finite(value: &serde_json::Value) -> Option<f64> {
    let number = value.as_f64()?;
    number.is_finite().then_some(number)
}

fn optional_yaw(object: &serde_json::Map<String, serde_json::Value>) -> Option<Option<f64>> {
    match object.get("yaw") {
        None | Some(serde_json::Value::Null) => Some(None),
        Some(value) => {
            let yaw = finite(value)?;
            if yaw.abs() > std::f64::consts::TAU {
                return None;
            }
            let wrapped = (yaw + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                - std::f64::consts::PI;
            Some(Some(wrapped))
        }
    }
}

fn optional_token(object: &serde_json::Map<String, serde_json::Value>) -> Option<Option<String>> {
    match object.get("token") {
        None | Some(serde_json::Value::Null) => Some(None),
        Some(serde_json::Value::String(token)) => valid_token(token).then(|| Some(token.clone())),
        Some(_) => None,
    }
}

fn valid_token(token: &str) -> bool {
    let bytes = token.as_bytes();
    (1..=64).contains(&bytes.len())
        && bytes
            .iter()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'_' | b':' | b'-'))
}

fn allowed(
    object: &serde_json::Map<String, serde_json::Value>,
    required: &[&str],
    optional: &[&str],
) -> bool {
    required.iter().all(|key| object.contains_key(*key))
        && object
            .keys()
            .all(|key| required.contains(&key.as_str()) || optional.contains(&key.as_str()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_local_wgs84_and_cancel_and_rejects_the_rest() {
        let local =
            decode_goal(br#"{"frame":"local","x":10.0,"y":-2.5,"token":"goal-1"}"#).unwrap();
        assert_eq!(
            local,
            GoalCommand::Local {
                x: 10.0,
                y: -2.5,
                yaw: None,
                token: Some("goal-1".into()),
            }
        );
        let aimed = decode_goal(br#"{"frame":"local","x":1,"y":2,"yaw":1.2}"#).unwrap();
        match aimed {
            GoalCommand::Local { yaw, .. } => assert!((yaw.unwrap() - 1.2).abs() < 1e-9),
            other => panic!("{other:?}"),
        }
        let geo =
            decode_goal(br#"{"frame":"wgs84","latitude":38.8299,"longitude":-77.3075,"yaw":null}"#)
                .unwrap();
        assert!(matches!(geo, GoalCommand::Wgs84 { .. }));
        assert_eq!(
            decode_goal(br#"{"cancel":true}"#),
            Some(GoalCommand::Cancel)
        );
        for bytes in [
            br#"{"cancel":false}"#.as_slice(),
            br#"{"cancel":true,"frame":"local","x":1,"y":2}"#,
            br#"{"frame":"local","x":1}"#,
            br#"{"frame":"local","x":1,"y":2,"linear":1}"#,
            br#"{"frame":"wgs84","latitude":91,"longitude":0}"#,
            br#"{"frame":"local","x":1,"y":"north"}"#,
            br#"{"frame":"utm","x":1,"y":2}"#,
            b"",
        ] {
            assert!(
                decode_goal(bytes).is_none(),
                "{}",
                String::from_utf8_lossy(bytes)
            );
        }
        assert!(decode_goal(&vec![b'{'; 2049]).is_none());
    }

    #[test]
    fn wgs84_uses_the_anchor_and_local_goals_stay_inside_the_square() {
        let anchor = crate::geo::GeoAnchor::try_new(
            crate::geo::DEFAULT_LATITUDE,
            crate::geo::DEFAULT_LONGITUDE,
            15,
        )
        .unwrap();
        let near =
            decode_goal(br#"{"frame":"wgs84","latitude":38.8299,"longitude":-77.3075}"#).unwrap();
        let resolved = resolve_goal(&near, Some(&anchor), 49.0).unwrap();
        assert!(resolved.x > 10.0 && resolved.x < 30.0, "{}", resolved.x);
        assert!(resolved.y.abs() < 1.0, "{}", resolved.y);
        let far =
            decode_goal(br#"{"frame":"wgs84","latitude":38.90,"longitude":-77.3075}"#).unwrap();
        assert!(resolve_goal(&far, Some(&anchor), 49.0).is_none());
        assert!(resolve_goal(&far, None, 49.0).is_none());
        let outside = decode_goal(br#"{"frame":"local","x":80,"y":0}"#).unwrap();
        assert!(resolve_goal(&outside, None, 49.0).is_none());
        let inside = decode_goal(br#"{"frame":"local","x":10,"y":-4,"yaw":0.5}"#).unwrap();
        let resolved = resolve_goal(&inside, None, 49.0).unwrap();
        assert_eq!(resolved.x, 10.0);
        assert_eq!(resolved.y, -4.0);
    }

    #[test]
    fn pursuit_faces_the_goal_and_a_unicycle_arrives() {
        let config = WaypointConfig::default();
        let ahead = pursuit(0.0, 0.0, 0.0, 10.0, 0.0, None, &config);
        assert!(ahead.linear > 0.8, "{}", ahead.linear);
        assert!(ahead.angular.abs() < 1e-4);
        assert!(!ahead.arrived);
        let left = pursuit(0.0, 0.0, 0.0, 0.0, 10.0, None, &config);
        assert_eq!(left.linear, 0.0);
        assert!(left.angular > 0.5, "{}", left.angular);
        let arrived = pursuit(0.1, -0.1, 0.2, 0.0, 0.0, None, &config);
        assert!(arrived.arrived && arrived.linear == 0.0 && arrived.angular == 0.0);
        let spin = pursuit(0.0, 0.0, 0.0, 0.0, 0.0, Some(1.0), &config);
        assert!(!spin.arrived && spin.linear == 0.0 && spin.angular > 0.0);

        let (goal_x, goal_y) = (8.0_f32, -5.0);
        let (mut x, mut y, mut yaw) = (0.0_f32, 0.0, 2.0);
        let mut done = false;
        for _ in 0..800 {
            let output = pursuit(x, y, yaw, goal_x, goal_y, Some(0.4), &config);
            if output.arrived {
                done = true;
                break;
            }
            let dt = 0.05;
            yaw = wrap(yaw + output.angular * dt);
            x += output.linear * dt * yaw.cos();
            y += output.linear * dt * yaw.sin();
        }
        assert!(done, "unicycle did not arrive, pose=({x},{y},{yaw})");
        let pose = robotics_pose(Vec3::new(2.0, 0.4, -3.0), Quat::IDENTITY);
        assert!((pose.0 - 3.0).abs() < 1e-5 && (pose.1 + 2.0).abs() < 1e-5);
        assert!(pose.2.abs() < 1e-5);
    }

    #[test]
    fn status_json_reports_progress_fields() {
        let bytes = encode_status(&GoalStatus {
            state: GoalState::Active,
            goal_id: 4,
            token: Some("goal-1".into()),
            distance: 3.5,
            x: 10.0,
            y: -2.0,
            yaw: None,
        })
        .unwrap();
        let value: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(value["state"], "active");
        assert_eq!(value["goal_id"], 4);
        assert_eq!(value["token"], "goal-1");
        assert_eq!(value["distance"], 3.5);
        assert!(value.get("yaw").is_none());
        let idle: serde_json::Value =
            serde_json::from_slice(&encode_status(&GoalStatus::idle()).unwrap()).unwrap();
        assert_eq!(idle["state"], "idle");
        assert_eq!(idle["goal_id"], 0);
    }
}
