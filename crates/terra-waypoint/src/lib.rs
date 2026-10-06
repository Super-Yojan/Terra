//! Lat/lon go-to-waypoint follower. The phone runs this through `terra-mobile`.
//! The simulator calls the same type; it does not keep a second controller.
use serde::Serialize;
use terra_types::InputError;

pub const MAX_GOAL_BYTES: usize = 2048;
const METRES_PER_DEGREE: f64 = 40_075_016.686 / 360.0;

/// Map origin. Goals in WGS84 are projected into robotics metres: +x north, +y west.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeoOrigin {
    pub latitude: f64,
    pub longitude: f64,
}

impl GeoOrigin {
    pub fn new(latitude: f64, longitude: f64) -> Result<Self, InputError> {
        if !latitude.is_finite()
            || !longitude.is_finite()
            || !(-85.0..=85.0).contains(&latitude)
            || !(-180.0..=180.0).contains(&longitude)
        {
            return Err(InputError::OutOfRange);
        }
        Ok(Self {
            latitude,
            longitude,
        })
    }

    pub fn to_local(self, latitude: f64, longitude: f64) -> (f64, f64) {
        let north = (latitude - self.latitude) * METRES_PER_DEGREE;
        let east =
            (longitude - self.longitude) * METRES_PER_DEGREE * self.latitude.to_radians().cos();
        (north, -east)
    }

    /// Inverse of [`Self::to_local`]. `x` is metres north, `y` is metres west.
    pub fn from_local(self, x: f64, y: f64) -> (f64, f64) {
        let east = -y;
        let latitude = self.latitude + x / METRES_PER_DEGREE;
        let cos = self.latitude.to_radians().cos();
        let longitude = if cos.abs() < 1e-8 {
            self.longitude
        } else {
            self.longitude + east / (METRES_PER_DEGREE * cos)
        };
        (latitude, longitude)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct WaypointConfig {
    pub cruise: f64,
    pub yaw_gain: f64,
    pub max_yaw_rate: f64,
    pub arrive_radius: f64,
    pub align_yaw: f64,
    pub slow_radius: f64,
    pub heading_gate: f64,
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

impl WaypointConfig {
    pub fn validate(&self) -> Result<(), InputError> {
        let positive = [
            self.cruise,
            self.yaw_gain,
            self.max_yaw_rate,
            self.arrive_radius,
            self.slow_radius,
            self.heading_gate,
        ];
        if positive
            .iter()
            .any(|value| !value.is_finite() || *value <= 0.0)
            || !self.align_yaw.is_finite()
            || self.align_yaw < 0.0
            || self.cruise > 5.0
            || self.max_yaw_rate > 5.0
            || self.arrive_radius > 20.0
            || self.heading_gate > std::f64::consts::PI
        {
            return Err(InputError::InvalidConfiguration);
        }
        Ok(())
    }
}

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

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum GoalState {
    Idle,
    Active,
    Arrived,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct WaypointStatus {
    pub state: GoalState,
    pub goal_id: u64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub token: Option<String>,
    pub distance: f64,
    pub x: f64,
    pub y: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub yaw: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub latitude: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub longitude: Option<f64>,
}

impl WaypointStatus {
    pub fn idle() -> Self {
        Self {
            state: GoalState::Idle,
            goal_id: 0,
            token: None,
            distance: 0.0,
            x: 0.0,
            y: 0.0,
            yaw: None,
            latitude: None,
            longitude: None,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct WaypointOutput {
    pub linear: f64,
    pub angular: f64,
    pub status: WaypointStatus,
}

#[derive(Clone, Debug)]
struct LatchedGoal {
    x: f64,
    y: f64,
    yaw: Option<f64>,
    token: Option<String>,
    latitude: Option<f64>,
    longitude: Option<f64>,
}

pub struct WaypointController {
    config: WaypointConfig,
    origin: Option<GeoOrigin>,
    goal_id: u64,
    goal: Option<LatchedGoal>,
}

impl WaypointController {
    pub fn new(config: WaypointConfig) -> Result<Self, InputError> {
        config.validate()?;
        Ok(Self {
            config,
            origin: None,
            goal_id: 0,
            goal: None,
        })
    }

    pub fn set_origin(&mut self, latitude: f64, longitude: f64) -> Result<(), InputError> {
        self.origin = Some(GeoOrigin::new(latitude, longitude)?);
        Ok(())
    }

    /// Latch a decoded goal. `Ok(false)` leaves the current goal in place
    /// (no origin for a geographic goal, or the point is outside `half_extent`).
    pub fn accept(&mut self, command: &GoalCommand, half_extent: f64) -> Result<bool, InputError> {
        if !half_extent.is_finite() || half_extent <= 0.0 {
            return Err(InputError::InvalidConfiguration);
        }
        let latched = match command {
            GoalCommand::Cancel => {
                self.goal = None;
                self.goal_id = 0;
                return Ok(true);
            }
            GoalCommand::Local { x, y, yaw, token } => {
                if !x.is_finite() || !y.is_finite() || !inside_square(*x, *y, half_extent) {
                    return Ok(false);
                }
                LatchedGoal {
                    x: *x,
                    y: *y,
                    yaw: *yaw,
                    token: token.clone(),
                    latitude: None,
                    longitude: None,
                }
            }
            GoalCommand::Wgs84 {
                latitude,
                longitude,
                yaw,
                token,
            } => {
                let Some(origin) = self.origin else {
                    return Ok(false);
                };
                let (x, y) = origin.to_local(*latitude, *longitude);
                if !x.is_finite() || !y.is_finite() || !inside_square(x, y, half_extent) {
                    return Ok(false);
                }
                LatchedGoal {
                    x,
                    y,
                    yaw: *yaw,
                    token: token.clone(),
                    latitude: Some(*latitude),
                    longitude: Some(*longitude),
                }
            }
        };
        self.goal_id = self.goal_id.saturating_add(1).max(1);
        self.goal = Some(latched);
        Ok(true)
    }

    pub fn cancel(&mut self) {
        self.goal = None;
        self.goal_id = 0;
    }

    /// `x` and `y` are the rover pose in the origin frame, metres. `yaw` is radians, 0 facing +x.
    pub fn step(&self, x: f64, y: f64, yaw: f64) -> WaypointOutput {
        let Some(goal) = &self.goal else {
            return WaypointOutput {
                linear: 0.0,
                angular: 0.0,
                status: WaypointStatus::idle(),
            };
        };
        let pursued = pursuit(x, y, yaw, goal.x, goal.y, goal.yaw, &self.config);
        WaypointOutput {
            linear: pursued.linear,
            angular: pursued.angular,
            status: WaypointStatus {
                state: if pursued.arrived {
                    GoalState::Arrived
                } else {
                    GoalState::Active
                },
                goal_id: self.goal_id,
                token: goal.token.clone(),
                distance: pursued.distance,
                x: goal.x,
                y: goal.y,
                yaw: goal.yaw,
                latitude: goal.latitude,
                longitude: goal.longitude,
            },
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct Pursuit {
    linear: f64,
    angular: f64,
    distance: f64,
    arrived: bool,
}

fn pursuit(
    x: f64,
    y: f64,
    yaw: f64,
    goal_x: f64,
    goal_y: f64,
    goal_yaw: Option<f64>,
    config: &WaypointConfig,
) -> Pursuit {
    let dx = goal_x - x;
    let dy = goal_y - y;
    let distance = dx.hypot(dy);
    if !distance.is_finite() || !yaw.is_finite() {
        return Pursuit {
            linear: 0.0,
            angular: 0.0,
            distance: 0.0,
            arrived: false,
        };
    }
    if distance <= config.arrive_radius {
        let Some(target) = goal_yaw else {
            return Pursuit {
                linear: 0.0,
                angular: 0.0,
                distance,
                arrived: true,
            };
        };
        let error = wrap(target - yaw);
        if error.abs() <= config.align_yaw {
            return Pursuit {
                linear: 0.0,
                angular: 0.0,
                distance,
                arrived: true,
            };
        }
        return Pursuit {
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
    Pursuit {
        linear,
        angular,
        distance,
        arrived: false,
    }
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

pub fn encode_status(status: &WaypointStatus) -> Option<Vec<u8>> {
    let bytes = serde_json::to_vec(status).ok()?;
    (bytes.len() <= 65_536).then_some(bytes)
}

fn inside_square(x: f64, y: f64, half_extent: f64) -> bool {
    let bevy_x = -y;
    let bevy_z = -x;
    bevy_x.abs() <= half_extent && bevy_z.abs() <= half_extent
}

fn wrap(angle: f64) -> f64 {
    (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
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
            Some(Some(wrap(yaw)))
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
    fn local_metres_round_trip_through_the_johnson_center() {
        let origin = GeoOrigin::new(38.8297, -77.3075).unwrap();
        let (x, y) = origin.to_local(38.82981, -77.3075);
        assert!(x > 12.0 && x < 12.5, "{x}");
        assert!(y.abs() < 1e-6, "{y}");
        let (latitude, longitude) = origin.from_local(x, y);
        assert!((latitude - 38.82981).abs() < 1e-9, "{latitude}");
        assert!((longitude + 77.3075).abs() < 1e-9, "{longitude}");
    }

    #[test]
    fn wgs84_uses_the_origin_and_local_goals_stay_inside_the_square() {
        let mut follower = WaypointController::new(WaypointConfig::default()).unwrap();
        let near =
            decode_goal(br#"{"frame":"wgs84","latitude":38.8299,"longitude":-77.3075}"#).unwrap();
        assert!(!follower.accept(&near, 49.0).unwrap());
        follower.set_origin(38.8297, -77.3075).unwrap();
        assert!(follower.accept(&near, 49.0).unwrap());
        let status = follower.step(0.0, 0.0, 0.0).status;
        assert!(status.x > 10.0 && status.x < 30.0, "{}", status.x);
        assert!(status.y.abs() < 1.0, "{}", status.y);
        assert_eq!(status.latitude, Some(38.8299));
        assert_eq!(status.longitude, Some(-77.3075));
        let far =
            decode_goal(br#"{"frame":"wgs84","latitude":38.90,"longitude":-77.3075}"#).unwrap();
        assert!(!follower.accept(&far, 49.0).unwrap());
        assert_eq!(follower.step(0.0, 0.0, 0.0).status.goal_id, 1);
        let outside = decode_goal(br#"{"frame":"local","x":80,"y":0}"#).unwrap();
        assert!(!follower.accept(&outside, 49.0).unwrap());
        let inside = decode_goal(br#"{"frame":"local","x":10,"y":-4,"yaw":0.5}"#).unwrap();
        assert!(follower.accept(&inside, 49.0).unwrap());
        let status = follower.step(0.0, 0.0, 0.0).status;
        assert_eq!(status.x, 10.0);
        assert_eq!(status.y, -4.0);
        assert!(status.latitude.is_none());
    }

    #[test]
    fn pursuit_faces_the_goal_and_a_unicycle_arrives() {
        let follower = WaypointController::new(WaypointConfig::default()).unwrap();
        let ahead = pursuit(0.0, 0.0, 0.0, 10.0, 0.0, None, &follower.config);
        assert!(ahead.linear > 0.8, "{}", ahead.linear);
        assert!(ahead.angular.abs() < 1e-4);
        assert!(!ahead.arrived);
        let left = pursuit(0.0, 0.0, 0.0, 0.0, 10.0, None, &follower.config);
        assert_eq!(left.linear, 0.0);
        assert!(left.angular > 0.5, "{}", left.angular);

        let mut follower = WaypointController::new(WaypointConfig::default()).unwrap();
        follower
            .accept(
                &GoalCommand::Local {
                    x: 8.0,
                    y: -5.0,
                    yaw: Some(0.4),
                    token: None,
                },
                49.0,
            )
            .unwrap();
        let (mut x, mut y, mut yaw) = (0.0, 0.0, 2.0);
        let mut done = false;
        for _ in 0..800 {
            let output = follower.step(x, y, yaw);
            if output.status.state == GoalState::Arrived {
                done = true;
                break;
            }
            let dt = 0.05;
            yaw = wrap(yaw + output.angular * dt);
            x += output.linear * dt * yaw.cos();
            y += output.linear * dt * yaw.sin();
        }
        assert!(done, "unicycle did not arrive, pose=({x},{y},{yaw})");
    }

    #[test]
    fn status_json_reports_the_geographic_goal() {
        let mut follower = WaypointController::new(WaypointConfig::default()).unwrap();
        follower.set_origin(38.8297, -77.3075).unwrap();
        follower
            .accept(
                &decode_goal(
                    br#"{"frame":"wgs84","latitude":38.82981,"longitude":-77.3075,"token":"gmu-north"}"#,
                )
                .unwrap(),
                49.0,
            )
            .unwrap();
        let bytes = encode_status(&follower.step(0.0, 0.0, 0.0).status).unwrap();
        let value: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(value["state"], "active");
        assert_eq!(value["goal_id"], 1);
        assert_eq!(value["token"], "gmu-north");
        assert_eq!(value["latitude"], 38.82981);
        assert_eq!(value["longitude"], -77.3075);
        assert!(value["x"].as_f64().unwrap() > 10.0);
        assert!(value.get("yaw").is_none());
        let idle: serde_json::Value =
            serde_json::from_slice(&encode_status(&WaypointStatus::idle()).unwrap()).unwrap();
        assert_eq!(idle["state"], "idle");
        assert_eq!(idle["goal_id"], 0);
        assert!(idle.get("latitude").is_none());
    }
}
